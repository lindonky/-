#include "training_session.h"

#ifdef TRAINING_SESSION_HOST_TEST
/* Replay test is single-threaded. Production firmware uses a task mutex. */
#ifndef CONFIG_TRAIN_LIFT_START_DEG
#define CONFIG_TRAIN_LIFT_START_DEG 8
#define CONFIG_TRAIN_RETURN_WINDOW_DEG 5
#define CONFIG_TRAIN_ROM_MIN_DEG 15
#define CONFIG_TRAIN_ROM_MAX_DEG 45
#define CONFIG_TRAIN_ROM_LIMIT_DEG 60
#define CONFIG_TRAIN_SPEED_MIN_DPS 10
#define CONFIG_TRAIN_SPEED_MAX_DPS 90
#define CONFIG_TRAIN_CYCLE_MIN_MS 800
#define CONFIG_TRAIN_CYCLE_MAX_MS 8000
#define CONFIG_TRAIN_LATERAL_MAX_DEG 8
#endif
#define TRAIN_LOCK() ((void)0)
#define TRAIN_UNLOCK() ((void)0)
#else
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#define TRAIN_LOCK() xSemaphoreTake(s_trainMutex, portMAX_DELAY)
#define TRAIN_UNLOCK() xSemaphoreGive(s_trainMutex)
#endif

#include <math.h>
#include <stdio.h>
#include <string.h>

#define TRAIN_HEIGHT_MIN_CM          100.0f
#define TRAIN_HEIGHT_MAX_CM          230.0f
#define TRAIN_LEG_HEIGHT_RATIO       0.5199f /* 中国成人：大腿28.13% + 小腿23.86% */
#define TRAIN_SHANK_HEIGHT_RATIO     0.2386f
#define TRAIN_PI                     3.14159265358979323846f

#define TRAIN_LIFT_START_DEG         ((float)CONFIG_TRAIN_LIFT_START_DEG)
#define TRAIN_RETURN_WINDOW_DEG      ((float)CONFIG_TRAIN_RETURN_WINDOW_DEG)
#define TRAIN_RETURN_COMPLETE_DEG    TRAIN_LIFT_START_DEG
#define TRAIN_PEAK_DROP_DEG          2.0f
#define TRAIN_TARGET_ROM_MIN_DEG     ((float)CONFIG_TRAIN_ROM_MIN_DEG)
#define TRAIN_TARGET_ROM_MAX_DEG     ((float)CONFIG_TRAIN_ROM_MAX_DEG)
#define TRAIN_ABSOLUTE_ROM_MAX_DEG   ((float)CONFIG_TRAIN_ROM_LIMIT_DEG)
#define TRAIN_SPEED_MIN_DPS          ((float)CONFIG_TRAIN_SPEED_MIN_DPS)
#define TRAIN_SPEED_MAX_DPS          ((float)CONFIG_TRAIN_SPEED_MAX_DPS)
#define TRAIN_CYCLE_MIN_MS           ((uint32_t)CONFIG_TRAIN_CYCLE_MIN_MS)
#define TRAIN_CYCLE_MAX_MS           ((uint32_t)CONFIG_TRAIN_CYCLE_MAX_MS)
#define TRAIN_SETTLE_HOLD_MS         200U
#define TRAIN_LATERAL_TARGET_DEG     ((float)CONFIG_TRAIN_LATERAL_MAX_DEG)
#define TRAIN_GOAL_ROM_SPAN_MIN_DEG  2.0f
#define TRAIN_GOAL_CADENCE_MIN_SPM   5.0f
#define TRAIN_GOAL_CADENCE_MAX_SPM   75.0f
#define TRAIN_GOAL_CADENCE_DEFAULT   30.0f
#define TRAIN_GOAL_CADENCE_TOL_PCT   0.15f
#define TRAIN_GOAL_CADENCE_TOL_MIN   3.0f

/* 初始跌倒候选阈值。传感器位于小腿且工作于水下，必须用实测数据重标定。
 * 只有“低加速度/冲击 + 高角速度 + 大姿态变化 + 随后稳定”组合才会确认。 */
#define FALL_LOW_ACCEL_G             0.60f
#define FALL_IMPACT_ACCEL_G          2.20f
#define FALL_TRIGGER_GYRO_DPS        100.0f
#define FALL_POSTURE_TILT_DEG        45.0f
#define FALL_STABLE_GYRO_DPS         18.0f
#define FALL_STABLE_ACCEL_MIN_G      0.70f
#define FALL_STABLE_ACCEL_MAX_G      1.35f
#define FALL_IMPACT_WINDOW_MS        900U
#define FALL_CONFIRM_WINDOW_MS       2500U
#define FALL_STABLE_HOLD_MS          800U

#if CONFIG_TRAIN_ROM_MIN_DEG > CONFIG_TRAIN_ROM_MAX_DEG
#error "TRAIN_ROM_MIN_DEG must not exceed TRAIN_ROM_MAX_DEG"
#endif
#if CONFIG_TRAIN_ROM_MAX_DEG > CONFIG_TRAIN_ROM_LIMIT_DEG
#error "TRAIN_ROM_MAX_DEG must not exceed TRAIN_ROM_LIMIT_DEG"
#endif
#if CONFIG_TRAIN_CYCLE_MIN_MS >= CONFIG_TRAIN_CYCLE_MAX_MS
#error "TRAIN_CYCLE_MIN_MS must be less than TRAIN_CYCLE_MAX_MS"
#endif

typedef struct {
    training_status_t pub;
    uint32_t nextSessionId;
    uint32_t startedMs;
    uint32_t pauseStartedMs;
    uint32_t pausedTotalMs;
    uint32_t phaseStartedMs;
    uint32_t peakMs;
    uint32_t settleStartedMs;
    uint32_t lastFeedMs;
    uint32_t lastStepMs;
    float lastPitchDeg;
    float peakExcursionDeg;
    float peakSpeedDps;
    float peakLateralDeg;
    float interventionTimeMs;
    float correctionLoad;
    float measuredShankCm;
    uint32_t fallStageStartedMs;
    uint32_t fallStableStartedMs;
    float fallReferencePitchDeg;
    float fallReferenceRollDeg;
    bool fallPending;
} training_ctx_t;

static training_ctx_t s_train;
#ifndef TRAINING_SESSION_HOST_TEST
static SemaphoreHandle_t s_trainMutex;
#endif

static float absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

static float maxf(float a, float b)
{
    return (a > b) ? a : b;
}

static float vector3_magnitude(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static void set_event_locked(const char *event)
{
    snprintf(s_train.pub.lastEvent, sizeof(s_train.pub.lastEvent), "%s", event);
}

static void update_body_estimates_locked(void)
{
    s_train.pub.estimatedLegCm = s_train.pub.heightCm * TRAIN_LEG_HEIGHT_RATIO;
    if (s_train.measuredShankCm > 0.0f) {
        s_train.pub.shankLengthCm = s_train.measuredShankCm;
        s_train.pub.shankLengthMeasured = true;
    } else {
        s_train.pub.shankLengthCm = s_train.pub.heightCm * TRAIN_SHANK_HEIGHT_RATIO;
        s_train.pub.shankLengthMeasured = false;
    }
    s_train.pub.lastEstimatedStepCm = (s_train.pub.shankLengthCm > 0.0f)
        ? 2.0f * s_train.pub.shankLengthCm *
              sinf(s_train.pub.lastRomDeg * TRAIN_PI / 360.0f)
        : 0.0f;
    s_train.pub.averageEstimatedStepCm = (s_train.pub.shankLengthCm > 0.0f)
        ? 2.0f * s_train.pub.shankLengthCm *
              sinf(s_train.pub.averageRomDeg * TRAIN_PI / 360.0f)
        : 0.0f;
}

static void set_default_goal_locked(void)
{
    s_train.pub.goalEnabled = false;
    s_train.pub.goalRomMinDeg = TRAIN_TARGET_ROM_MIN_DEG;
    s_train.pub.goalRomMaxDeg = TRAIN_TARGET_ROM_MAX_DEG;
    s_train.pub.goalCadenceSpm = TRAIN_GOAL_CADENCE_DEFAULT;
    s_train.pub.goalCadenceToleranceSpm =
        maxf(TRAIN_GOAL_CADENCE_TOL_MIN,
             TRAIN_GOAL_CADENCE_DEFAULT * TRAIN_GOAL_CADENCE_TOL_PCT);
}

static void clear_cycle_locked(void)
{
    s_train.pub.phase = TRAIN_PHASE_SETTLED;
    s_train.pub.currentExcursionDeg = 0.0f;
    s_train.phaseStartedMs = 0U;
    s_train.peakMs = 0U;
    s_train.settleStartedMs = 0U;
    s_train.peakExcursionDeg = 0.0f;
    s_train.peakSpeedDps = 0.0f;
    s_train.peakLateralDeg = 0.0f;
}

static uint32_t elapsed_locked(uint32_t nowMs)
{
    if (s_train.pub.state == TRAIN_STATE_IDLE || s_train.startedMs == 0U) return 0U;
    uint32_t endMs = nowMs;
    if (s_train.pub.state == TRAIN_STATE_PAUSED) endMs = s_train.pauseStartedMs;
    if (s_train.pub.state == TRAIN_STATE_FINISHED) return s_train.pub.elapsedMs;
    return endMs - s_train.startedMs - s_train.pausedTotalMs;
}

static void reset_fall_locked(void)
{
    s_train.pub.fallStage = 0U;
    s_train.fallStageStartedMs = 0U;
    s_train.fallStableStartedMs = 0U;
}

static void finish_cycle_locked(const training_sample_t *s, float excursion,
                                uint32_t endMs)
{
    const uint32_t cycleMs = endMs - s_train.phaseStartedMs;
    const uint32_t liftMs = (s_train.peakMs > s_train.phaseStartedMs)
                                ? s_train.peakMs - s_train.phaseStartedMs : 0U;
    const uint32_t returnMs = (endMs > s_train.peakMs) ? endMs - s_train.peakMs : 0U;
    const float rom = s_train.peakExcursionDeg;
    const float meanSpeed = (cycleMs > 0U) ? (2000.0f * rom / (float)cycleMs) : 0.0f;
    const float returnError = excursion;
    float instantCadence = 0.0f;
    if (s_train.lastStepMs != 0U && endMs > s_train.lastStepMs) {
        instantCadence = 60000.0f / (float)(endMs - s_train.lastStepMs);
    } else if (cycleMs > 0U) {
        instantCadence = 60000.0f / (float)cycleMs;
    }
    if (instantCadence > 0.0f) {
        s_train.pub.cadenceSpm = (s_train.pub.cadenceSpm == 0.0f)
                                     ? instantCadence
                                     : (0.75f * s_train.pub.cadenceSpm +
                                        0.25f * instantCadence);
    }
    uint32_t quality = 0U;

    const float romMin = s_train.pub.goalEnabled
                             ? s_train.pub.goalRomMinDeg : TRAIN_TARGET_ROM_MIN_DEG;
    const float romMax = s_train.pub.goalEnabled
                             ? s_train.pub.goalRomMaxDeg : TRAIN_TARGET_ROM_MAX_DEG;
    if (rom < romMin) quality |= TRAIN_QUALITY_ROM_LOW;
    if (rom > romMax) quality |= TRAIN_QUALITY_ROM_HIGH;
    if (meanSpeed < TRAIN_SPEED_MIN_DPS) quality |= TRAIN_QUALITY_SPEED_LOW;
    if (meanSpeed > TRAIN_SPEED_MAX_DPS) quality |= TRAIN_QUALITY_SPEED_HIGH;
    if (s_train.peakLateralDeg > TRAIN_LATERAL_TARGET_DEG) quality |= TRAIN_QUALITY_LATERAL;
    if (returnError > TRAIN_RETURN_WINDOW_DEG) quality |= TRAIN_QUALITY_RETURN;
    if (cycleMs < TRAIN_CYCLE_MIN_MS || cycleMs > TRAIN_CYCLE_MAX_MS) {
        quality |= TRAIN_QUALITY_CYCLE_TIME;
    }
    if (s_train.pub.goalEnabled && instantCadence > 0.0f) {
        const float cadenceLow = s_train.pub.goalCadenceSpm -
                                 s_train.pub.goalCadenceToleranceSpm;
        const float cadenceHigh = s_train.pub.goalCadenceSpm +
                                  s_train.pub.goalCadenceToleranceSpm;
        if (instantCadence < cadenceLow) quality |= TRAIN_QUALITY_CADENCE_LOW;
        if (instantCadence > cadenceHigh) quality |= TRAIN_QUALITY_CADENCE_HIGH;
    }

    s_train.pub.steps++;
    if (quality == 0U) s_train.pub.qualifiedSteps++;
    s_train.pub.lastQualityFlags = quality;
    s_train.pub.lastRomDeg = rom;
    s_train.pub.lastPeakSpeedDps = s_train.peakSpeedDps;
    s_train.pub.lastCycleSec = (float)cycleMs / 1000.0f;
    s_train.pub.lastLiftSec = (float)liftMs / 1000.0f;
    s_train.pub.lastReturnSec = (float)returnMs / 1000.0f;
    s_train.pub.lastLateralDeg = s_train.peakLateralDeg;
    s_train.pub.lastReturnErrorDeg = returnError;
    const float n = (float)s_train.pub.steps;
    s_train.pub.averageRomDeg += (rom - s_train.pub.averageRomDeg) / n;
    update_body_estimates_locked();
    s_train.pub.qualifiedPct = 100.0f * (float)s_train.pub.qualifiedSteps / n;
    s_train.lastStepMs = endMs;
    set_event_locked((quality == 0U) ? "step_qualified" : "step_completed");
    (void)s;
}

static void update_fall_locked(const training_sample_t *s, float accelMag,
                               float gyroMag)
{
    const float pitchTilt = absf(s->pitchDeg - s_train.fallReferencePitchDeg);
    const float rollTilt = absf(s->rollDeg - s_train.fallReferenceRollDeg);
    const float tilt = maxf(pitchTilt, rollTilt);
    s_train.pub.lastAccelMagnitudeG = accelMag;
    s_train.pub.lastGyroMagnitudeDps = gyroMag;
    s_train.pub.lastFallTiltDeg = tilt;

    switch (s_train.pub.fallStage) {
    case 0:
        if ((accelMag < FALL_LOW_ACCEL_G && gyroMag > FALL_TRIGGER_GYRO_DPS) ||
            (accelMag > FALL_IMPACT_ACCEL_G && gyroMag > FALL_TRIGGER_GYRO_DPS)) {
            s_train.pub.fallStage = (accelMag > FALL_IMPACT_ACCEL_G) ? 2U : 1U;
            s_train.fallStageStartedMs = s->nowMs;
            set_event_locked("fall_candidate");
        }
        break;
    case 1: /* 低加速度/快速旋转后等待冲击 */
        if (accelMag > FALL_IMPACT_ACCEL_G) {
            s_train.pub.fallStage = 2U;
            s_train.fallStageStartedMs = s->nowMs;
        } else if (tilt >= FALL_POSTURE_TILT_DEG &&
                   gyroMag <= FALL_STABLE_GYRO_DPS &&
                   accelMag >= FALL_STABLE_ACCEL_MIN_G &&
                   accelMag <= FALL_STABLE_ACCEL_MAX_G) {
            /* 水的缓冲可能没有陆地式强冲击：明显姿态改变后直接进入
             * 持续确认阶段，仍要求 0.8s 静止以抑制快速抬腿误报。 */
            s_train.pub.fallStage = 2U;
            s_train.fallStableStartedMs = s->nowMs;
        } else if ((s->nowMs - s_train.fallStageStartedMs) > FALL_IMPACT_WINDOW_MS) {
            reset_fall_locked();
        }
        break;
    case 2: /* 冲击后等待显著姿态改变并趋于静止 */
        if ((s->nowMs - s_train.fallStageStartedMs) > FALL_CONFIRM_WINDOW_MS) {
            reset_fall_locked();
        } else if (tilt >= FALL_POSTURE_TILT_DEG &&
                   gyroMag <= FALL_STABLE_GYRO_DPS &&
                   accelMag >= FALL_STABLE_ACCEL_MIN_G &&
                   accelMag <= FALL_STABLE_ACCEL_MAX_G) {
            if (s_train.fallStableStartedMs == 0U) {
                s_train.fallStableStartedMs = s->nowMs;
            } else if ((s->nowMs - s_train.fallStableStartedMs) >= FALL_STABLE_HOLD_MS) {
                s_train.pub.fallStage = 3U;
                s_train.pub.fallEvents++;
                s_train.fallPending = true;
                s_train.pub.state = TRAIN_STATE_PAUSED;
                s_train.pauseStartedMs = s->nowMs;
                clear_cycle_locked();
                set_event_locked("suspected_fall");
            }
        } else {
            s_train.fallStableStartedMs = 0U;
        }
        break;
    default:
        break;
    }
}

void training_session_init(void)
{
#ifndef TRAINING_SESSION_HOST_TEST
    if (s_trainMutex == NULL) {
        s_trainMutex = xSemaphoreCreateMutex();
        configASSERT(s_trainMutex != NULL);
    }
#endif
    TRAIN_LOCK();
    memset(&s_train, 0, sizeof(s_train));
    s_train.pub.state = TRAIN_STATE_IDLE;
    s_train.pub.phase = TRAIN_PHASE_SETTLED;
    s_train.nextSessionId = 1U;
    set_default_goal_locked();
    update_body_estimates_locked();
    set_event_locked("ready");
    TRAIN_UNLOCK();
}

bool training_session_start(uint32_t nowMs, float pitchDeg, float rollDeg,
                            int mode, bool imuValid, bool estop)
{
    if (!imuValid || estop) return false;
    TRAIN_LOCK();
    if (s_train.pub.state == TRAIN_STATE_RUNNING ||
        s_train.pub.state == TRAIN_STATE_PAUSED) {
        TRAIN_UNLOCK();
        return false;
    }
    const float height = s_train.pub.heightCm;
    const float measuredShank = s_train.measuredShankCm;
    const bool goalEnabled = s_train.pub.goalEnabled;
    const float goalRomMin = s_train.pub.goalRomMinDeg;
    const float goalRomMax = s_train.pub.goalRomMaxDeg;
    const float goalCadence = s_train.pub.goalCadenceSpm;
    const float goalCadenceTolerance = s_train.pub.goalCadenceToleranceSpm;
    const uint32_t nextId = s_train.nextSessionId;
    memset(&s_train, 0, sizeof(s_train));
    s_train.pub.heightCm = height;
    s_train.measuredShankCm = measuredShank;
    s_train.pub.goalEnabled = goalEnabled;
    s_train.pub.goalRomMinDeg = goalRomMin;
    s_train.pub.goalRomMaxDeg = goalRomMax;
    s_train.pub.goalCadenceSpm = goalCadence;
    s_train.pub.goalCadenceToleranceSpm = goalCadenceTolerance;
    s_train.nextSessionId = nextId + 1U;
    s_train.pub.sessionId = nextId;
    s_train.pub.state = TRAIN_STATE_RUNNING;
    s_train.pub.phase = TRAIN_PHASE_SETTLED;
    s_train.pub.mode = mode;
    s_train.pub.baselinePitchDeg = pitchDeg;
    s_train.pub.baselineRollDeg = rollDeg;
    s_train.startedMs = nowMs;
    s_train.lastFeedMs = nowMs;
    s_train.lastPitchDeg = pitchDeg;
    s_train.fallReferencePitchDeg = pitchDeg;
    s_train.fallReferenceRollDeg = rollDeg;
    update_body_estimates_locked();
    set_event_locked("session_started");
    TRAIN_UNLOCK();
    return true;
}

bool training_session_pause(uint32_t nowMs)
{
    bool ok = false;
    TRAIN_LOCK();
    if (s_train.pub.state == TRAIN_STATE_RUNNING) {
        s_train.pub.elapsedMs = elapsed_locked(nowMs);
        s_train.pub.state = TRAIN_STATE_PAUSED;
        s_train.pauseStartedMs = nowMs;
        clear_cycle_locked();
        set_event_locked("session_paused");
        ok = true;
    }
    TRAIN_UNLOCK();
    return ok;
}

bool training_session_resume(uint32_t nowMs, float pitchDeg, float rollDeg,
                             bool imuValid)
{
    if (!imuValid) return false;
    bool ok = false;
    TRAIN_LOCK();
    if (s_train.pub.state == TRAIN_STATE_PAUSED && !s_train.fallPending) {
        s_train.pausedTotalMs += nowMs - s_train.pauseStartedMs;
        s_train.pub.state = TRAIN_STATE_RUNNING;
        s_train.pub.baselinePitchDeg = pitchDeg;
        s_train.pub.baselineRollDeg = rollDeg;
        s_train.lastPitchDeg = pitchDeg;
        s_train.lastFeedMs = nowMs;
        s_train.fallReferencePitchDeg = pitchDeg;
        s_train.fallReferenceRollDeg = rollDeg;
        reset_fall_locked();
        clear_cycle_locked();
        set_event_locked("session_resumed");
        ok = true;
    }
    TRAIN_UNLOCK();
    return ok;
}

bool training_session_stop(uint32_t nowMs)
{
    bool ok = false;
    TRAIN_LOCK();
    if (s_train.pub.state == TRAIN_STATE_RUNNING || s_train.pub.state == TRAIN_STATE_PAUSED) {
        s_train.pub.elapsedMs = elapsed_locked(nowMs);
        s_train.pub.state = TRAIN_STATE_FINISHED;
        clear_cycle_locked();
        set_event_locked("session_finished");
        ok = true;
    }
    TRAIN_UNLOCK();
    return ok;
}

void training_session_reset(void)
{
    TRAIN_LOCK();
    const float height = s_train.pub.heightCm;
    const float measuredShank = s_train.measuredShankCm;
    const bool goalEnabled = s_train.pub.goalEnabled;
    const float goalRomMin = s_train.pub.goalRomMinDeg;
    const float goalRomMax = s_train.pub.goalRomMaxDeg;
    const float goalCadence = s_train.pub.goalCadenceSpm;
    const float goalCadenceTolerance = s_train.pub.goalCadenceToleranceSpm;
    const uint32_t nextId = s_train.nextSessionId;
    memset(&s_train, 0, sizeof(s_train));
    s_train.pub.heightCm = height;
    s_train.measuredShankCm = measuredShank;
    s_train.pub.goalEnabled = goalEnabled;
    s_train.pub.goalRomMinDeg = goalRomMin;
    s_train.pub.goalRomMaxDeg = goalRomMax;
    s_train.pub.goalCadenceSpm = goalCadence;
    s_train.pub.goalCadenceToleranceSpm = goalCadenceTolerance;
    s_train.nextSessionId = nextId;
    s_train.pub.state = TRAIN_STATE_IDLE;
    s_train.pub.phase = TRAIN_PHASE_SETTLED;
    update_body_estimates_locked();
    set_event_locked("session_reset");
    TRAIN_UNLOCK();
}

bool training_session_set_height(float heightCm)
{
    if (!isfinite(heightCm) || heightCm < TRAIN_HEIGHT_MIN_CM ||
        heightCm > TRAIN_HEIGHT_MAX_CM) return false;
    TRAIN_LOCK();
    s_train.pub.heightCm = heightCm;
    update_body_estimates_locked();
    set_event_locked("profile_updated");
    TRAIN_UNLOCK();
    return true;
}

bool training_session_set_shank_length(float shankLengthCm)
{
    if (!isfinite(shankLengthCm) ||
        (shankLengthCm != 0.0f && (shankLengthCm < 20.0f || shankLengthCm > 70.0f))) {
        return false;
    }
    TRAIN_LOCK();
    s_train.measuredShankCm = shankLengthCm;
    update_body_estimates_locked();
    set_event_locked("profile_updated");
    TRAIN_UNLOCK();
    return true;
}

bool training_session_set_goal(bool enabled, float romMinDeg, float romMaxDeg,
                               float cadenceSpm)
{
    if (enabled &&
        (!isfinite(romMinDeg) || !isfinite(romMaxDeg) || !isfinite(cadenceSpm) ||
         romMinDeg < TRAIN_LIFT_START_DEG ||
         romMaxDeg > TRAIN_ABSOLUTE_ROM_MAX_DEG ||
         (romMaxDeg - romMinDeg) < TRAIN_GOAL_ROM_SPAN_MIN_DEG ||
         cadenceSpm < TRAIN_GOAL_CADENCE_MIN_SPM ||
         cadenceSpm > TRAIN_GOAL_CADENCE_MAX_SPM)) {
        return false;
    }

    bool ok = false;
    TRAIN_LOCK();
    if (s_train.pub.state != TRAIN_STATE_RUNNING &&
        s_train.pub.state != TRAIN_STATE_PAUSED) {
        if (enabled) {
            s_train.pub.goalEnabled = true;
            s_train.pub.goalRomMinDeg = romMinDeg;
            s_train.pub.goalRomMaxDeg = romMaxDeg;
            s_train.pub.goalCadenceSpm = cadenceSpm;
            s_train.pub.goalCadenceToleranceSpm =
                maxf(TRAIN_GOAL_CADENCE_TOL_MIN,
                     cadenceSpm * TRAIN_GOAL_CADENCE_TOL_PCT);
        } else {
            set_default_goal_locked();
        }
        set_event_locked(enabled ? "goal_updated" : "goal_disabled");
        ok = true;
    }
    TRAIN_UNLOCK();
    return ok;
}

void training_session_feed(const training_sample_t *s)
{
    if (s == NULL || !s->imuValid) return;
    const float accelMag = vector3_magnitude(s->accelXG, s->accelYG, s->accelZG);
    const float gyroMag = vector3_magnitude(s->gyroXDps, s->gyroYDps, s->gyroZDps);

    TRAIN_LOCK();
    if (s_train.pub.state != TRAIN_STATE_RUNNING) {
        s_train.pub.lastAccelMagnitudeG = accelMag;
        s_train.pub.lastGyroMagnitudeDps = gyroMag;
        TRAIN_UNLOCK();
        return;
    }

    if (s->estop) {
        s_train.pub.elapsedMs = elapsed_locked(s->nowMs);
        s_train.pub.state = TRAIN_STATE_PAUSED;
        s_train.pauseStartedMs = s->nowMs;
        clear_cycle_locked();
        set_event_locked("emergency_stop");
        TRAIN_UNLOCK();
        return;
    }

    s_train.pub.elapsedMs = elapsed_locked(s->nowMs);
    const uint32_t dtMs = (s_train.lastFeedMs != 0U && s->nowMs >= s_train.lastFeedMs)
                              ? s->nowMs - s_train.lastFeedMs : 0U;
    const float cap = (s->controlCap > 1.0f) ? s->controlCap : 1000.0f;
    float intervention = absf(s->correctionOut) / cap;
    if (intervention > 1.0f) intervention = 1.0f;
    s_train.interventionTimeMs += intervention * (float)dtMs;
    s_train.correctionLoad += intervention * absf(s->gyroXDps) * ((float)dtMs / 1000.0f);
    if (s_train.pub.elapsedMs > 0U) {
        s_train.pub.interventionMeanPct =
            100.0f * s_train.interventionTimeMs / (float)s_train.pub.elapsedMs;
    }
    s_train.pub.interventionPeakPct = maxf(s_train.pub.interventionPeakPct,
                                           intervention * 100.0f);
    s_train.pub.correctionLoadIndex = s_train.correctionLoad;

    update_fall_locked(s, accelMag, gyroMag);
    if (s_train.pub.state != TRAIN_STATE_RUNNING) {
        TRAIN_UNLOCK();
        return;
    }

    const float excursion = absf(s->pitchDeg - s_train.pub.baselinePitchDeg);
    const float lateral = absf(s->rollDeg - s_train.pub.baselineRollDeg);
    const float pitchDelta = absf(s->pitchDeg - s_train.lastPitchDeg);
    const float derivativeDps = (dtMs > 0U) ? pitchDelta * 1000.0f / (float)dtMs : 0.0f;
    s_train.pub.currentExcursionDeg = excursion;

    switch (s_train.pub.phase) {
    case TRAIN_PHASE_SETTLED:
        if (excursion >= TRAIN_LIFT_START_DEG && derivativeDps >= 2.0f) {
            s_train.pub.phase = TRAIN_PHASE_LIFTING;
            s_train.phaseStartedMs = s->nowMs;
            s_train.peakMs = s->nowMs;
            s_train.peakExcursionDeg = excursion;
            s_train.peakSpeedDps = absf(s->gyroYDps);
            s_train.peakLateralDeg = lateral;
            set_event_locked("lift_started");
        }
        break;
    case TRAIN_PHASE_LIFTING:
        s_train.peakSpeedDps = maxf(s_train.peakSpeedDps, absf(s->gyroYDps));
        s_train.peakLateralDeg = maxf(s_train.peakLateralDeg, lateral);
        if (excursion > s_train.peakExcursionDeg) {
            s_train.peakExcursionDeg = excursion;
            s_train.peakMs = s->nowMs;
        }
        if (s_train.peakExcursionDeg > TRAIN_ABSOLUTE_ROM_MAX_DEG ||
            (s->nowMs - s_train.phaseStartedMs) > TRAIN_CYCLE_MAX_MS) {
            s_train.pub.invalidCycles++;
            set_event_locked("cycle_rejected");
            clear_cycle_locked();
        } else if (s_train.peakExcursionDeg >= TRAIN_LIFT_START_DEG &&
                   excursion <= (s_train.peakExcursionDeg - TRAIN_PEAK_DROP_DEG)) {
            s_train.pub.phase = TRAIN_PHASE_RETURNING;
            set_event_locked("return_started");
        }
        break;
    case TRAIN_PHASE_RETURNING:
        s_train.peakSpeedDps = maxf(s_train.peakSpeedDps, absf(s->gyroYDps));
        s_train.peakLateralDeg = maxf(s_train.peakLateralDeg, lateral);
        if ((s->nowMs - s_train.phaseStartedMs) > TRAIN_CYCLE_MAX_MS) {
            s_train.pub.invalidCycles++;
            set_event_locked("cycle_timeout");
            clear_cycle_locked();
        } else if (excursion <= TRAIN_RETURN_COMPLETE_DEG) {
            if (s_train.settleStartedMs == 0U) s_train.settleStartedMs = s->nowMs;
            if ((s->nowMs - s_train.settleStartedMs) >= TRAIN_SETTLE_HOLD_MS) {
                finish_cycle_locked(s, excursion, s->nowMs);
                clear_cycle_locked();
            }
        } else {
            s_train.settleStartedMs = 0U;
        }
        break;
    default:
        clear_cycle_locked();
        break;
    }

    s_train.lastPitchDeg = s->pitchDeg;
    s_train.lastFeedMs = s->nowMs;
    TRAIN_UNLOCK();
}

void training_session_get_status(training_status_t *out, uint32_t nowMs)
{
    if (out == NULL) return;
    TRAIN_LOCK();
    *out = s_train.pub;
    out->elapsedMs = elapsed_locked(nowMs);
    TRAIN_UNLOCK();
}

bool training_session_take_fall_event(void)
{
    bool pending;
    TRAIN_LOCK();
    pending = s_train.fallPending;
    s_train.fallPending = false;
    TRAIN_UNLOCK();
    return pending;
}

const char *training_state_name(training_state_t state)
{
    static const char *const names[] = { "idle", "running", "paused", "finished" };
    return (state <= TRAIN_STATE_FINISHED) ? names[state] : "unknown";
}

const char *training_phase_name(training_phase_t phase)
{
    static const char *const names[] = { "settled", "lifting", "returning" };
    return (phase <= TRAIN_PHASE_RETURNING) ? names[phase] : "unknown";
}
