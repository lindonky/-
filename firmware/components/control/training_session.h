#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 单腿训练会话。
 *
 * JY61P 安装在小腿中部：pitch 用作抬腿主运动轴，roll 用作横向偏移轴。
 * 一步 = 离开中立位 -> 峰值 -> 回到中立位的一次完整周期。
 * 这里的 ROM/达标率/等效步长都是小腿轨迹工程指标，不是膝关节临床量。
 */

typedef enum {
    TRAIN_STATE_IDLE = 0,
    TRAIN_STATE_RUNNING,
    TRAIN_STATE_PAUSED,
    TRAIN_STATE_FINISHED,
} training_state_t;

typedef enum {
    TRAIN_PHASE_SETTLED = 0,
    TRAIN_PHASE_LIFTING,
    TRAIN_PHASE_RETURNING,
} training_phase_t;

enum {
    TRAIN_QUALITY_ROM_LOW       = (1U << 0),
    TRAIN_QUALITY_ROM_HIGH      = (1U << 1),
    TRAIN_QUALITY_SPEED_LOW     = (1U << 2),
    TRAIN_QUALITY_SPEED_HIGH    = (1U << 3),
    TRAIN_QUALITY_LATERAL       = (1U << 4),
    TRAIN_QUALITY_RETURN        = (1U << 5),
    TRAIN_QUALITY_CYCLE_TIME    = (1U << 6),
    TRAIN_QUALITY_CADENCE_LOW   = (1U << 7),
    TRAIN_QUALITY_CADENCE_HIGH  = (1U << 8),
};

typedef struct {
    uint32_t nowMs;
    bool imuValid;
    bool estop;
    int mode;
    float pitchDeg;
    float rollDeg;
    float gyroXDps;
    float gyroYDps;
    float gyroZDps;
    float accelXG;
    float accelYG;
    float accelZG;
    float correctionOut;  /* 左右纠偏控制量，-1000..1000 */
    float controlCap;     /* 当前模式控制上限，1..1000 */
} training_sample_t;

typedef struct {
    /* 会话状态 */
    training_state_t state;
    training_phase_t phase;
    uint32_t sessionId;
    uint32_t elapsedMs;
    int mode;
    uint32_t steps;          /* 完整抬起—回位周期 */
    uint32_t qualifiedSteps; /* 满足当前工程阈值 */
    uint32_t invalidCycles;  /* 超时或未达到启动幅度的候选周期 */
    uint32_t fallEvents;
    uint32_t lastQualityFlags;

    /* 安装中立位与人体参数 */
    float baselinePitchDeg;
    float baselineRollDeg;
    float heightCm;
    float estimatedLegCm;
    float shankLengthCm;
    bool shankLengthMeasured;

    /* 可选训练目标。仅改变动作评价，不改变硬角度限位或跌倒保护。 */
    bool goalEnabled;
    float goalRomMinDeg;
    float goalRomMaxDeg;
    float goalCadenceSpm;
    float goalCadenceToleranceSpm;

    /* 本周期/累计指标 */
    float currentExcursionDeg;
    float lastRomDeg;
    float averageRomDeg;
    float lastPeakSpeedDps;
    float lastCycleSec;
    float lastLiftSec;
    float lastReturnSec;
    float cadenceSpm;
    float lastLateralDeg;
    float lastReturnErrorDeg;
    float lastEstimatedStepCm;
    float averageEstimatedStepCm;
    float qualifiedPct;

    /* 设备作用代理指标；均为无量纲/工程指数，不是力、力矩或功率 */
    float interventionMeanPct;
    float interventionPeakPct;
    float correctionLoadIndex;

    /* 跌倒候选证据 */
    uint8_t fallStage;
    float lastAccelMagnitudeG;
    float lastGyroMagnitudeDps;
    float lastFallTiltDeg;
    char lastEvent[32];
} training_status_t;

void training_session_init(void);

bool training_session_start(uint32_t nowMs, float pitchDeg, float rollDeg,
                            int mode, bool imuValid, bool estop);
bool training_session_pause(uint32_t nowMs);
bool training_session_resume(uint32_t nowMs, float pitchDeg, float rollDeg,
                             bool imuValid);
bool training_session_stop(uint32_t nowMs);
void training_session_reset(void);

bool training_session_set_height(float heightCm);
bool training_session_set_shank_length(float shankLengthCm); /* 0=恢复身高估算 */
bool training_session_set_goal(bool enabled, float romMinDeg, float romMaxDeg,
                               float cadenceSpm);

void training_session_feed(const training_sample_t *sample);
void training_session_get_status(training_status_t *out, uint32_t nowMs);

/* 多阶段疑似跌倒确认后置位一次；控制任务取走后进入现有闩锁急停。 */
bool training_session_take_fall_event(void);

const char *training_state_name(training_state_t state);
const char *training_phase_name(training_phase_t phase);
