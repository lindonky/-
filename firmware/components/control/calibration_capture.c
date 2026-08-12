#include "calibration_capture.h"

#ifdef CALIBRATION_CAPTURE_HOST_TEST
#ifndef CONFIG_CAPTURE_MAX_SAMPLES
#define CONFIG_CAPTURE_MAX_SAMPLES 8
#endif
#define LOCK()
#define UNLOCK()
#else
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
static SemaphoreHandle_t s_mutex;
#define LOCK()   do { if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY); } while (0)
#define UNLOCK() do { if (s_mutex) xSemaphoreGive(s_mutex); } while (0)
#endif

#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <string.h>

static calibration_capture_record_t s_records[CONFIG_CAPTURE_MAX_SAMPLES];
static calibration_capture_status_t s_status;
static bool s_exporting;

static int16_t quantize(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > (float)INT16_MAX) scaled = (float)INT16_MAX;
    if (scaled < (float)INT16_MIN) scaled = (float)INT16_MIN;
    return (int16_t)lroundf(scaled);
}

static uint16_t quantize_pulse(uint32_t pulseUs)
{
    return pulseUs > UINT16_MAX ? UINT16_MAX : (uint16_t)pulseUs;
}

static void copy_label(char out[32], const char *label)
{
    const char *src = (label && label[0] != '\0') ? label : "unlabeled";
    size_t n = 0U;
    while (src[n] != '\0' && n < 31U) {
        const unsigned char c = (unsigned char)src[n];
        out[n] = (char)((isalnum(c) || c == '-' || c == '_') ? c : '_');
        n++;
    }
    out[n] = '\0';
}

static void update_derived(uint32_t nowMs)
{
    if (s_status.state == CAPTURE_STATE_RECORDING) {
        s_status.durationMs = nowMs - s_status.startedMs;
    } else if (s_status.sampleCount > 0U) {
        s_status.durationMs = s_status.endedMs - s_status.startedMs;
    } else {
        s_status.durationMs = 0U;
    }
    s_status.sampleRateHz = s_status.durationMs > 0U && s_status.sampleCount > 1U
        ? ((float)(s_status.sampleCount - 1U) * 1000.0f / (float)s_status.durationMs)
        : 0.0f;
}

void calibration_capture_init(void)
{
#ifndef CALIBRATION_CAPTURE_HOST_TEST
    if (!s_mutex) s_mutex = xSemaphoreCreateMutex();
#endif
    LOCK();
    memset(&s_status, 0, sizeof(s_status));
    s_exporting = false;
    s_status.state = CAPTURE_STATE_EMPTY;
    s_status.capacity = CONFIG_CAPTURE_MAX_SAMPLES;
    UNLOCK();
}

bool calibration_capture_start(uint32_t nowMs, bool imuValid, const char *label)
{
    bool ok = false;
    LOCK();
    if (imuValid && !s_exporting && s_status.state == CAPTURE_STATE_EMPTY) {
        memset(&s_status, 0, sizeof(s_status));
        s_status.state = CAPTURE_STATE_RECORDING;
        s_status.startedMs = nowMs;
        s_status.capacity = CONFIG_CAPTURE_MAX_SAMPLES;
        copy_label(s_status.label, label);
        ok = true;
    }
    UNLOCK();
    return ok;
}

bool calibration_capture_stop(void)
{
    bool ok = false;
    LOCK();
    if (s_status.state == CAPTURE_STATE_RECORDING) {
        ok = true;
        if (s_status.sampleCount > 0U) {
            s_status.state = CAPTURE_STATE_READY;
            /* 保留最后一个有效样本时间，不把按钮延迟算入采样率。 */
            update_derived(s_status.endedMs);
        } else {
            memset(&s_status, 0, sizeof(s_status));
            s_status.state = CAPTURE_STATE_EMPTY;
            s_status.capacity = CONFIG_CAPTURE_MAX_SAMPLES;
        }
    }
    UNLOCK();
    return ok;
}

bool calibration_capture_clear(void)
{
    bool ok = false;
    LOCK();
    if (!s_exporting && s_status.state != CAPTURE_STATE_RECORDING) {
        memset(s_records, 0, sizeof(s_records));
        memset(&s_status, 0, sizeof(s_status));
        s_status.state = CAPTURE_STATE_EMPTY;
        s_status.capacity = CONFIG_CAPTURE_MAX_SAMPLES;
        ok = true;
    }
    UNLOCK();
    return ok;
}

void calibration_capture_feed(const calibration_capture_sample_t *s)
{
    if (!s || !s->imuValid) return;
    LOCK();
    if (s_status.state != CAPTURE_STATE_RECORDING) {
        UNLOCK();
        return;
    }
    if (s_status.sampleCount >= CONFIG_CAPTURE_MAX_SAMPLES) {
        s_status.state = CAPTURE_STATE_READY;
        s_status.endedMs = s->nowMs;
        s_status.full = true;
        update_derived(s->nowMs);
        UNLOCK();
        return;
    }

    if (s_status.sampleCount == 0U) s_status.startedMs = s->nowMs;
    calibration_capture_record_t *r = &s_records[s_status.sampleCount++];
    r->timestampMs = s->nowMs;
    r->sequence = s->sequence;
    r->rollCentiDeg = quantize(s->rollDeg, 100.0f);
    r->pitchCentiDeg = quantize(s->pitchDeg, 100.0f);
    r->yawCentiDeg = quantize(s->yawDeg, 100.0f);
    r->motionCentiDeg = quantize(s->motionDeg, 100.0f);
    r->lateralCentiDeg = quantize(s->lateralDeg, 100.0f);
    r->gyroXDeciDps = quantize(s->gyroXDps, 10.0f);
    r->gyroYDeciDps = quantize(s->gyroYDps, 10.0f);
    r->gyroZDeciDps = quantize(s->gyroZDps, 10.0f);
    r->accelXMilliG = quantize(s->accelXG, 1000.0f);
    r->accelYMilliG = quantize(s->accelYG, 1000.0f);
    r->accelZMilliG = quantize(s->accelZG, 1000.0f);
    r->pitchOut = quantize(s->pitchOut, 1.0f);
    r->rollOut = quantize(s->rollOut, 1.0f);
    r->t1PulseUs = quantize_pulse(s->t1PulseUs);
    r->t2PulseUs = quantize_pulse(s->t2PulseUs);
    r->t3PulseUs = quantize_pulse(s->t3PulseUs);
    r->mode = (int8_t)s->mode;
    r->flags = (uint8_t)((s->imuValid ? CAPTURE_FLAG_IMU_VALID : 0U) |
                         (s->estop ? CAPTURE_FLAG_ESTOP : 0U) |
                         (s->stabilizeEnabled ? CAPTURE_FLAG_STABILIZE : 0U));

    s_status.endedMs = s->nowMs;
    update_derived(s->nowMs);
    if (s_status.sampleCount >= CONFIG_CAPTURE_MAX_SAMPLES) {
        s_status.state = CAPTURE_STATE_READY;
        s_status.full = true;
    }
    UNLOCK();
}

void calibration_capture_get_status(calibration_capture_status_t *out,
                                    uint32_t nowMs)
{
    if (!out) return;
    LOCK();
    update_derived(nowMs);
    *out = s_status;
    UNLOCK();
}

bool calibration_capture_get_record(uint32_t index,
                                    calibration_capture_record_t *out)
{
    if (!out) return false;
    bool ok = false;
    LOCK();
    if (s_status.state != CAPTURE_STATE_RECORDING && index < s_status.sampleCount) {
        *out = s_records[index];
        ok = true;
    }
    UNLOCK();
    return ok;
}

bool calibration_capture_begin_export(calibration_capture_status_t *out)
{
    if (!out) return false;
    bool ok = false;
    LOCK();
    if (!s_exporting && s_status.state == CAPTURE_STATE_READY &&
        s_status.sampleCount > 0U) {
        s_exporting = true;
        *out = s_status;
        ok = true;
    }
    UNLOCK();
    return ok;
}

void calibration_capture_end_export(void)
{
    LOCK();
    s_exporting = false;
    UNLOCK();
}

const char *calibration_capture_state_name(calibration_capture_state_t state)
{
    switch (state) {
    case CAPTURE_STATE_RECORDING: return "recording";
    case CAPTURE_STATE_READY:     return "ready";
    case CAPTURE_STATE_EMPTY:
    default:                      return "empty";
    }
}
