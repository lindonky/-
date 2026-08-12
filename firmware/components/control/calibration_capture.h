#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 短时高频标定采集。数据只保存在 RAM，停止后才允许导出。
 * 整数定点存储用于限制 ESP32 内存占用；CSV 导出时恢复单位。 */

typedef enum {
    CAPTURE_STATE_EMPTY = 0,
    CAPTURE_STATE_RECORDING,
    CAPTURE_STATE_READY,
} calibration_capture_state_t;

enum {
    CAPTURE_FLAG_IMU_VALID = (1U << 0),
    CAPTURE_FLAG_ESTOP     = (1U << 1),
    CAPTURE_FLAG_STABILIZE = (1U << 2),
};

typedef struct {
    uint32_t timestampMs;
    uint32_t sequence;
    int16_t rollCentiDeg;
    int16_t pitchCentiDeg;
    int16_t yawCentiDeg;
    int16_t motionCentiDeg;
    int16_t lateralCentiDeg;
    int16_t gyroXDeciDps;
    int16_t gyroYDeciDps;
    int16_t gyroZDeciDps;
    int16_t accelXMilliG;
    int16_t accelYMilliG;
    int16_t accelZMilliG;
    int16_t pitchOut;
    int16_t rollOut;
    uint16_t t1PulseUs;
    uint16_t t2PulseUs;
    uint16_t t3PulseUs;
    int8_t mode;
    uint8_t flags;
} calibration_capture_record_t;

typedef struct {
    uint32_t nowMs;
    uint32_t sequence;
    bool imuValid;
    bool estop;
    bool stabilizeEnabled;
    int mode;
    float rollDeg;
    float pitchDeg;
    float yawDeg;
    float motionDeg;
    float lateralDeg;
    float gyroXDps;
    float gyroYDps;
    float gyroZDps;
    float accelXG;
    float accelYG;
    float accelZG;
    float pitchOut;
    float rollOut;
    uint32_t t1PulseUs;
    uint32_t t2PulseUs;
    uint32_t t3PulseUs;
} calibration_capture_sample_t;

typedef struct {
    calibration_capture_state_t state;
    uint32_t startedMs;
    uint32_t endedMs;
    uint32_t durationMs;
    uint32_t sampleCount;
    uint32_t capacity;
    float sampleRateHz;
    bool full;
    char label[32];
} calibration_capture_status_t;

void calibration_capture_init(void);
bool calibration_capture_start(uint32_t nowMs, bool imuValid, const char *label);
bool calibration_capture_stop(void);
bool calibration_capture_clear(void);
void calibration_capture_feed(const calibration_capture_sample_t *sample);
void calibration_capture_get_status(calibration_capture_status_t *out,
                                    uint32_t nowMs);
bool calibration_capture_get_record(uint32_t index,
                                    calibration_capture_record_t *out);
/* 导出期间锁定 ready 快照；不阻塞 IMU/控制任务。 */
bool calibration_capture_begin_export(calibration_capture_status_t *out);
void calibration_capture_end_export(void);
const char *calibration_capture_state_name(calibration_capture_state_t state);
