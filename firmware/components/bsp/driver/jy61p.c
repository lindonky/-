#include "jy61p.h"

#include <string.h>

#define JY61P_HEADER     (0x55U)
#define JY61P_ACCEL      (0x51U)
#define JY61P_GYRO       (0x52U)
#define JY61P_ANGLE      (0x53U)
#define JY61P_FRAME_LEN  (11U)
#define JY61P_FRAME_TIMEOUT_MS (30U)  /* 帧中字节间隔超过此值视为丢帧 */

static int16_t read_i16_le(const uint8_t *d)
{
    return (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8U));
}

/* int16 原始值按满量程换算 */
static float scale_i16(const uint8_t *d, float fullScale)
{
    return ((float)read_i16_le(d) * fullScale) / 32768.0f;
}

static bool checksum_ok(const uint8_t *frame)
{
    uint8_t sum = 0U;
    for (uint8_t i = 0U; i < (JY61P_FRAME_LEN - 1U); i++) {
        sum = (uint8_t)(sum + frame[i]);
    }
    return sum == frame[JY61P_FRAME_LEN - 1U];
}

static void parse_frame(jy61p_t *p, const uint8_t *frame, uint32_t nowMs)
{
    if (!checksum_ok(frame)) {
        p->checksumErrors++;
        return;
    }
    p->validFrames++;

    switch (frame[1]) {
    case JY61P_ACCEL:
        /* 加速度 ±16g */
        p->accelXG = scale_i16(&frame[2], 16.0f);
        p->accelYG = scale_i16(&frame[4], 16.0f);
        p->accelZG = scale_i16(&frame[6], 16.0f);
        break;
    case JY61P_GYRO:
        /* 角速度 ±2000dps */
        p->gyroXDps = scale_i16(&frame[2], 2000.0f);
        p->gyroYDps = scale_i16(&frame[4], 2000.0f);
        p->gyroZDps = scale_i16(&frame[6], 2000.0f);
        p->gyroSeq++;
        break;
    case JY61P_ANGLE:
        /* 角度 ±180°，与最新角速度/加速度组成一份姿态样本 */
        p->latest.rollDeg  = scale_i16(&frame[2], 180.0f);
        p->latest.pitchDeg = scale_i16(&frame[4], 180.0f);
        p->latest.yawDeg   = scale_i16(&frame[6], 180.0f);
        p->latest.gyroXDps = p->gyroXDps;
        p->latest.gyroYDps = p->gyroYDps;
        p->latest.gyroZDps = p->gyroZDps;
        p->latest.accelXG  = p->accelXG;
        p->latest.accelYG  = p->accelYG;
        p->latest.accelZG  = p->accelZG;
        p->latest.timestampMs = nowMs;
        p->latest.sequence++;
        p->latest.valid = true;
        break;
    default:
        break;
    }
}

void jy61p_init(jy61p_t *p)
{
    memset(p, 0, sizeof(*p));
}

void jy61p_push_byte(jy61p_t *p, uint8_t byte, uint32_t nowMs)
{
    bool ok;

    if (p == NULL) return;

    /* 帧中字节间隔超时则丢弃半截帧 */
    if ((p->frameIndex != 0U) &&
        ((nowMs - p->frameStartMs) > JY61P_FRAME_TIMEOUT_MS)) {
        p->frameIndex = 0U;
    }

    if (p->frameIndex == 0U) {
        if (byte != JY61P_HEADER) return;   /* 等待帧头 */
        p->frame[p->frameIndex++] = byte;
        p->frameStartMs = nowMs;
        return;
    }

    p->frame[p->frameIndex++] = byte;
    if (p->frameIndex < JY61P_FRAME_LEN) return;

    ok = checksum_ok(p->frame);
    parse_frame(p, p->frame, nowMs);
    p->frameIndex = 0U;

    /* 校验失败时末字节可能恰是下一帧头，保留它继续组帧 */
    if (!ok && (byte == JY61P_HEADER)) {
        p->frame[0] = byte;
        p->frameIndex = 1U;
        p->frameStartMs = nowMs;
    }
}

bool jy61p_get_new(const jy61p_t *p, uint32_t prevSequence, jy61p_data_t *out)
{
    if ((p == NULL) || (out == NULL) || !p->latest.valid ||
        (p->latest.sequence == prevSequence)) {
        return false;
    }
    *out = p->latest;
    return true;
}

uint32_t jy61p_last_frame_ms(const jy61p_t *p)
{
    return (p != NULL) ? p->latest.timestampMs : 0U;
}
