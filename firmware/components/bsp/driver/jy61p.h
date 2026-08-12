#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 维特智能 JY61P 六轴姿态传感器解析器。
 * 模块上电以 115200 8N1 连续输出 0x55 帧：
 *   0x55 0x51 加速度(±16g) / 0x55 0x52 角速度(±2000dps) / 0x55 0x53 角度(±180°/±90°)
 * 每帧 11 字节，末字节为前 10 字节累加和。0x53 角度帧自带姿态解算，
 * 与最近的 0x52 角速度帧组合成一份姿态样本。
 */

typedef struct {
    float rollDeg;   /* 横滚角  -180~+180° */
    float pitchDeg;  /* 俯仰角   -90~+90° */
    float yawDeg;    /* 航向角    0~360° */
    float gyroXDps;  /* 角速度（与角度帧同周期） */
    float gyroYDps;
    float gyroZDps;
    float accelXG;   /* 加速度（最新一帧） */
    float accelYG;
    float accelZG;
    uint32_t sequence;   /* 姿态样本序号，用于判断是否有新数据 */
    uint32_t timestampMs;
    bool valid;
} jy61p_data_t;

typedef struct {
    uint8_t frame[11];
    uint8_t frameIndex;
    uint32_t frameStartMs;
    float gyroXDps, gyroYDps, gyroZDps;   /* 暂存最近角速度 */
    float accelXG, accelYG, accelZG;      /* 暂存最近加速度 */
    uint32_t gyroSeq;                     /* 角速度帧序号，防重样 */
    jy61p_data_t latest;
    uint32_t checksumErrors;
    uint32_t validFrames;
} jy61p_t;

void jy61p_init(jy61p_t *p);

/* 喂入一字节（串口接收任务调用），nowMs 为系统毫秒时间。 */
void jy61p_push_byte(jy61p_t *p, uint8_t byte, uint32_t nowMs);

/* 读取新姿态样本：传入上一次的 sequence，有新样本则填 out 并返回 true。 */
bool jy61p_get_new(const jy61p_t *p, uint32_t prevSequence, jy61p_data_t *out);

/* 最近一次有效帧时间（ms），用于超时检测；无有效帧返回 0。 */
uint32_t jy61p_last_frame_ms(const jy61p_t *p);
