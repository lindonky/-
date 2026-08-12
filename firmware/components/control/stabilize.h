#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 姿态稳定应用：JY61P 读姿态 -> 控制律 -> 三路电调映射。
 *   俯仰(pitch) -> 推进(T2)
 *   横滚(roll)  -> 左舵(T1)=+roll、右舵(T3)=-roll 差动
 * 安全：IMU 数据超时停止输出；未使能不介入；电调校准时不介入。
 */

typedef struct {
    bool enabled;
    bool imuValid;        /* 最近收到有效帧 */
    bool staleFault;      /* 数据超时 */
    int  mode;            /* 0=助力 1=主动 2=阻抗（control_mode_get） */
    bool estop;           /* 急停闩锁（control_is_estop） */
    float rollDeg;        /* 相对零位 */
    float pitchDeg;
    float yawDeg;
    float gyroDps;        /* 俯仰轴角速度（控制环当前值） */
    float gyroRollDps;    /* 横滚轴角速度 */
    float pitchOut;       /* 俯仰环输出（推进，已按模式限幅） */
    float rollOut;        /* 横滚环输出（差动，已按模式限幅） */
    bool pitchInDeadband;
    bool pitchAngleStop;
    uint32_t sampleSeq;
    /* IMU 串口诊断 */
    uint32_t rxBytes;          /* UART2 收到的原始字节数 */
    uint32_t validFrames;
    uint32_t checksumErrors;
    uint32_t lastFrameAgeMs;   /* 距最近有效帧的毫秒数 */
} stabilize_status_t;

void stabilize_init(void);
void stabilize_set_enabled(bool en);
bool stabilize_is_enabled(void);
/* 当前姿态作为零位（腿放平后调用） */
void stabilize_zero(void);
const stabilize_status_t *stabilize_get_status(void);
