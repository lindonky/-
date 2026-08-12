#pragma once

#include <stdbool.h>

/* 姿态稳定控制律（三段式，输出归一化 -1000~+1000）：
 *   |误差| <= 死区          -> 输出 0（静止区，腿在正常范围）
 *   死区 < |误差| <= 最大角  -> 输出 = Kp*误差 - Kd*角速度，钳在 ±最大输出
 *   |误差| > 最大角          -> 输出 0（超角度停止推进，安全）
 */
typedef struct {
    float kp;            /* 角度误差增益（输出单位/度） */
    float kd;            /* 角速度阻尼增益 */
    float deadbandDeg;   /* 静止区死区（度） */
    float maxAngleDeg;   /* 超此角度停止输出（度） */
    float maxOutput;     /* 输出限幅 0~1000 */
} attitude_ctrl_tunings_t;

typedef struct {
    attitude_ctrl_tunings_t tunings;
    float targetDeg;
    float measuredDeg;
    float gyroDps;
    float errorDeg;
    float output;        /* -maxOutput..+maxOutput */
    bool inDeadband;     /* 处于静止区 */
    bool angleStop;      /* 超角度停止 */
} attitude_ctrl_t;

void attitude_ctrl_init(attitude_ctrl_t *c);
void attitude_ctrl_reset(attitude_ctrl_t *c);
bool attitude_ctrl_set_tunings(attitude_ctrl_t *c,
                               const attitude_ctrl_tunings_t *t);
void attitude_ctrl_set_target(attitude_ctrl_t *c, float targetDeg);
/* 修改输出限幅（0~1000）。模式切换时由稳定环统一设置上限。 */
void attitude_ctrl_set_output_limit(attitude_ctrl_t *c, float maxOutput);
void attitude_ctrl_update(attitude_ctrl_t *c, float measuredDeg, float gyroDps);
