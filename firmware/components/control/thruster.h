#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define THRUSTER_COUNT      3
#define THRUSTER_SPEED_MIN  (-1000)
#define THRUSTER_SPEED_MAX  1000

/* 初始化 3 路推力器状态（输出停止脉宽）。 */
void thruster_init(void);

/* 设置第 idx 路转速 -1000~1000（负=反转，单向模式下钳到 0）。 */
void thruster_set_speed(size_t idx, int speed);

/* 直接设置第 idx 路脉宽（us），跳过缓动（校准/调试用）。 */
void thruster_set_pulse(size_t idx, uint32_t pulse_us);

/* 所有通道平滑回中位（缓动）。 */
void thruster_stop_all(void);

/* 所有通道**立即**回中位（绕过缓动，紧急停止用）。 */
void thruster_estop(void);

/* 阻塞约 6s：3 路同时 2200us -> 停止脉宽。 */
void thruster_calibrate(void);

bool     thruster_is_calibrating(void);
uint32_t thruster_get_current_pulse(size_t idx);
uint32_t thruster_get_target_pulse(size_t idx);
int      thruster_get_speed(size_t idx);   /* 最近一次指令转速 -1000~1000 */

/* 平滑缓动后台任务。 */
void thruster_task(void *arg);
