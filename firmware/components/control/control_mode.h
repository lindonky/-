#pragma once

#include <stdbool.h>

/* 三模式控制 + 急停闩锁。
 *   助力 assist  : pitch->推进、roll->左右舵差动（默认，三段式姿态稳定）
 *   主动 active  : 推进保持中位（不助推），左右舵只做横向腿位纠偏
 *   阻抗 impedance: 抬脚时推进反转（需双向电调），左右舵维持姿态
 * 每模式有独立的输出上限（PWM 幅值上限），防止超限。
 */
typedef enum {
    MODE_ASSIST = 0,
    MODE_ACTIVE = 1,
    MODE_IMPEDANCE = 2,
} control_mode_t;

void control_mode_init(void);

control_mode_t control_mode_get(void);
const char   *control_mode_name(control_mode_t m);
bool          control_mode_set(control_mode_t m);   /* 无效值返回 false */

/* 当前模式的 |输出| 上限（0..1000） */
int control_mode_cap(void);

/* 急停闩锁：置位后控制环与手动速度指令都不再输出（三路立即回中位），
 * 直到 control_estop_release()（START 命令）解除。 */
bool control_is_estop(void);
void control_estop_latch(void);
void control_estop_release(void);
