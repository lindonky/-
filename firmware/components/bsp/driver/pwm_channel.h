#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* 50Hz 脉宽输出通道驱动（driver 层，基于 LEDC）。 */

typedef struct {
    uint32_t freq_hz;      /* PWM 频率，电调用 50 */
    uint32_t period_us;    /* 周期，20000us */
    uint32_t min_pulse_us; /* 最小脉宽，800us */
    uint32_t max_pulse_us; /* 最大脉宽，2200us */
} pwm_channel_cfg_t;

/* 初始化 count 个通道（共用一个 LEDC 定时器，各占一个通道）。 */
esp_err_t pwm_channel_init(const int *gpios, size_t count, const pwm_channel_cfg_t *cfg);

/* 设置某通道脉宽（us），自动钳位在 min~max。 */
esp_err_t pwm_channel_set_pulse(size_t idx, uint32_t pulse_us);

uint32_t pwm_channel_min_pulse(void);
uint32_t pwm_channel_max_pulse(void);
