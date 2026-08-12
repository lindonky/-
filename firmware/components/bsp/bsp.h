#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#define BSP_ESC_COUNT 3

/* 初始化板级资源：3 路电调 PWM + 控制台串口 + 日志重定向到串口。 */
void bsp_init(void);

/* 设置第 idx 路电调脉宽（us），自动钳位 800~2200。 */
void bsp_esc_set_pulse(size_t idx, uint32_t pulse_us);

size_t bsp_esc_count(void);

/* 控制台串口收发（非阻塞写 / 带超时读）。 */
void bsp_console_write(const char *data, size_t len);
int  bsp_console_read(uint8_t *buf, size_t len, TickType_t timeout);
