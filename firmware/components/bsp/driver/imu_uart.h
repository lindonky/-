#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

/* JY61P 姿态传感器串口（UART2）驱动：RX 环形缓冲非阻塞读。 */

/* 初始化 UART2（引脚/波特率见 Kconfig）。 */
esp_err_t imu_uart_init(void);

/* 阻塞读取最多 len 字节（带超时），返回实际字节数。 */
int imu_uart_read(uint8_t *buf, size_t len, TickType_t timeout);
