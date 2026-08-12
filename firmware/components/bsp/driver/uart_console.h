#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

/* 控制台串口（UART0，板载 CH340）。非阻塞收发，带 TX/RX 环形缓冲。 */

/* 安装 UART0 驱动并把 VFS 控制台切到驱动模式（printf 与 uart API 共用 UART0）。 */
esp_err_t uart_console_init(int baud);

/* 非阻塞发送（拷贝进 TX 环形缓冲，由 ISR 逐字节发出）。 */
int uart_console_write(const char *data, size_t len);

/* 阻塞读取最多 len 字节，返回实际收到字节数。 */
int uart_console_read(uint8_t *buf, size_t len, TickType_t timeout);
