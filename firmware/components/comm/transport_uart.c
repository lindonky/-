/* UART 控制台传输：板载 CH340 串口（UART0）。 */
#include "transport_uart.h"

#include "bsp.h"
#include "cmd_parser.h"
#include "line_reader.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void uart_send(void *ctx, const char *data, size_t len)
{
    (void)ctx;
    bsp_console_write(data, len);
}

static const cmd_transport_t s_transport = {
    .name = "UART",
    .send = uart_send,
    .ctx  = NULL,
};

static void uart_rx_task(void *arg)
{
    (void)arg;
    line_reader_t lr;
    char line[CMD_LINE_MAX];
    uint8_t buf[128];

    line_reader_reset(&lr);
    bsp_console_write("esc_s3 ready, type 'help'\r\n> ", 30);

    while (true) {
        int r = bsp_console_read(buf, sizeof(buf), pdMS_TO_TICKS(50));
        if (r <= 0) continue;

        /* 回显非行尾字节（方便电脑端手动输入） */
        for (int i = 0; i < r; i++) {
            const uint8_t c = buf[i];
            if (c != '\r' && c != '\n') {
                bsp_console_write((const char *)&c, 1);
            }
        }
        /* 逐字节组行 -> 交给协议层 */
        for (int i = 0; i < r; i++) {
            if (line_reader_feed_byte(&lr, buf[i], line, sizeof(line))) {
                cmd_parser_feed_line(&s_transport, line);
                bsp_console_write("\r\n> ", 4);
            }
        }
    }
}

void transport_uart_init(void)
{
    xTaskCreate(uart_rx_task, "uart_console", 4096, NULL, 5, NULL);
}
