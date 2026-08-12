#include "uart_console.h"

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_vfs_dev.h"

#define CONSOLE_UART   UART_NUM_0
#define CONSOLE_RX_BUF 1024
#define CONSOLE_TX_BUF 1024

static const char *TAG = "uart_console";

esp_err_t uart_console_init(int baud)
{
    const uart_config_t uart_config = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    /* 系统控制台（UART0）默认只做寄存器级输出，未安装 driver/uart，
     * 直接调 uart_write_bytes 会因队列为 NULL 而断言。这里补装驱动并
     * 让 VFS 控制台走驱动，printf/uart API 共用 UART0。 */
    ESP_RETURN_ON_ERROR(uart_driver_install(CONSOLE_UART, CONSOLE_RX_BUF, CONSOLE_TX_BUF, 0, NULL, 0),
                        TAG, "driver install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(CONSOLE_UART, &uart_config),
                        TAG, "param config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(CONSOLE_UART, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "set pin failed");
    esp_vfs_dev_uart_use_driver(CONSOLE_UART);
    return ESP_OK;
}

int uart_console_write(const char *data, size_t len)
{
    return (int)uart_write_bytes(CONSOLE_UART, data, len);
}

int uart_console_read(uint8_t *buf, size_t len, TickType_t timeout)
{
    return uart_read_bytes(CONSOLE_UART, buf, len, timeout);
}
