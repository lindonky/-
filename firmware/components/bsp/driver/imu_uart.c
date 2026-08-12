#include "imu_uart.h"

#include "driver/uart.h"
#include "esp_check.h"

#define IMU_UART       UART_NUM_2
#define IMU_RX_BUF     1024

static const char *TAG = "imu_uart";

esp_err_t imu_uart_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate  = CONFIG_ESC_IMU_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(IMU_UART, IMU_RX_BUF, 0, 0, NULL, 0),
                        TAG, "driver install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(IMU_UART, &uart_cfg),
                        TAG, "param config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(IMU_UART,
                                     CONFIG_ESC_IMU_UART_TX_GPIO,
                                     CONFIG_ESC_IMU_UART_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "set pin failed");
    return ESP_OK;
}

int imu_uart_read(uint8_t *buf, size_t len, TickType_t timeout)
{
    return uart_read_bytes(IMU_UART, buf, len, timeout);
}
