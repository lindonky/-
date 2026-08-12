#include "bsp.h"

#include "driver/pwm_channel.h"
#include "driver/uart_console.h"

#include "esp_log.h"

#include <stdarg.h>
#include <stdio.h>

static const char *TAG = "bsp";

static const int s_esc_gpios[BSP_ESC_COUNT] = {
    CONFIG_ESC_MOTOR_GPIO_1,
    CONFIG_ESC_MOTOR_GPIO_2,
    CONFIG_ESC_MOTOR_GPIO_3,
};

static const pwm_channel_cfg_t s_pwm_cfg = {
    .freq_hz      = 50,
    .period_us    = 20000,
    .min_pulse_us = 800,
    .max_pulse_us = 2200,
};

/* 日志全部重定向到控制台串口，方便电脑端串口调试。 */
static int bsp_console_vprintf(const char *fmt, va_list ap)
{
    char buf[256];
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (len > 0) {
        uart_console_write(buf, len);
    }
    return len;
}

void bsp_init(void)
{
    ESP_ERROR_CHECK(pwm_channel_init(s_esc_gpios, BSP_ESC_COUNT, &s_pwm_cfg));
    ESP_ERROR_CHECK(uart_console_init(CONFIG_ESC_CONSOLE_BAUD));
    esp_log_set_vprintf(bsp_console_vprintf);
    ESP_LOGI(TAG, "bsp ready: %d ESCs, console UART0 @%d",
             BSP_ESC_COUNT, CONFIG_ESC_CONSOLE_BAUD);
}

void bsp_esc_set_pulse(size_t idx, uint32_t pulse_us)
{
    ESP_ERROR_CHECK(pwm_channel_set_pulse(idx, pulse_us));
}

size_t bsp_esc_count(void) { return BSP_ESC_COUNT; }

void bsp_console_write(const char *data, size_t len)
{
    uart_console_write(data, len);
}

int bsp_console_read(uint8_t *buf, size_t len, TickType_t timeout)
{
    return uart_console_read(buf, len, timeout);
}
