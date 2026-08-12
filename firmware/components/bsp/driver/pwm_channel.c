#include "pwm_channel.h"

#include "driver/ledc.h"
#include "esp_check.h"

/* ESP32-S3 的 LEDC 定时器最大分辨率 14bit。 */
#define PWM_DUTY_RES   LEDC_TIMER_14_BIT
#define PWM_DUTY_MAX   ((1U << PWM_DUTY_RES) - 1U)

static const char *TAG = "pwm_channel";

static uint32_t s_period_us = 20000;
static uint32_t s_min_pulse = 800;
static uint32_t s_max_pulse = 2200;
static size_t   s_count     = 0;

esp_err_t pwm_channel_init(const int *gpios, size_t count, const pwm_channel_cfg_t *cfg)
{
    s_period_us = cfg->period_us;
    s_min_pulse = cfg->min_pulse_us;
    s_max_pulse = cfg->max_pulse_us;
    s_count     = count;

    const ledc_timer_config_t timer_config = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = PWM_DUTY_RES,
        .freq_hz         = cfg->freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "timer config failed");

    for (size_t i = 0; i < count; i++) {
        const ledc_channel_config_t ch = {
            .gpio_num   = gpios[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)i,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0,
            .flags      = { .output_invert = 0 },
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&ch), TAG, "channel %d config failed", (int)i);
    }
    return ESP_OK;
}

esp_err_t pwm_channel_set_pulse(size_t idx, uint32_t pulse_us)
{
    if (idx >= s_count) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pulse_us < s_min_pulse) pulse_us = s_min_pulse;
    if (pulse_us > s_max_pulse) pulse_us = s_max_pulse;

    const uint32_t duty = (pulse_us * PWM_DUTY_MAX) / s_period_us;
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx, duty),
                        TAG, "set duty failed");
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx);
}

uint32_t pwm_channel_min_pulse(void) { return s_min_pulse; }
uint32_t pwm_channel_max_pulse(void) { return s_max_pulse; }
