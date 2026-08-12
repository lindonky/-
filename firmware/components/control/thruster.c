#include "thruster.h"

#include "bsp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define THRUSTER_MIN_PULSE  800
#define THRUSTER_MAX_PULSE  2200
#define THRUSTER_STOP_PULSE CONFIG_ESC_STOP_PULSE_US

static const char *TAG = "thruster";

typedef struct {
    volatile uint32_t current_pulse;
    volatile uint32_t target_pulse;
    volatile int32_t  speed;   /* 最近一次指令转速 */
} thruster_t;

static thruster_t       s_thrusters[THRUSTER_COUNT];
static volatile bool    s_calibrating = false;

static uint32_t clamp_pulse(uint32_t us)
{
    if (us < THRUSTER_MIN_PULSE) us = THRUSTER_MIN_PULSE;
    if (us > THRUSTER_MAX_PULSE) us = THRUSTER_MAX_PULSE;
    return us;
}

void thruster_init(void)
{
    for (size_t i = 0; i < THRUSTER_COUNT; i++) {
        s_thrusters[i].current_pulse = THRUSTER_STOP_PULSE;
        s_thrusters[i].target_pulse  = THRUSTER_STOP_PULSE;
        s_thrusters[i].speed         = 0;
        bsp_esc_set_pulse(i, THRUSTER_STOP_PULSE);
    }
}

void thruster_set_pulse(size_t idx, uint32_t pulse_us)
{
    if (idx >= THRUSTER_COUNT) return;
    pulse_us = clamp_pulse(pulse_us);
    s_thrusters[idx].current_pulse = pulse_us;
    s_thrusters[idx].target_pulse  = pulse_us;
    if (pulse_us == THRUSTER_STOP_PULSE) {
        s_thrusters[idx].speed = 0;
    } else if (pulse_us > THRUSTER_STOP_PULSE &&
               THRUSTER_MAX_PULSE > THRUSTER_STOP_PULSE) {
        s_thrusters[idx].speed =
            (int32_t)((pulse_us - THRUSTER_STOP_PULSE) * 1000U /
                      (THRUSTER_MAX_PULSE - THRUSTER_STOP_PULSE));
    } else if (pulse_us < THRUSTER_STOP_PULSE &&
               THRUSTER_STOP_PULSE > THRUSTER_MIN_PULSE) {
        s_thrusters[idx].speed =
            -(int32_t)((THRUSTER_STOP_PULSE - pulse_us) * 1000U /
                       (THRUSTER_STOP_PULSE - THRUSTER_MIN_PULSE));
    } else {
        s_thrusters[idx].speed = 0;
    }
    bsp_esc_set_pulse(idx, pulse_us);
}

void thruster_set_speed(size_t idx, int speed)
{
    if (idx >= THRUSTER_COUNT) return;
    if (speed < THRUSTER_SPEED_MIN) speed = THRUSTER_SPEED_MIN;
    if (speed > THRUSTER_SPEED_MAX) speed = THRUSTER_SPEED_MAX;
#if !CONFIG_ESC_BIDIRECTIONAL
    /* 硬件电调只能正转：负值钳到 0（反转问题记录在 README）。 */
    if (speed < 0) speed = 0;
#endif

    uint32_t pulse;
    if (speed >= 0) {
        pulse = THRUSTER_STOP_PULSE +
                (uint32_t)speed * (THRUSTER_MAX_PULSE - THRUSTER_STOP_PULSE) / 1000U;
    } else {
        pulse = THRUSTER_STOP_PULSE -
                (uint32_t)(-speed) * (THRUSTER_STOP_PULSE - THRUSTER_MIN_PULSE) / 1000U;
    }
    s_thrusters[idx].target_pulse = clamp_pulse(pulse);
    s_thrusters[idx].speed        = speed;
}

void thruster_stop_all(void)
{
    for (size_t i = 0; i < THRUSTER_COUNT; i++) {
        s_thrusters[i].target_pulse = THRUSTER_STOP_PULSE;
        s_thrusters[i].speed        = 0;
    }
}

/* 急停：三路**立即**写中位（直接驱动硬件，绕过缓动），并清 speed/target。
 * 与 thruster_stop_all 的平滑回中不同，这是真正的紧急停止。 */
void thruster_estop(void)
{
    for (size_t i = 0; i < THRUSTER_COUNT; i++) {
        s_thrusters[i].current_pulse = THRUSTER_STOP_PULSE;
        s_thrusters[i].target_pulse  = THRUSTER_STOP_PULSE;
        s_thrusters[i].speed         = 0;
        bsp_esc_set_pulse(i, THRUSTER_STOP_PULSE);
    }
}

void thruster_calibrate(void)
{
    s_calibrating = true;

    ESP_LOGW(TAG, "calibration: hold 2200us for %dms on %d channels",
             CONFIG_ESC_CALIB_HIGH_TIME_MS, THRUSTER_COUNT);
    for (size_t i = 0; i < THRUSTER_COUNT; i++) {
        s_thrusters[i].current_pulse = THRUSTER_MAX_PULSE;
        s_thrusters[i].target_pulse  = THRUSTER_MAX_PULSE;
        bsp_esc_set_pulse(i, THRUSTER_MAX_PULSE);
    }
    vTaskDelay(pdMS_TO_TICKS(CONFIG_ESC_CALIB_HIGH_TIME_MS));

    ESP_LOGW(TAG, "calibration: hold stop %dus for %dms",
             THRUSTER_STOP_PULSE, CONFIG_ESC_CALIB_STOP_TIME_MS);
    for (size_t i = 0; i < THRUSTER_COUNT; i++) {
        s_thrusters[i].current_pulse = THRUSTER_STOP_PULSE;
        s_thrusters[i].target_pulse  = THRUSTER_STOP_PULSE;
        bsp_esc_set_pulse(i, THRUSTER_STOP_PULSE);
    }
    vTaskDelay(pdMS_TO_TICKS(CONFIG_ESC_CALIB_STOP_TIME_MS));

    s_calibrating = false;
    ESP_LOGW(TAG, "calibration done: ESCs should beep and ready");
}

bool thruster_is_calibrating(void)
{
    return s_calibrating;
}

uint32_t thruster_get_current_pulse(size_t idx)
{
    return (idx < THRUSTER_COUNT) ? s_thrusters[idx].current_pulse : 0;
}

uint32_t thruster_get_target_pulse(size_t idx)
{
    return (idx < THRUSTER_COUNT) ? s_thrusters[idx].target_pulse : 0;
}

int thruster_get_speed(size_t idx)
{
    return (idx < THRUSTER_COUNT) ? (int)s_thrusters[idx].speed : 0;
}

/* 后台缓动：每步向目标靠近，反向时必然缓慢通过中位。 */
void thruster_task(void *arg)
{
    (void)arg;
    while (true) {
        if (!s_calibrating) {
            for (size_t i = 0; i < THRUSTER_COUNT; i++) {
                uint32_t cur = s_thrusters[i].current_pulse;
                const uint32_t tgt = s_thrusters[i].target_pulse;
                const uint32_t step = CONFIG_ESC_RAMP_STEP_US;

                if (cur < tgt) {
                    cur += step;
                    if (cur > tgt) cur = tgt;
                } else if (cur > tgt) {
                    cur -= step;
                    if (cur < tgt) cur = tgt;
                }
                if (cur != s_thrusters[i].current_pulse) {
                    s_thrusters[i].current_pulse = cur;
                    bsp_esc_set_pulse(i, cur);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_ESC_RAMP_INTERVAL_MS));
    }
}
