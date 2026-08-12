#include "control_mode.h"

#include "thruster.h"

#include "esp_log.h"

static const char *TAG = "mode";

static const char *const MODE_NAMES[] = { "assist", "active", "impedance" };

static control_mode_t s_mode = MODE_ASSIST;
static volatile bool  s_estop = false;

void control_mode_init(void)
{
    s_mode  = MODE_ASSIST;
    s_estop = false;
    ESP_LOGI(TAG, "control mode: %s (caps assist=%d active=%d impedance=%d)",
             MODE_NAMES[MODE_ASSIST],
             CONFIG_MODE_ASSIST_MAX, CONFIG_MODE_ACTIVE_MAX,
             CONFIG_MODE_IMPEDANCE_MAX);
}

control_mode_t control_mode_get(void)
{
    return s_mode;
}

const char *control_mode_name(control_mode_t m)
{
    return (m <= MODE_IMPEDANCE) ? MODE_NAMES[m] : "?";
}

bool control_mode_set(control_mode_t m)
{
    if (m > MODE_IMPEDANCE) return false;
    s_mode = m;
    if (m == MODE_ACTIVE) {
        /* 切入主动模式先清掉所有遗留手动目标，随后控制环只会重建
         * T1/T3 横向纠偏；T2 始终保持中位。 */
        thruster_set_pulse(0, CONFIG_ESC_STOP_PULSE_US);
        thruster_set_pulse(1, CONFIG_ESC_STOP_PULSE_US);
        thruster_set_pulse(2, CONFIG_ESC_STOP_PULSE_US);
    }
    ESP_LOGI(TAG, "mode -> %s", MODE_NAMES[m]);
    return true;
}

int control_mode_cap(void)
{
    switch (s_mode) {
    case MODE_ACTIVE:    return CONFIG_MODE_ACTIVE_MAX;
    case MODE_IMPEDANCE: return CONFIG_MODE_IMPEDANCE_MAX;
    default:             return CONFIG_MODE_ASSIST_MAX;
    }
}

bool control_is_estop(void)
{
    return s_estop;
}

void control_estop_latch(void)
{
    s_estop = true;
    thruster_estop();   /* 三路立即回中位（不缓动） */
    ESP_LOGW(TAG, "ESTOP latched, all thrusters -> neutral now");
}

void control_estop_release(void)
{
    if (s_estop) {
        ESP_LOGI(TAG, "ESTOP released");
    }
    s_estop = false;
}
