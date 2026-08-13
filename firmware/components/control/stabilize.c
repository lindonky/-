/* 姿态稳定应用：IMU RX 任务喂字节给 JY61P 解析器；控制任务每 STAB_PERIOD_MS
 * 跑一次控制律并把输出直接映射到三路电调脉宽（绕过手动缓动，保证响应速度）。 */
#include "stabilize.h"

#include "attitude_control.h"
#include "bsp.h"
#include "calibration_capture.h"
#include "control_mode.h"
#include "driver/imu_uart.h"
#include "driver/jy61p.h"
#include "thruster.h"
#include "training_session.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

#define STAB_STOP_PULSE CONFIG_ESC_STOP_PULSE_US
#define STAB_PULSE_RANGE_US 700  /* 2200-1500 */

static const char *TAG = "stab";

static jy61p_t            s_imu;
static attitude_ctrl_t    s_pitch_ctrl;
static attitude_ctrl_t    s_roll_ctrl;
static stabilize_status_t s_status;
static float              s_zeroRoll = 0.0f;
static float              s_zeroPitch = 0.0f;
static float              s_zeroYaw = 0.0f;
static float              s_rawRoll = 0.0f;    /* 零位基于原始角 */
static float              s_rawPitch = 0.0f;
static float              s_rawYaw = 0.0f;

/* 归一化输出(-1000..1000) -> 脉宽(800..2200)，钳位交给 thruster_set_pulse */
static uint32_t output_to_pulse(float out)
{
    int32_t pulse = (int32_t)STAB_STOP_PULSE +
                    (int32_t)(out * STAB_PULSE_RANGE_US / 1000.0f);
    return (uint32_t)pulse;
}

/* 按模式上限钳制 |输出|（保留符号） */
static float clamp_abs(float out, float cap)
{
    if (out > cap)  return cap;
    if (out < -cap) return -cap;
    return out;
}

/* 角度差折回 ±180° */
static float wrap_angle(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/* --- IMU 接收任务：读 UART2 -> 喂解析器 -> 更新相对姿态 --- */
static void imu_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[64];
    uint32_t prevSeq = 0U;

    while (true) {
        int r = imu_uart_read(buf, sizeof(buf), pdMS_TO_TICKS(20));
        const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000U);
        s_status.rxBytes += (uint32_t)r;
        for (int i = 0; i < r; i++) {
            jy61p_push_byte(&s_imu, buf[i], now);
        }

        jy61p_data_t d;
        if (jy61p_get_new(&s_imu, prevSeq, &d)) {
            prevSeq = d.sequence;
            s_rawRoll  = d.rollDeg;
            s_rawPitch = d.pitchDeg;
            s_rawYaw   = d.yawDeg;
            s_status.imuValid  = true;
            s_status.sampleSeq = d.sequence;
            s_status.rollDeg   = wrap_angle(d.rollDeg - s_zeroRoll);
            s_status.pitchDeg  = wrap_angle(d.pitchDeg - s_zeroPitch);
            s_status.yawDeg    = wrap_angle(d.yawDeg - s_zeroYaw);
            s_status.gyroDps   = d.gyroYDps;   /* 俯仰轴角速度 */
            s_status.gyroRollDps = d.gyroXDps; /* 横滚轴角速度 */
            s_status.gyroXDps  = d.gyroXDps;
            s_status.gyroYDps  = d.gyroYDps;
            s_status.gyroZDps  = d.gyroZDps;
            s_status.accelXG   = d.accelXG;
            s_status.accelYG   = d.accelYG;
            s_status.accelZG   = d.accelZG;
            s_status.sampleTimestampMs = d.timestampMs;

            /* 训练主轴跟随控制轴配置。完整遥测仍保留 JY61P 原始轴。 */
#if CONFIG_STAB_SWAP_PITCH_ROLL
            const float trainPitch = s_status.rollDeg;
            const float trainRoll = s_status.pitchDeg;
            const float trainPitchGyro = d.gyroXDps;
            const float trainRollGyro = d.gyroYDps;
#else
            const float trainPitch = s_status.pitchDeg;
            const float trainRoll = s_status.rollDeg;
            const float trainPitchGyro = d.gyroYDps;
            const float trainRollGyro = d.gyroXDps;
#endif
            s_status.motionDeg = trainPitch;
            s_status.lateralDeg = trainRoll;
            s_status.motionGyroDps = trainPitchGyro;
            s_status.lateralGyroDps = trainRollGyro;
            const training_sample_t sample = {
                .nowMs = d.timestampMs,
                .imuValid = true,
                .estop = control_is_estop(),
                .mode = (int)control_mode_get(),
                .pitchDeg = trainPitch,
                .rollDeg = trainRoll,
                .gyroXDps = trainRollGyro,
                .gyroYDps = trainPitchGyro,
                .gyroZDps = d.gyroZDps,
                .accelXG = d.accelXG,
                .accelYG = d.accelYG,
                .accelZG = d.accelZG,
                .correctionOut = s_status.rollOut,
                .controlCap = (float)control_mode_cap(),
            };
            training_session_feed(&sample);
            const calibration_capture_sample_t captureSample = {
                .nowMs = d.timestampMs,
                .sequence = d.sequence,
                .imuValid = true,
                .estop = control_is_estop(),
                .stabilizeEnabled = s_status.enabled,
                .mode = (int)control_mode_get(),
                .rollDeg = s_status.rollDeg,
                .pitchDeg = s_status.pitchDeg,
                .yawDeg = s_status.yawDeg,
                .motionDeg = trainPitch,
                .lateralDeg = trainRoll,
                .gyroXDps = d.gyroXDps,
                .gyroYDps = d.gyroYDps,
                .gyroZDps = d.gyroZDps,
                .accelXG = d.accelXG,
                .accelYG = d.accelYG,
                .accelZG = d.accelZG,
                .pitchOut = s_status.pitchOut,
                .rollOut = s_status.rollOut,
                .t1PulseUs = thruster_get_current_pulse(0),
                .t2PulseUs = thruster_get_current_pulse(1),
                .t3PulseUs = thruster_get_current_pulse(2),
            };
            calibration_capture_feed(&captureSample);
        }

        /* 诊断计数（轻量，仅读） */
        s_status.validFrames    = s_imu.validFrames;
        s_status.checksumErrors = s_imu.checksumErrors;
        s_status.lastFrameAgeMs = (s_imu.latest.timestampMs != 0U)
                                      ? (now - s_imu.latest.timestampMs) : 0U;
    }
}

/* --- 控制任务：姿态 -> 推进/左右舵 --- */
static void control_task(void *arg)
{
    (void)arg;
    while (true) {
        const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000U);
        const uint32_t last = jy61p_last_frame_ms(&s_imu);
        const bool stale = (last != 0U) &&
                           ((now - last) > CONFIG_STAB_STALE_TIMEOUT_MS);

        s_status.staleFault = stale;
        if (!s_status.imuValid || stale) {
            s_status.imuValid = false;
        }

        s_status.mode  = (int)control_mode_get();
        s_status.estop = control_is_estop();

        if (training_session_take_fall_event()) {
            ESP_LOGE(TAG, "suspected fall confirmed: latching emergency stop");
            control_estop_latch();
            s_status.estop = true;
        }

        if (s_status.enabled) {
            if (s_status.imuValid && !s_status.staleFault &&
                !thruster_is_calibrating()) {
                if (s_status.estop) {
                    /* 急停闩锁：保持中位不输出（thruster_estop 已立即回中，
                     * 这里每周期维持，防止任何来源覆盖） */
                    s_status.pitchOut = 0.0f;
                    s_status.rollOut  = 0.0f;
                    thruster_stop_all();
                    vTaskDelay(pdMS_TO_TICKS(CONFIG_STAB_PERIOD_MS));
                    continue;
                }
                /* ---- 姿态→推力映射（与网页 3D 显示同一套轴约定）----
                 * 默认：pitch(JY61P Y轴, 前后抬腿) -> 推进(T2)
                 *       roll (JY61P X轴, 左右倾)  -> 左舵(T1)=+roll / 右舵(T3)=-roll 差动
                 * 方向调整（idf.py menuconfig -> ESC Control）：
                 *   CONFIG_STAB_PITCH_SIGN     ±1：前后抬腿方向反了改 -1
                 *   CONFIG_STAB_ROLL_SIGN      ±1：左右倾方向反了改 -1
                 *   CONFIG_STAB_SWAP_PITCH_ROLL：模块横装 90° 时勾选，pitch/roll 互换
                 */
#if CONFIG_STAB_SWAP_PITCH_ROLL
                const float ctrlPitchMeas = s_status.rollDeg;
                const float ctrlPitchGyro = s_status.gyroRollDps;
                const float ctrlRollMeas  = s_status.pitchDeg;
                const float ctrlRollGyro  = s_status.gyroDps;
#else
                const float ctrlPitchMeas = s_status.pitchDeg;
                const float ctrlPitchGyro = s_status.gyroDps;
                const float ctrlRollMeas  = s_status.rollDeg;
                const float ctrlRollGyro  = s_status.gyroRollDps;
#endif
                attitude_ctrl_update(&s_pitch_ctrl, ctrlPitchMeas,
                                     CONFIG_STAB_PITCH_SIGN * ctrlPitchGyro);
                attitude_ctrl_update(&s_roll_ctrl, ctrlRollMeas,
                                     CONFIG_STAB_ROLL_SIGN * ctrlRollGyro);

                s_status.pitchOut = CONFIG_STAB_PITCH_SIGN * s_pitch_ctrl.output;
                s_status.rollOut  = CONFIG_STAB_ROLL_SIGN * s_roll_ctrl.output;
                s_status.pitchInDeadband = s_pitch_ctrl.inDeadband;
                /* 主动模式 T2 永远中位，俯仰超控制角不会关闭横向纠偏。
                 * 训练模块使用独立、可配置的 ROM 范围评价小腿动作。 */
                s_status.pitchAngleStop  =
                    (control_mode_get() != MODE_ACTIVE) && s_pitch_ctrl.angleStop;

                /* 按当前模式统一限幅（PWM 幅值上限） */
                const float cap = (float)control_mode_cap();
                s_status.pitchOut = clamp_abs(s_status.pitchOut, cap);
                s_status.rollOut  = clamp_abs(s_status.rollOut, cap);

                /* ---- 三模式输出映射 ----
                 * 助力 assist   : pitch->推进；roll->左右舵差动
                 * 主动 active   : 推进保持中位；左右舵只做横向腿位纠偏
                 * 阻抗 impedance: 抬脚(pitch>0) -> 推进反转(负输出)；左右舵维持姿态
                 *                 （反转脉宽需双向电调；单向电调只会减速到停） */
                float t2;
                switch (control_mode_get()) {
                case MODE_ACTIVE:
                    t2 = 0.0f;  /* 患者主动抬腿：主推进器不提供助推 */
                    break;
                case MODE_IMPEDANCE:
                    t2 = -s_status.pitchOut;   /* 阻抗：抵抗抬腿 = 反向 */
                    break;
                default: /* MODE_ASSIST */
                    t2 = s_status.pitchOut;
                    break;
                }
                t2 = clamp_abs(t2, cap);

                /* 俯仰 -> 推进；横滚 -> 左右舵差动（直接脉宽，响应快） */
                thruster_set_pulse(1, output_to_pulse(t2));
                thruster_set_pulse(0, output_to_pulse(s_status.rollOut));
                thruster_set_pulse(2, output_to_pulse(-s_status.rollOut));
            } else {
                /* 数据失效：停止三路输出 */
                s_status.pitchOut = 0.0f;
                s_status.rollOut  = 0.0f;
                thruster_stop_all();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CONFIG_STAB_PERIOD_MS));
    }
}

void stabilize_init(void)
{
    jy61p_init(&s_imu);
    ESP_ERROR_CHECK(imu_uart_init());

    control_mode_init();

    attitude_ctrl_init(&s_pitch_ctrl);
    attitude_ctrl_init(&s_roll_ctrl);
    /* 默认目标 0°（即归零位为水平目标）。
     * 控制器内部限幅提到 1000：每模式上限由 control_task 统一按 mode 钳制，
     * 避免 800 的固定限幅挡住主动模式更高的上限。 */
    attitude_ctrl_set_target(&s_pitch_ctrl, 0.0f);
    attitude_ctrl_set_target(&s_roll_ctrl, 0.0f);
    attitude_ctrl_set_output_limit(&s_pitch_ctrl, 1000.0f);
    attitude_ctrl_set_output_limit(&s_roll_ctrl, 1000.0f);

#if CONFIG_STAB_ENABLE_ON_BOOT
    s_status.enabled = true;
#else
    s_status.enabled = false;
#endif

    xTaskCreatePinnedToCore(imu_rx_task, "imu_rx", 3072, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(control_task, "stab", 3072, NULL, 4, NULL, 1);
    ESP_LOGI(TAG, "stabilization ready (period %dms, deadband %ddeg, max %ddeg)",
             CONFIG_STAB_PERIOD_MS, CONFIG_STAB_DEADBAND_DEG,
             CONFIG_STAB_MAX_ANGLE_DEG);
}

void stabilize_set_enabled(bool en)
{
    s_status.enabled = en;
    if (!en) {
        s_status.pitchOut = 0.0f;
        s_status.rollOut  = 0.0f;
        thruster_stop_all();
    }
    ESP_LOGI(TAG, "%s", en ? "enabled" : "disabled");
}

bool stabilize_is_enabled(void)
{
    return s_status.enabled;
}

void stabilize_zero(void)
{
    s_zeroRoll  = s_rawRoll;
    s_zeroPitch = s_rawPitch;
    s_zeroYaw   = s_rawYaw;
    ESP_LOGI(TAG, "zeroed (r %.1f p %.1f y %.1f)", s_zeroRoll, s_zeroPitch, s_zeroYaw);
}

const stabilize_status_t *stabilize_get_status(void)
{
    return &s_status;
}
