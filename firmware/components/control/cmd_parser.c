/* 文本指令协议：T1|T2|T3/TALL/START/STOP/PULSE/STATUS/CAL/HELP。
 * 每条指令回传 XX_OK 或 XX_ERR:<原因>，回传到发起指令的传输。
 */
#include "cmd_parser.h"

#include "calibration_capture.h"
#include "control_mode.h"
#include "stabilize.h"
#include "thruster.h"
#include "training_session.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "cmd";

typedef struct {
    const cmd_transport_t *trans;
    char line[CMD_LINE_MAX];
} cmd_msg_t;

static QueueHandle_t s_cmd_q;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000U);
}

static bool training_controls_locked(void)
{
    training_status_t ts;
    training_session_get_status(&ts, now_ms());
    return ts.state == TRAIN_STATE_RUNNING || ts.state == TRAIN_STATE_PAUSED;
}

/* 组装一行（含 CRLF）并发往指定传输，保证整行原子。 */
static void send_line(const cmd_transport_t *t, const char *fmt, ...)
{
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (len < 0) return;

    size_t n = (size_t)len;
    if (n + 2 < sizeof(buf)) {
        buf[n]     = '\r';
        buf[n + 1] = '\n';
        buf[n + 2] = '\0';
        n += 2;
    }
    if (t && t->send) {
        t->send(t->ctx, buf, n);
    }
}

static void handle_help(const cmd_transport_t *t)
{
    send_line(t, "HELP");
    send_line(t, "  START              release emergency stop / enable output");
    send_line(t, "  STOP               emergency stop (latch, all neutral now)");
    send_line(t, "  MODE [assist|active|impedance]   query or set control mode");
    send_line(t, "  T1|T2|T3 <speed>   set thruster speed -1000..1000");
    send_line(t, "  TALL <speed>       set all thrusters");
    send_line(t, "  JOY <x> <y>        vector: X=rudders diff, Y=propulsion");
    send_line(t, "  STAB ON|OFF|ZERO   attitude stabilization");
    send_line(t, "  TRAIN START|PAUSE|RESUME|STOP|RESET   training session");
    send_line(t, "  TRAIN HEIGHT <cm>  set body height 100..230cm");
    send_line(t, "  TRAIN SHANK <cm>   measured shank 20..70cm; 0=estimate");
    send_line(t, "  TRAIN GOAL <rom_min> <rom_max> <spm>|OFF   optional session target");
    send_line(t, "  CAPTURE START [label]|STOP|CLEAR   temporary high-rate IMU capture");
    send_line(t, "  PULSE <n> <us>     direct pulse 800..2200us (debug)");
    send_line(t, "  STATUS             show current/target pulses");
    send_line(t, "  CAL                re-run calibration (~6s)");
}

static void handle_cmd(const cmd_transport_t *t, const char *line)
{
    char cmd[16];
    if (sscanf(line, "%15s", cmd) != 1) {
        send_line(t, "ERR:empty");
        return;
    }

    if (strcasecmp(cmd, "HELP") == 0 || strcasecmp(cmd, "?") == 0) {
        handle_help(t);
    } else if (strcasecmp(cmd, "START") == 0) {
        /* 解除急停闩锁，恢复输出 */
        control_estop_release();
        send_line(t, "START_OK");
    } else if (strcasecmp(cmd, "STOP") == 0) {
        /* 急停：立即回中位并闩锁（控制环/后续指令都被封锁，直到 START） */
        control_estop_latch();
        training_session_pause(now_ms());
        send_line(t, "STOP_OK");
    } else if (strcasecmp(cmd, "MODE") == 0) {
        char sub[16];
        if (sscanf(line, "%*s %15s", sub) != 1) {
            send_line(t, "MODE:%s", control_mode_name(control_mode_get()));
        } else if (training_controls_locked()) {
            send_line(t, "MODE_ERR:training_active");
        } else if (strcasecmp(sub, "assist") == 0 || strcasecmp(sub, "0") == 0) {
            control_mode_set(MODE_ASSIST);
            send_line(t, "MODE_OK:assist");
        } else if (strcasecmp(sub, "active") == 0 || strcasecmp(sub, "1") == 0) {
            control_mode_set(MODE_ACTIVE);
            send_line(t, "MODE_OK:active");
        } else if (strcasecmp(sub, "impedance") == 0 || strcasecmp(sub, "2") == 0) {
            control_mode_set(MODE_IMPEDANCE);
            send_line(t, "MODE_OK:impedance");
        } else {
            send_line(t, "MODE_ERR:arg");
        }
    } else if (strcasecmp(cmd, "TRAIN") == 0) {
        const uint32_t now = now_ms();
        const stabilize_status_t *st = stabilize_get_status();
        training_status_t ts;
        char sub[16];
        training_session_get_status(&ts, now);
        if (sscanf(line, "%*s %15s", sub) != 1) {
            send_line(t, "TRAIN:%s phase=%s id=%lu time=%lus steps=%lu good=%lu score=%.0f%% goal=%d rom=%.0f-%.0f cadence=%.0f",
                      training_state_name(ts.state), training_phase_name(ts.phase),
                      (unsigned long)ts.sessionId,
                      (unsigned long)(ts.elapsedMs / 1000U),
                      (unsigned long)ts.steps, (unsigned long)ts.qualifiedSteps,
                      ts.qualifiedPct, (int)ts.goalEnabled,
                      ts.goalRomMinDeg, ts.goalRomMaxDeg, ts.goalCadenceSpm);
        } else if (strcasecmp(sub, "START") == 0) {
            if (control_is_estop()) {
                send_line(t, "TRAIN_ERR:estop");
            } else if (training_session_start(now, st->motionDeg, st->lateralDeg,
                                              st->mode, st->imuValid,
                                              control_is_estop())) {
                send_line(t, "TRAIN_OK:started");
            } else {
                send_line(t, "TRAIN_ERR:imu_or_state");
            }
        } else if (strcasecmp(sub, "PAUSE") == 0) {
            send_line(t, training_session_pause(now) ? "TRAIN_OK:paused" : "TRAIN_ERR:state");
        } else if (strcasecmp(sub, "RESUME") == 0) {
            if (control_is_estop()) {
                send_line(t, "TRAIN_ERR:estop");
            } else {
                send_line(t, training_session_resume(now, st->motionDeg, st->lateralDeg,
                                                     st->imuValid)
                                 ? "TRAIN_OK:resumed" : "TRAIN_ERR:imu_or_state");
            }
        } else if (strcasecmp(sub, "STOP") == 0) {
            send_line(t, training_session_stop(now) ? "TRAIN_OK:finished" : "TRAIN_ERR:state");
        } else if (strcasecmp(sub, "RESET") == 0) {
            if (training_controls_locked()) {
                send_line(t, "TRAIN_ERR:session_active");
            } else {
                training_session_reset();
                send_line(t, "TRAIN_OK:reset");
            }
        } else if (strcasecmp(sub, "HEIGHT") == 0) {
            float height = 0.0f;
            if (training_controls_locked()) {
                send_line(t, "TRAIN_ERR:session_active");
            } else if (sscanf(line, "%*s %*s %f", &height) != 1) {
                send_line(t, "TRAIN_ERR:height_arg");
            } else if (training_session_set_height(height)) {
                send_line(t, "TRAIN_OK:height=%.1fcm", height);
            } else {
                send_line(t, "TRAIN_ERR:height_range_100_230");
            }
        } else if (strcasecmp(sub, "SHANK") == 0) {
            float shank = 0.0f;
            if (training_controls_locked()) {
                send_line(t, "TRAIN_ERR:session_active");
            } else if (sscanf(line, "%*s %*s %f", &shank) != 1) {
                send_line(t, "TRAIN_ERR:shank_arg");
            } else if (training_session_set_shank_length(shank)) {
                send_line(t, "TRAIN_OK:shank=%.1fcm", shank);
            } else {
                send_line(t, "TRAIN_ERR:shank_range_20_70_or_0");
            }
        } else if (strcasecmp(sub, "GOAL") == 0) {
            char arg[8];
            float romMin = 0.0f, romMax = 0.0f, cadence = 0.0f;
            if (training_controls_locked()) {
                send_line(t, "TRAIN_ERR:session_active");
            } else if (sscanf(line, "%*s %*s %7s", arg) == 1 &&
                       strcasecmp(arg, "OFF") == 0) {
                send_line(t, training_session_set_goal(false, 0.0f, 0.0f, 0.0f)
                                 ? "TRAIN_OK:goal=off" : "TRAIN_ERR:state");
            } else if (sscanf(line, "%*s %*s %f %f %f",
                              &romMin, &romMax, &cadence) != 3) {
                send_line(t, "TRAIN_ERR:goal_arg");
            } else if (training_session_set_goal(true, romMin, romMax, cadence)) {
                send_line(t, "TRAIN_OK:goal rom=%.1f-%.1fdeg cadence=%.1fspm",
                          romMin, romMax, cadence);
            } else {
                send_line(t, "TRAIN_ERR:goal_range_rom_8_60_span2_cadence_5_75");
            }
        } else {
            send_line(t, "TRAIN_ERR:arg");
        }
    } else if (strcasecmp(cmd, "CAPTURE") == 0) {
        const uint32_t now = now_ms();
        const stabilize_status_t *st = stabilize_get_status();
        calibration_capture_status_t cs;
        char sub[16];
        calibration_capture_get_status(&cs, now);
        if (sscanf(line, "%*s %15s", sub) != 1) {
            send_line(t, "CAPTURE:%s label=%s samples=%lu/%lu time=%.1fs rate=%.1fHz full=%d",
                      calibration_capture_state_name(cs.state),
                      cs.label[0] ? cs.label : "-",
                      (unsigned long)cs.sampleCount, (unsigned long)cs.capacity,
                      (double)cs.durationMs / 1000.0, (double)cs.sampleRateHz,
                      (int)cs.full);
        } else if (strcasecmp(sub, "START") == 0) {
            char label[32] = "unlabeled";
            (void)sscanf(line, "%*s %*s %31s", label);
            send_line(t, calibration_capture_start(now, st->imuValid, label)
                             ? "CAPTURE_OK:started" : "CAPTURE_ERR:imu_or_state");
        } else if (strcasecmp(sub, "STOP") == 0) {
            send_line(t, calibration_capture_stop()
                             ? "CAPTURE_OK:stopped" : "CAPTURE_ERR:state");
        } else if (strcasecmp(sub, "CLEAR") == 0) {
            send_line(t, calibration_capture_clear()
                             ? "CAPTURE_OK:cleared" : "CAPTURE_ERR:recording_or_exporting");
        } else {
            send_line(t, "CAPTURE_ERR:arg");
        }
    } else if (strcasecmp(cmd, "STATUS") == 0) {
        send_line(t,
                  "STATUS:T1=%lu/%luus,T2=%lu/%luus,T3=%lu/%luus,cal=%d",
                  (unsigned long)thruster_get_current_pulse(0),
                  (unsigned long)thruster_get_target_pulse(0),
                  (unsigned long)thruster_get_current_pulse(1),
                  (unsigned long)thruster_get_target_pulse(1),
                  (unsigned long)thruster_get_current_pulse(2),
                  (unsigned long)thruster_get_target_pulse(2),
                  (int)thruster_is_calibrating());
    } else if (strcasecmp(cmd, "CAL") == 0) {
        if (training_controls_locked()) { send_line(t, "CAL_ERR:training_active"); return; }
        send_line(t, "CAL_OK:running");
        thruster_calibrate();
        send_line(t, "CAL_OK");
    } else if (strcasecmp(cmd, "TALL") == 0) {
        if (control_is_estop()) { send_line(t, "ERR:estop"); return; }
        if (training_controls_locked()) { send_line(t, "TALL_ERR:training_active"); return; }
        int speed = 0;
        if (sscanf(line, "%*s %d", &speed) != 1) {
            send_line(t, "TALL_ERR:arg");
            return;
        }
        if (speed < THRUSTER_SPEED_MIN || speed > THRUSTER_SPEED_MAX) {
            send_line(t, "TALL_ERR:range");
            return;
        }
        for (size_t i = 0; i < THRUSTER_COUNT; i++) {
            /* 主动模式 T2 必须保持中位；TALL 只影响左右纠偏通道。 */
            thruster_set_speed(i, (control_mode_get() == MODE_ACTIVE && i == 1U) ? 0 : speed);
        }
        send_line(t, "TALL_OK:%d", speed);
    } else if (strcasecmp(cmd, "PULSE") == 0) {
        if (control_is_estop()) { send_line(t, "ERR:estop"); return; }
        if (training_controls_locked()) { send_line(t, "PULSE_ERR:training_active"); return; }
        int idx = 0, us = 0;
        if (sscanf(line, "%*s %d %d", &idx, &us) != 2) {
            send_line(t, "PULSE_ERR:arg");
            return;
        }
        if (idx < 1 || idx > THRUSTER_COUNT) {
            send_line(t, "PULSE_ERR:channel");
            return;
        }
        if (control_mode_get() == MODE_ACTIVE && idx == 2 &&
            us != CONFIG_ESC_STOP_PULSE_US) {
            send_line(t, "PULSE_ERR:active_t2_locked");
            return;
        }
        thruster_set_pulse((size_t)(idx - 1), (uint32_t)us);
        send_line(t, "PULSE_OK:%dus", (int)thruster_get_current_pulse((size_t)(idx - 1)));
    } else if (strcasecmp(cmd, "STAB") == 0) {
        const stabilize_status_t *st = stabilize_get_status();
        char sub[16];
        if (sscanf(line, "%*s %15s", sub) != 1) {
            /* 无参数 = 查询（含 IMU 诊断） */
            send_line(t, "STAB:en=%d roll=%.1f pitch=%.1f po=%d ro=%d stop=%d "
                         "imu=%d rx=%lu frames=%lu cksum=%lu age=%lums",
                      (int)st->enabled, st->rollDeg, st->pitchDeg,
                      (int)st->pitchOut, (int)st->rollOut,
                      (int)st->pitchAngleStop,
                      (int)st->imuValid,
                      (unsigned long)st->rxBytes,
                      (unsigned long)st->validFrames,
                      (unsigned long)st->checksumErrors,
                      (unsigned long)st->lastFrameAgeMs);
        } else if (strcasecmp(sub, "ON") == 0) {
            stabilize_set_enabled(true);
            send_line(t, "STAB_OK:on");
        } else if (strcasecmp(sub, "OFF") == 0) {
            if (training_controls_locked()) {
                send_line(t, "STAB_ERR:training_active");
            } else {
                stabilize_set_enabled(false);
                send_line(t, "STAB_OK:off");
            }
        } else if (strcasecmp(sub, "ZERO") == 0) {
            if (training_controls_locked()) {
                send_line(t, "STAB_ERR:training_active");
            } else {
                stabilize_zero();
                send_line(t, "STAB_OK:zeroed");
            }
        } else {
            send_line(t, "STAB_ERR:arg");
        }
    } else if (strcasecmp(cmd, "JOY") == 0) {
        if (control_is_estop()) { send_line(t, "ERR:estop"); return; }
        if (training_controls_locked()) { send_line(t, "JOY_ERR:training_active"); return; }
        int x = 0, y = 0;
        if (sscanf(line, "%*s %d %d", &x, &y) != 2) {
            send_line(t, "JOY_ERR:arg");
            return;
        }
        if (x < -1000) x = -1000;
        if (x > 1000)  x = 1000;
        if (y < -1000) y = -1000;
        if (y > 1000)  y = 1000;
        if (control_mode_get() == MODE_ACTIVE) y = 0;
        /* 矢量力：Y->推进，X->左右舵差动 */
        thruster_set_speed(0, x);
        thruster_set_speed(1, y);
        thruster_set_speed(2, -x);
        send_line(t, "JOY_OK:%d,%d", x, y);
    } else if ((cmd[0] == 'T' || cmd[0] == 't') && isdigit((unsigned char)cmd[1])) {
        if (control_is_estop()) { send_line(t, "ERR:estop"); return; }
        if (training_controls_locked()) { send_line(t, "T_ERR:training_active"); return; }
        const int idx = cmd[1] - '0';
        if (idx < 1 || idx > THRUSTER_COUNT) {
            send_line(t, "ERR:channel");
            return;
        }
        int speed = 0;
        if (sscanf(line, "%*s %d", &speed) != 1) {
            send_line(t, "T%d_ERR:arg", idx);
            return;
        }
        if (speed < THRUSTER_SPEED_MIN || speed > THRUSTER_SPEED_MAX) {
            send_line(t, "T%d_ERR:range", idx);
            return;
        }
        if (control_mode_get() == MODE_ACTIVE && idx == 2 && speed != 0) {
            send_line(t, "T2_ERR:active_locked");
            return;
        }
        thruster_set_speed((size_t)(idx - 1), speed);
        send_line(t, "T%d_OK:%d", idx, speed);
    } else {
        send_line(t, "ERR:unknown:%s", cmd);
    }
}

static void cmd_task(void *arg)
{
    (void)arg;
    cmd_msg_t msg;
    while (true) {
        if (xQueueReceive(s_cmd_q, &msg, portMAX_DELAY) == pdTRUE) {
            handle_cmd(msg.trans, msg.line);
        }
    }
}

void cmd_parser_init(void)
{
    s_cmd_q = xQueueCreate(16, sizeof(cmd_msg_t));
    if (s_cmd_q == NULL) {
        ESP_LOGE(TAG, "failed to create cmd queue");
        return;
    }
    xTaskCreate(cmd_task, "cmd", 4096, NULL, 4, NULL);
}

void cmd_parser_feed_line(const cmd_transport_t *t, const char *line)
{
    if (s_cmd_q == NULL) return;

    cmd_msg_t msg = { .trans = t };
    size_t len = strlen(line);
    if (len >= sizeof(msg.line)) {
        len = sizeof(msg.line) - 1;
    }
    memcpy(msg.line, line, len);
    msg.line[len] = '\0';

    /* 非阻塞入队；队列满则丢弃本行（防拥塞）。 */
    if (xQueueSend(s_cmd_q, &msg, 0) != pdTRUE) {
        ESP_LOGW(TAG, "cmd queue full, dropped: %s", msg.line);
    }
}
