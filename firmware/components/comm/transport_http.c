/* WiFi 热点 + HTTP 网页控制。复用 cmd_parser 协议：网页指令经 /api/cmd
 * 走与 UART/BLE 完全相同的解析与 XX_OK 回传。
 */
#include "transport_http.h"

#include "calibration_capture.h"
#include "cmd_parser.h"
#include "stabilize.h"
#include "thruster.h"
#include "training_session.h"

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

#define HTTP_RESP_MAX 512

static const char *TAG = "http_ctl";

/* 内嵌网页（components/comm/web_page.html，由 EMBED_TXTFILES 生成符号） */
extern const uint8_t web_page_html_start[] asm("_binary_web_page_html_start");
extern const uint8_t web_page_html_end[]   asm("_binary_web_page_html_end");

/* 内嵌腿部模型（components/comm/leg_model.js） */
extern const uint8_t leg_model_js_start[] asm("_binary_leg_model_js_start");
extern const uint8_t leg_model_js_end[]   asm("_binary_leg_model_js_end");

static char            s_resp_buf[HTTP_RESP_MAX];
static SemaphoreHandle_t s_resp_sem;   /* 协议层回传一行 -> 置位 */
static SemaphoreHandle_t s_req_mutex;  /* 同时只处理一个 HTTP 指令 */
static uint32_t          s_boot_id;    /* 区分设备重启后重复的会话 ID */

/* --- 协议层回传（HTTP 传输）：拷贝到响应缓冲 --- */
static void http_cmd_send(void *ctx, const char *data, size_t len)
{
    (void)ctx;
    size_t n = strlen(s_resp_buf);
    if (n + len < sizeof(s_resp_buf) - 1) {
        memcpy(s_resp_buf + n, data, len);
        s_resp_buf[n + len] = '\0';
    }
    xSemaphoreGive(s_resp_sem);
}

static const cmd_transport_t s_http_transport = {
    .name = "HTTP",
    .send = http_cmd_send,
    .ctx  = NULL,
};

/* --- 页面 --- */
static esp_err_t handle_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, (const char *)web_page_html_start,
                    web_page_html_end - web_page_html_start);
    return ESP_OK;
}

/* --- 腿部模型 JS --- */
static esp_err_t handle_leg_model(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_send(req, (const char *)leg_model_js_start,
                    leg_model_js_end - leg_model_js_start);
    return ESP_OK;
}

/* --- 状态 JSON：三路脉宽 + 姿态 + 稳定输出 --- */
static esp_err_t handle_status(httpd_req_t *req)
{
    const stabilize_status_t *st = stabilize_get_status();
    char buf[HTTP_RESP_MAX];
    const int len = snprintf(
        buf, sizeof(buf),
        "{\"cal\":%d,\"r\":%.1f,\"p\":%.1f,\"y\":%.1f,\"g\":%.1f,"
        "\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,"
        "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"seq\":%lu,\"age\":%lu,\"imu\":%d,"
        "\"stab\":%d,\"mde\":%d,\"es\":%d,\"po\":%d,\"ro\":%d,\"stop\":%d,"
        "\"m\":[{\"s\":%d,\"c\":%lu,\"t\":%lu},"
        "{\"s\":%d,\"c\":%lu,\"t\":%lu},"
        "{\"s\":%d,\"c\":%lu,\"t\":%lu}]}",
        (int)thruster_is_calibrating(),
        st->rollDeg, st->pitchDeg, st->yawDeg, st->gyroDps,
        st->gyroXDps, st->gyroYDps, st->gyroZDps,
        st->accelXG, st->accelYG, st->accelZG,
        (unsigned long)st->sampleSeq, (unsigned long)st->lastFrameAgeMs,
        (int)st->imuValid,
        (int)st->enabled, (int)st->mode, (int)st->estop,
        (int)st->pitchOut, (int)st->rollOut,
        (int)st->pitchAngleStop,
        thruster_get_speed(0), (unsigned long)thruster_get_current_pulse(0),
        (unsigned long)thruster_get_target_pulse(0),
        thruster_get_speed(1), (unsigned long)thruster_get_current_pulse(1),
        (unsigned long)thruster_get_target_pulse(1),
        thruster_get_speed(2), (unsigned long)thruster_get_current_pulse(2),
        (unsigned long)thruster_get_target_pulse(2));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

/* --- 训练会话摘要：计步在 ESP32 高频任务中完成，网页只读取结果 --- */
static esp_err_t handle_training_status(httpd_req_t *req)
{
    training_status_t ts;
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000U);
    training_session_get_status(&ts, now);
    char buf[2048];
    const int len = snprintf(
        buf, sizeof(buf),
        "{\"algo\":\"single-leg-mvp-0.2\",\"device_boot\":%lu,\"state\":\"%s\",\"phase\":\"%s\",\"id\":%lu,\"elapsed_ms\":%lu,\"mode\":%d,"
        "\"steps\":%lu,\"qualified\":%lu,\"invalid\":%lu,\"score_pct\":%.1f,\"last_score_pct\":%.1f,"
        "\"rom_last_deg\":%.1f,\"rom_avg_deg\":%.1f,\"peak_speed_dps\":%.1f,"
        "\"cycle_s\":%.2f,\"lift_s\":%.2f,\"return_s\":%.2f,\"cadence_spm\":%.1f,"
        "\"lateral_deg\":%.1f,\"return_error_deg\":%.1f,"
        "\"height_cm\":%.1f,\"leg_cm_est\":%.1f,\"shank_cm\":%.1f,\"shank_measured\":%d,"
        "\"goal_enabled\":%d,\"goal_rom_min_deg\":%.1f,\"goal_rom_max_deg\":%.1f,"
        "\"goal_cadence_spm\":%.1f,\"goal_cadence_tolerance_spm\":%.1f,"
        "\"step_cm_est\":%.1f,\"step_avg_cm_est\":%.1f,\"distance_cm_est\":%.1f,"
        "\"intervention_mean_pct\":%.1f,\"intervention_peak_pct\":%.1f,\"correction_load_index\":%.1f,"
        "\"quality_flags\":%lu,\"fall_stage\":%u,\"fall_events\":%lu,"
        "\"accel_g\":%.2f,\"gyro_dps\":%.1f,\"fall_tilt_deg\":%.1f,\"event\":\"%s\","
        "\"limits\":{\"lift_start_deg\":%d,\"rom_min_deg\":%d,\"rom_max_deg\":%d,"
        "\"rom_limit_deg\":%d,\"speed_min_dps\":%d,\"speed_max_dps\":%d,"
        "\"cycle_min_ms\":%d,\"cycle_max_ms\":%d,\"lateral_max_deg\":%d,\"return_deg\":%d}}",
        (unsigned long)s_boot_id, training_state_name(ts.state), training_phase_name(ts.phase),
        (unsigned long)ts.sessionId, (unsigned long)ts.elapsedMs, ts.mode,
        (unsigned long)ts.steps, (unsigned long)ts.qualifiedSteps,
        (unsigned long)ts.invalidCycles, ts.qualifiedPct,
        ts.lastTrajectoryScorePct,
        ts.lastRomDeg, ts.averageRomDeg, ts.lastPeakSpeedDps,
        ts.lastCycleSec, ts.lastLiftSec, ts.lastReturnSec, ts.cadenceSpm,
        ts.lastLateralDeg, ts.lastReturnErrorDeg,
        ts.heightCm, ts.estimatedLegCm, ts.shankLengthCm, (int)ts.shankLengthMeasured,
        (int)ts.goalEnabled, ts.goalRomMinDeg, ts.goalRomMaxDeg,
        ts.goalCadenceSpm, ts.goalCadenceToleranceSpm,
        ts.lastEstimatedStepCm, ts.averageEstimatedStepCm,
        ts.totalEstimatedDistanceCm,
        ts.interventionMeanPct, ts.interventionPeakPct, ts.correctionLoadIndex,
        (unsigned long)ts.lastQualityFlags, (unsigned int)ts.fallStage,
        (unsigned long)ts.fallEvents, ts.lastAccelMagnitudeG,
        ts.lastGyroMagnitudeDps, ts.lastFallTiltDeg, ts.lastEvent,
        CONFIG_TRAIN_LIFT_START_DEG, CONFIG_TRAIN_ROM_MIN_DEG,
        CONFIG_TRAIN_ROM_MAX_DEG, CONFIG_TRAIN_ROM_LIMIT_DEG,
        CONFIG_TRAIN_SPEED_MIN_DPS, CONFIG_TRAIN_SPEED_MAX_DPS,
        CONFIG_TRAIN_CYCLE_MIN_MS, CONFIG_TRAIN_CYCLE_MAX_MS,
        CONFIG_TRAIN_LATERAL_MAX_DEG, CONFIG_TRAIN_RETURN_WINDOW_DEG);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, buf, (len > 0 && len < (int)sizeof(buf)) ? len : HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* --- 短时高频标定采集：RAM 状态与停止后流式 CSV --- */
static esp_err_t handle_capture_status(httpd_req_t *req)
{
    calibration_capture_status_t cs;
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000U);
    calibration_capture_get_status(&cs, now);
    char buf[256];
    const int len = snprintf(
        buf, sizeof(buf),
        "{\"state\":\"%s\",\"label\":\"%s\",\"samples\":%lu,\"capacity\":%lu,"
        "\"duration_ms\":%lu,\"rate_hz\":%.1f,\"full\":%d,\"download_ready\":%d}",
        calibration_capture_state_name(cs.state),
        cs.label,
        (unsigned long)cs.sampleCount, (unsigned long)cs.capacity,
        (unsigned long)cs.durationMs, cs.sampleRateHz, (int)cs.full,
        (int)(cs.state == CAPTURE_STATE_READY && cs.sampleCount > 0U));
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

static esp_err_t capture_send_chunk(httpd_req_t *req, const char *data,
                                    int len, size_t capacity)
{
    if (len <= 0 || (size_t)len >= capacity) return ESP_FAIL;
    return httpd_resp_send_chunk(req, data, (size_t)len);
}

static esp_err_t handle_capture_csv(httpd_req_t *req)
{
    calibration_capture_status_t cs;
    if (!calibration_capture_begin_export(&cs)) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        httpd_resp_sendstr(req, "capture is not ready; stop recording first");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "text/csv; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "attachment; filename=alwaysbea-calibration-capture.csv");

    esp_err_t err = ESP_OK;
    char row[384];
    int len = snprintf(row, sizeof(row),
                       "\xEF\xBB\xBF# schema=alwaysbea-calibration-capture-v1\r\n"
                       "# device_boot=%lu,samples=%lu,duration_ms=%lu,rate_hz=%.1f,full=%d\r\n"
                       "# label=%s\r\n"
                       "# angle=deg,gyro=deg/s,accel=g,control=-1000..1000\r\n",
                       (unsigned long)s_boot_id, (unsigned long)cs.sampleCount,
                       (unsigned long)cs.durationMs, cs.sampleRateHz, (int)cs.full,
                       cs.label);
    if (capture_send_chunk(req, row, len, sizeof(row)) != ESP_OK) err = ESP_FAIL;

    static const char header[] =
        "timestamp_ms,elapsed_ms,sequence,roll_deg,pitch_deg,yaw_deg,motion_deg,lateral_deg,"
        "gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,pitch_out,roll_out,"
        "t1_pulse_us,t2_pulse_us,t3_pulse_us,mode,imu_valid,estop,stabilize\r\n";
    if (err == ESP_OK && httpd_resp_send_chunk(req, header, sizeof(header) - 1U) != ESP_OK) {
        err = ESP_FAIL;
    }

    for (uint32_t i = 0U; err == ESP_OK && i < cs.sampleCount; ++i) {
        calibration_capture_record_t r;
        if (!calibration_capture_get_record(i, &r)) {
            err = ESP_FAIL;
            break;
        }
        len = snprintf(
            row, sizeof(row),
            "%lu,%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,"
            "%.3f,%.3f,%.3f,%d,%d,%u,%u,%u,%d,%d,%d,%d\r\n",
            (unsigned long)r.timestampMs,
            (unsigned long)(r.timestampMs - cs.startedMs),
            (unsigned long)r.sequence,
            (double)r.rollCentiDeg / 100.0, (double)r.pitchCentiDeg / 100.0,
            (double)r.yawCentiDeg / 100.0, (double)r.motionCentiDeg / 100.0,
            (double)r.lateralCentiDeg / 100.0,
            (double)r.gyroXDeciDps / 10.0, (double)r.gyroYDeciDps / 10.0,
            (double)r.gyroZDeciDps / 10.0,
            (double)r.accelXMilliG / 1000.0, (double)r.accelYMilliG / 1000.0,
            (double)r.accelZMilliG / 1000.0,
            (int)r.pitchOut, (int)r.rollOut,
            (unsigned int)r.t1PulseUs, (unsigned int)r.t2PulseUs,
            (unsigned int)r.t3PulseUs, (int)r.mode,
            (int)((r.flags & CAPTURE_FLAG_IMU_VALID) != 0U),
            (int)((r.flags & CAPTURE_FLAG_ESTOP) != 0U),
            (int)((r.flags & CAPTURE_FLAG_STABILIZE) != 0U));
        if (capture_send_chunk(req, row, len, sizeof(row)) != ESP_OK) err = ESP_FAIL;
    }

    if (err == ESP_OK) httpd_resp_send_chunk(req, NULL, 0U);
    calibration_capture_end_export();
    return err;
}

/* --- 指令：正文为协议文本，回传 XX_OK/XX_ERR --- */
static esp_err_t handle_cmd(httpd_req_t *req)
{
    char cmd[128];
    int total = 0;
    while (total < (int)sizeof(cmd) - 1) {
        int r = httpd_req_recv(req, cmd + total, (int)sizeof(cmd) - 1 - total);
        if (r <= 0) break;
        total += r;
    }
    cmd[total] = '\0';

    /* 去首尾空白 */
    char *p = cmd;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
        p++;
    }
    size_t len = strlen(p);
    while (len > 0 && (p[len - 1] == ' ' || p[len - 1] == '\r' || p[len - 1] == '\n')) {
        len--;
    }

    if (len == 0) {
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_send(req, "ERR:empty", 9);
        return ESP_OK;
    }
    char cmd2[sizeof(cmd)];
    memcpy(cmd2, p, len);
    cmd2[len] = '\0';

    xSemaphoreTake(s_req_mutex, portMAX_DELAY);
    s_resp_buf[0] = '\0';

    cmd_parser_feed_line(&s_http_transport, cmd2);

    /* 等第一条回传（CAL 约 6s），再留一小段收集多行回传 */
    if (xSemaphoreTake(s_resp_sem, pdMS_TO_TICKS(8000)) == pdTRUE) {
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_send(req, s_resp_buf, strlen(s_resp_buf));
    xSemaphoreGive(s_req_mutex);
    return ESP_OK;
}

/* --- WiFi 热点 --- */
static void wifi_ap_init(void)
{
    /* 容错：netif/事件循环可能已被其它模块创建 */
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    wifi_config_t ap_cfg = { 0 };
    strncpy((char *)ap_cfg.ap.ssid, CONFIG_ESC_AP_SSID, sizeof(ap_cfg.ap.ssid) - 1);
    ap_cfg.ap.ssid_len     = strlen(CONFIG_ESC_AP_SSID);
    ap_cfg.ap.channel      = 1;
    ap_cfg.ap.max_connection = 4;
    if (strlen(CONFIG_ESC_AP_PASSWORD) == 0) {
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    } else {
        ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
        strncpy((char *)ap_cfg.ap.password, CONFIG_ESC_AP_PASSWORD, sizeof(ap_cfg.ap.password) - 1);
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi AP '%s' up, IP 192.168.4.1", CONFIG_ESC_AP_SSID);
}

static void http_server_init(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    cfg.stack_size       = 6144;
    cfg.core_id          = 0; /* Wi-Fi/HTTP 留在 Core 0，实时控制固定到 Core 1。 */

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd start failed");
        return;
    }
    static const httpd_uri_t uri_root   = { .uri = "/",           .method = HTTP_GET,  .handler = handle_root,     .user_ctx = NULL };
    static const httpd_uri_t uri_leg    = { .uri = "/leg.js",     .method = HTTP_GET,  .handler = handle_leg_model,.user_ctx = NULL };
    static const httpd_uri_t uri_status = { .uri = "/api/status", .method = HTTP_GET,  .handler = handle_status,   .user_ctx = NULL };
    static const httpd_uri_t uri_train  = { .uri = "/api/training/status", .method = HTTP_GET, .handler = handle_training_status, .user_ctx = NULL };
    static const httpd_uri_t uri_cap_status = { .uri = "/api/capture/status", .method = HTTP_GET, .handler = handle_capture_status, .user_ctx = NULL };
    static const httpd_uri_t uri_cap_csv = { .uri = "/api/capture.csv", .method = HTTP_GET, .handler = handle_capture_csv, .user_ctx = NULL };
    static const httpd_uri_t uri_cmd    = { .uri = "/api/cmd",    .method = HTTP_POST, .handler = handle_cmd,      .user_ctx = NULL };
    httpd_register_uri_handler(server, (httpd_uri_t *)&uri_root);
    httpd_register_uri_handler(server, (httpd_uri_t *)&uri_leg);
    httpd_register_uri_handler(server, (httpd_uri_t *)&uri_status);
    httpd_register_uri_handler(server, (httpd_uri_t *)&uri_train);
    httpd_register_uri_handler(server, (httpd_uri_t *)&uri_cap_status);
    httpd_register_uri_handler(server, (httpd_uri_t *)&uri_cap_csv);
    httpd_register_uri_handler(server, (httpd_uri_t *)&uri_cmd);
    ESP_LOGI(TAG, "web UI at http://192.168.4.1");
}

void transport_http_init(void)
{
    s_boot_id = esp_random();
    if (s_boot_id == 0U) s_boot_id = 1U;
    s_resp_sem   = xSemaphoreCreateBinary();
    s_req_mutex  = xSemaphoreCreateMutex();
    wifi_ap_init();
    http_server_init();
}
