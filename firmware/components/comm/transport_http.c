/* WiFi 热点 + HTTP 网页控制。复用 cmd_parser 协议：网页指令经 /api/cmd
 * 走与 UART/BLE 完全相同的解析与 XX_OK 回传。
 */
#include "transport_http.h"

#include "cmd_parser.h"
#include "stabilize.h"
#include "thruster.h"

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

#define HTTP_RESP_MAX 256

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
        "\"stab\":%d,\"mde\":%d,\"es\":%d,\"po\":%d,\"ro\":%d,\"stop\":%d,"
        "\"m\":[{\"s\":%d,\"c\":%lu,\"t\":%lu},"
        "{\"s\":%d,\"c\":%lu,\"t\":%lu},"
        "{\"s\":%d,\"c\":%lu,\"t\":%lu}]}",
        (int)thruster_is_calibrating(),
        st->rollDeg, st->pitchDeg, st->yawDeg, st->gyroDps,
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
    httpd_resp_send(req, buf, len);
    return ESP_OK;
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

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd start failed");
        return;
    }
    static const httpd_uri_t uri_root   = { .uri = "/",           .method = HTTP_GET,  .handler = handle_root,     .user_ctx = NULL };
    static const httpd_uri_t uri_leg    = { .uri = "/leg.js",     .method = HTTP_GET,  .handler = handle_leg_model,.user_ctx = NULL };
    static const httpd_uri_t uri_status = { .uri = "/api/status", .method = HTTP_GET,  .handler = handle_status,   .user_ctx = NULL };
    static const httpd_uri_t uri_cmd    = { .uri = "/api/cmd",    .method = HTTP_POST, .handler = handle_cmd,      .user_ctx = NULL };
    httpd_register_uri_handler(server, (httpd_uri_t *)&uri_root);
    httpd_register_uri_handler(server, (httpd_uri_t *)&uri_leg);
    httpd_register_uri_handler(server, (httpd_uri_t *)&uri_status);
    httpd_register_uri_handler(server, (httpd_uri_t *)&uri_cmd);
    ESP_LOGI(TAG, "web UI at http://192.168.4.1");
}

void transport_http_init(void)
{
    s_resp_sem   = xSemaphoreCreateBinary();
    s_req_mutex  = xSemaphoreCreateMutex();
    wifi_ap_init();
    http_server_init();
}
