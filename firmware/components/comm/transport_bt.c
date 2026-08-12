/* BLE UART 传输：ESP32-S3 只支持 BLE（无经典蓝牙），实现标准
 * Nordic UART Service（NUS）：
 *   Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX 特性（手机->设备，写）: 6E400002-...
 *   TX 特性（设备->手机，notify）: 6E400003-...
 *
 * 设计要点：
 *  - RX：ESP_GATTS_WRITE_EVT 回调里只做轻量组行，行交给 cmd_parser 命令队列
 *        （cmd 任务上下文执行，绝不在 GATT 回调里做阻塞操作）；
 *  - TX：协议回传只入队，由独立 TX 任务调用 esp_ble_gatts_send_indicate(notify)，
 *        防止阻塞蓝牙栈与丢包。
 */
#include "transport_bt.h"

#include "cmd_parser.h"
#include "line_reader.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <string.h>

#define BLE_TX_ITEM_SIZE 192

static const char *TAG = "ble_uart";

/* NUS UUID（小端字节序，标准 Nordic 定义） */
static const uint8_t nus_svc_uuid128[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                                          0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};
static const uint8_t nus_rx_uuid128[]  = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                                          0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E};
static const uint8_t nus_tx_uuid128[]  = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                                          0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E};

/* GATT 属性表：服务声明 + RX 写特性 + TX notify 特性 + CCCD */
#define NUS_IDX_SVC      0
#define NUS_IDX_RX_CHAR  1
#define NUS_IDX_RX_VAL   2
#define NUS_IDX_TX_CHAR  3
#define NUS_IDX_TX_VAL   4
#define NUS_IDX_TX_CCCD  5
#define NUS_IDX_NB       6

static uint16_t nus_handle_table[NUS_IDX_NB];

static const uint16_t nus_primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t nus_char_decl_uuid       = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t nus_cccd_uuid            = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t  nus_char_prop_rx         = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t  nus_char_prop_tx         = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t  nus_initial_val[]        = "ok";

static const esp_gatts_attr_db_t nus_gatt_db[NUS_IDX_NB] = {
    [NUS_IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&nus_primary_service_uuid,
         ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&nus_svc_uuid128},
    },
    [NUS_IDX_RX_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&nus_char_decl_uuid,
         ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&nus_char_prop_rx},
    },
    [NUS_IDX_RX_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *)nus_rx_uuid128,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(nus_initial_val), sizeof(nus_initial_val), (uint8_t *)nus_initial_val},
    },
    [NUS_IDX_TX_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&nus_char_decl_uuid,
         ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&nus_char_prop_tx},
    },
    [NUS_IDX_TX_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *)nus_tx_uuid128,
         ESP_GATT_PERM_READ, sizeof(nus_initial_val), sizeof(nus_initial_val), (uint8_t *)nus_initial_val},
    },
    [NUS_IDX_TX_CCCD] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&nus_cccd_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)nus_initial_val},
    },
};

static esp_gatt_if_t     s_gatts_if      = ESP_GATT_IF_NONE;
static uint16_t          s_conn_id       = 0;
static volatile bool     s_connected     = false;
static volatile bool     s_notify_enabled = false;
static QueueHandle_t     s_tx_q;
static line_reader_t     s_bt_lr;   /* 只从 GATT 回调访问（蓝牙栈任务），无并发 */

static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min       = 0x20,
    .adv_int_max       = 0x40,
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* 广播内容：设备名 + NUS 服务 UUID（128bit） */
static esp_ble_adv_data_t s_adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x20,
    .max_interval        = 0x40,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = ESP_UUID_LEN_128,
    .p_service_uuid      = (uint8_t *)nus_svc_uuid128,
    .flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

typedef struct {
    char data[BLE_TX_ITEM_SIZE];
} ble_tx_msg_t;

/* --- 回传（协议层 -> BLE）：仅入队，非阻塞 --- */
static void ble_send(void *ctx, const char *data, size_t len)
{
    (void)ctx;
    if (s_tx_q == NULL || !s_connected || !s_notify_enabled) return;

    ble_tx_msg_t msg;
    if (len >= sizeof(msg.data)) {
        len = sizeof(msg.data) - 1;
    }
    memcpy(msg.data, data, len);
    msg.data[len] = '\0';

    /* 队列满则丢弃（防拥塞，不阻塞蓝牙栈） */
    if (xQueueSend(s_tx_q, &msg, 0) != pdTRUE) {
        ESP_LOGW(TAG, "ble tx queue full, dropped");
    }
}

static const cmd_transport_t s_transport = {
    .name = "BLE",
    .send = ble_send,
    .ctx  = NULL,
};

/* --- TX 任务：逐包 notify --- */
static void ble_tx_task(void *arg)
{
    (void)arg;
    ble_tx_msg_t msg;
    while (true) {
        if (xQueueReceive(s_tx_q, &msg, portMAX_DELAY) == pdTRUE) {
            if (s_connected && s_notify_enabled && s_gatts_if != ESP_GATT_IF_NONE) {
                esp_ble_gatts_send_indicate(s_gatts_if, s_conn_id,
                                            nus_handle_table[NUS_IDX_TX_VAL],
                                            (int)strlen(msg.data),
                                            (uint8_t *)msg.data, false);
            }
        }
    }
}

/* --- RX：GATT 写回调中组行 -> cmd_parser --- */
static void feed_ble_data(const uint8_t *data, size_t len)
{
    char line[CMD_LINE_MAX];
    for (size_t i = 0; i < len; i++) {
        if (line_reader_feed_byte(&s_bt_lr, data[i], line, sizeof(line))) {
            cmd_parser_feed_line(&s_transport, line);
        }
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        if (param->reg.status == ESP_GATT_OK) {
            s_gatts_if = gatts_if;
            esp_ble_gatts_create_attr_tab(nus_gatt_db, gatts_if, NUS_IDX_NB, NUS_IDX_SVC);
        } else {
            ESP_LOGE(TAG, "GATTS register failed, status=%d", param->reg.status);
        }
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK) {
            memcpy(nus_handle_table, param->add_attr_tab.handles, NUS_IDX_NB * sizeof(uint16_t));
            esp_ble_gatts_start_service(nus_handle_table[NUS_IDX_SVC]);
        } else {
            ESP_LOGE(TAG, "create attr tab failed, status=%d", param->add_attr_tab.status);
        }
        break;
    case ESP_GATTS_CONNECT_EVT:
        s_conn_id    = param->connect.conn_id;
        s_connected  = true;
        ESP_LOGI(TAG, "BLE connected, conn_id=%d", s_conn_id);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        s_connected      = false;
        s_notify_enabled = false;
        ESP_LOGI(TAG, "BLE disconnected, restart advertising");
        esp_ble_gap_start_advertising(&s_adv_params);
        break;
    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == nus_handle_table[NUS_IDX_RX_VAL]) {
            feed_ble_data(param->write.value, param->write.len);
        } else if (param->write.handle == nus_handle_table[NUS_IDX_TX_CCCD]) {
            /* 手机订阅/取消 notify */
            s_notify_enabled = (param->write.len >= 2) && (param->write.value[0] & 0x01);
        }
        break;
    default:
        break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&s_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "advertising started, name='%s'", CONFIG_ESC_BT_NAME);
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "advertising stopped");
        break;
    default:
        break;
    }
}

void transport_bt_init(void)
{
    /* BT 栈需要 NVS 存储配置 */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_err = nvs_flash_init();
    }
    if (nvs_err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(nvs_err));
    }

    s_tx_q = xQueueCreate(8, sizeof(ble_tx_msg_t));
    xTaskCreate(ble_tx_task, "ble_tx", 4096, NULL, 4, NULL);
    line_reader_reset(&s_bt_lr);

    /* 只用 BLE：释放经典蓝牙控制器内存 */
    esp_err_t err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mem_release(CLASSIC): %s", esp_err_to_name(err));
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    /* 顺序很重要：先注册 GAP/GATTS 回调，再配置广播（否则完成事件可能丢失） */
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(CONFIG_ESC_BT_NAME));
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&s_adv_data)); /* 配置后触发 ADV_DATA_SET_COMPLETE -> 开始广播 */
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

    ESP_LOGI(TAG, "BLE UART init started (device name '%s')", CONFIG_ESC_BT_NAME);
}
