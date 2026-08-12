/* 装配层：bsp（板级）-> control（3 路推力器 + 指令协议）-> comm（UART/蓝牙传输）。
 * 启动流程：初始化硬件 -> 启动任务 -> 上电校准（可配置）-> 等待指令。
 */
#include "sdkconfig.h"

#include "bsp.h"
#include "calibration_capture.h"
#include "cmd_parser.h"
#include "stabilize.h"
#include "thruster.h"
#include "training_session.h"
#include "transport_bt.h"
#include "transport_http.h"
#include "transport_uart.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

void app_main(void)
{
    bsp_init();
    thruster_init();
    training_session_init();
    calibration_capture_init();
    cmd_parser_init();

    xTaskCreate(thruster_task, "thruster", 4096, NULL, 5, NULL);

    transport_uart_init();
    transport_bt_init();
    transport_http_init();
    stabilize_init();

#if CONFIG_ESC_CALIBRATE_ON_BOOT
    ESP_LOGI(TAG, "starting ESC calibration...");
    thruster_calibrate();
#else
    thruster_stop_all();
#endif

    ESP_LOGW(TAG, "ready. commands: HELP / START / STOP / MODE / TRAIN / CAPTURE / STATUS / CAL");
}
