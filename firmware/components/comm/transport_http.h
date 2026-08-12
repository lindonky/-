#pragma once

/* 初始化 WiFi 热点 + HTTP 控制服务：
 *   GET  /            控制网页（滑块/按钮）
 *   GET  /api/status  三路状态 JSON
 *   POST /api/cmd     指令（正文为协议文本），回传 XX_OK/XX_ERR
 */
void transport_http_init(void);
