#pragma once

/* 通信传输层契约（可插拔）。
 *
 * 每个传输（UART / 蓝牙 SPP）实现同一个模式：
 *   1. 收到字节 -> line_reader 组行 -> cmd_parser_feed_line(&本传输的 cmd_transport_t, 行)
 *   2. 协议层回传通过 cmd_transport_t.send() 非阻塞入队发回本传输
 * 因此指令从哪个通道来，XX_OK/XX_ERR 就回哪个通道。
 *
 * 各传输入口：
 *   transport_uart_init()  —— 板载 CH340 串口（UART0）
 *   transport_bt_init()    —— 经典蓝牙 SPP（Android 蓝牙串口）
 */
