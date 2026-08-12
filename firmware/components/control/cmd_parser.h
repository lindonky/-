#pragma once

#include <stddef.h>

#define CMD_LINE_MAX 128

/* 传输句柄：指令从哪个传输来，回传（XX_OK/XX_ERR）就去哪个传输。 */
typedef struct {
    const char *name;                                   /* "UART" / "BT" */
    void (*send)(void *ctx, const char *data, size_t len); /* 非阻塞发送一行数据 */
    void *ctx;
} cmd_transport_t;

/* 创建命令处理任务与队列。 */
void cmd_parser_init(void);

/* 把一条完整行交给协议层处理（可在传输回调中调用，非阻塞）。 */
void cmd_parser_feed_line(const cmd_transport_t *t, const char *line);
