#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LINE_READER_MAX 128

/* 字节流 -> 完整行 的组包器（\r 或 \n 结尾；超长整行丢弃）。 */
typedef struct {
    uint8_t buf[LINE_READER_MAX];
    size_t  len;
    bool    overflow;
} line_reader_t;

void line_reader_reset(line_reader_t *lr);

/* 喂入 1 字节；凑齐一行（含行尾）时返回 true 并把行写入 out（不含行尾）。 */
bool line_reader_feed_byte(line_reader_t *lr, uint8_t c, char *out, size_t out_sz);
