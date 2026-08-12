#include "line_reader.h"

#include <string.h>

void line_reader_reset(line_reader_t *lr)
{
    lr->len      = 0;
    lr->overflow = false;
}

bool line_reader_feed_byte(line_reader_t *lr, uint8_t c, char *out, size_t out_sz)
{
    if (c == '\r' || c == '\n') {
        if (lr->len > 0 && !lr->overflow) {
            size_t n = (lr->len < out_sz - 1) ? lr->len : out_sz - 1;
            memcpy(out, lr->buf, n);
            out[n] = '\0';
            lr->len      = 0;
            lr->overflow = false;
            return true;
        }
        /* 空行或超长行：丢弃 */
        lr->len      = 0;
        lr->overflow = false;
        return false;
    }

    if (lr->len < sizeof(lr->buf)) {
        lr->buf[lr->len++] = c;
    } else {
        lr->overflow = true;
    }
    return false;
}
