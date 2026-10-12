#include "pcm_ring.h"

void pcm_ring_init(pcm_ring_t *r, int16_t *storage, size_t cap)
{
    r->buf = storage;
    r->cap = cap;
    r->rd = 0;
    r->count = 0;
    portMUX_INITIALIZE(&r->mux);
}

void pcm_ring_write(pcm_ring_t *r, const int16_t *src, size_t n)
{
    portENTER_CRITICAL(&r->mux);
    for (size_t i = 0; i < n; i++) {
        size_t wr = (r->rd + r->count) % r->cap;
        r->buf[wr] = src ? src[i] : 0;
        if (r->count < r->cap) {
            r->count++;
        } else {
            r->rd = (r->rd + 1) % r->cap; /* overwrite oldest */
        }
    }
    portEXIT_CRITICAL(&r->mux);
}

size_t pcm_ring_read(pcm_ring_t *r, int16_t *dst, size_t n)
{
    portENTER_CRITICAL(&r->mux);
    size_t got = n < r->count ? n : r->count;
    for (size_t i = 0; i < got; i++) {
        dst[i] = r->buf[r->rd];
        r->rd = (r->rd + 1) % r->cap;
    }
    r->count -= got;
    portEXIT_CRITICAL(&r->mux);
    return got;
}

size_t pcm_ring_count(pcm_ring_t *r)
{
    portENTER_CRITICAL(&r->mux);
    size_t c = r->count;
    portEXIT_CRITICAL(&r->mux);
    return c;
}

void pcm_ring_clear(pcm_ring_t *r)
{
    portENTER_CRITICAL(&r->mux);
    r->rd = 0;
    r->count = 0;
    portEXIT_CRITICAL(&r->mux);
}

void pcm_ring_trim(pcm_ring_t *r, size_t keep)
{
    portENTER_CRITICAL(&r->mux);
    if (r->count > keep) {
        size_t drop = r->count - keep;
        r->rd = (r->rd + drop) % r->cap;
        r->count = keep;
    }
    portEXIT_CRITICAL(&r->mux);
}
