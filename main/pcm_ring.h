#pragma once

#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"

/* Single-producer / single-consumer ring of 16-bit samples.
 * When full, the oldest samples are overwritten (keeps latency bounded). */
typedef struct {
    int16_t     *buf;
    size_t       cap;
    size_t       rd;
    size_t       count;
    portMUX_TYPE mux;
} pcm_ring_t;

void   pcm_ring_init(pcm_ring_t *r, int16_t *storage, size_t cap);
void   pcm_ring_write(pcm_ring_t *r, const int16_t *src, size_t n);
size_t pcm_ring_read(pcm_ring_t *r, int16_t *dst, size_t n);
size_t pcm_ring_count(pcm_ring_t *r);
void   pcm_ring_clear(pcm_ring_t *r);
/* Drop the oldest samples so that at most `keep` remain. */
void   pcm_ring_trim(pcm_ring_t *r, size_t keep);
