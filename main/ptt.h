#pragma once

#include <stdbool.h>

/* Toggle-type PTT with TX time-out (TOT). */
void ptt_init(void);
void ptt_set(bool on, const char *reason);
void ptt_toggle(const char *reason);
bool ptt_get(void);
