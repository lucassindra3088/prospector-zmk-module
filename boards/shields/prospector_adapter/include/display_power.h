/*
 * Prospector display power state helper
 */
#pragma once

#include <stdbool.h>

void prospector_display_set_sleeping(bool sleeping);
bool prospector_display_is_sleeping(void);
void prospector_brightness_fade_off(void);
void prospector_brightness_fade_on(uint8_t target);
uint8_t prospector_brightness_get_current(void);
