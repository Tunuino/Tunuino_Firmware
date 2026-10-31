/* Copyright (c) 2026 Tunuino. Persisted start/warm-up corrections. */
#ifndef TUNUINO_FUEL_CORRECTIONS_H
#define TUNUINO_FUEL_CORRECTIONS_H
#include <stdbool.h>
#include <stdint.h>
#define TUNUINO_FUEL_CORRECTIONS_SIZE 25u
typedef struct {
    uint8_t enabled,fixed_cranking_advance;
    uint16_t cranking_rpm,cranking_percent,afterstart_percent,afterstart_ms;
    int16_t temperature[3],cranking_advance;
    uint16_t warmup_percent[3];
} tunuino_fuel_corrections;
typedef struct {uint32_t started_at;uint8_t running,afterstart_active;} tunuino_fuel_correction_state;
/* Settings page bytes 103..127. All-zero is the legacy disabled default;
 * disabled, valid calibration values may also be retained for later use. */
bool tunuino_fuel_corrections_decode(const uint8_t bytes[25],tunuino_fuel_corrections *out);
/* Percentages use 10000 = 100%; temperature is measured milli-Celsius.
 * During cranking use the cranking multiplier. Above the threshold, use WUE
 * times a linearly decaying ASE multiplier. Dead time is added afterwards. */
bool tunuino_fuel_corrections_apply(const tunuino_fuel_corrections *config,
    tunuino_fuel_correction_state *state,uint32_t now,uint32_t rpm_milli,
    bool coolant_valid,int32_t coolant_milli,uint32_t *percent,int16_t *advance);
#endif
