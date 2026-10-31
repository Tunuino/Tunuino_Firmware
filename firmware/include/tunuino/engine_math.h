/* Copyright (c) 2026 Tunuino. Portable dry-run engine timing model. */
#ifndef TUNUINO_ENGINE_MATH_H
#define TUNUINO_ENGINE_MATH_H
#include <stdint.h>

/* No cylinder phase, TDC, coil dwell, injector flow, fuel mass or GPIO here.
 * This model computes relative timing and a dimensionless air-load proxy;
 * it does NOT authorize or schedule actual engine outputs. */
typedef struct {
    uint32_t revolution_us;   /* 360 crank degrees, rounded to 1 us. */
    uint32_t cycle_us;        /* 720 crank degrees (four-stroke only). */
    uint32_t wheel_slot_us;   /* Nominal angular slot, including missing teeth. */
    uint32_t load_centi_percent; /* VE% * MAP(kPa)/100kPa, scaled by 100. */
    int32_t spark_relative_us; /* Negative: before hypothetical TDC; positive: after. */
} tunuino_engine_math_result;

typedef enum {
    TUNUINO_ENGINE_MATH_OK=0,
    TUNUINO_ENGINE_MATH_ARGUMENT,
    TUNUINO_ENGINE_MATH_RANGE
} tunuino_engine_math_status;

/* Inputs: RPM*1000, crank wheel slots and missing count, VE%*100,
 * ignition advance*100 degrees BTDC, MAP*kPa*1000.
 * Invalid input leaves the output unchanged. No physical timing reference. */
tunuino_engine_math_status tunuino_engine_math_calculate(
    uint32_t rpm_milli, uint16_t wheel_slots, uint8_t missing,
    uint16_t ve_centi_percent, int16_t advance_centi_deg,
    uint32_t map_milli_kpa, tunuino_engine_math_result *result);
#endif
