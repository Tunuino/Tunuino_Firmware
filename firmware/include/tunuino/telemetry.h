/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_TELEMETRY_H
#define TUNUINO_TELEMETRY_H
#include "tunuino/capture_processor.h"
#include "tunuino/live_data.h"
#define TUNUINO_TELEMETRY_SENSOR_MASK (TUNUINO_LIVE_MAP_VALID | TUNUINO_LIVE_LAMBDA_VALID | TUNUINO_LIVE_COOLANT_VALID)
/* Board-supplied, already calibrated/plausibility-checked sensor snapshots.
 * The mask can only declare MAP/lambda/coolant; RPM comes from captures. */
typedef struct {
    uint64_t map_time_us, lambda_time_us, coolant_time_us;
    uint32_t map_milli_kpa, lambda_million;
    int32_t coolant_milli_c;
    uint8_t valid_mask;
} tunuino_telemetry_sensors;
typedef struct { uint32_t map_max_age_us, lambda_max_age_us, coolant_max_age_us; } tunuino_telemetry_policy;
typedef enum { TUNUINO_TELEMETRY_READY = 0, TUNUINO_TELEMETRY_NOT_READY, TUNUINO_TELEMETRY_ARGUMENT } tunuino_telemetry_status;
/* Consumer only: serialize with processor drain/read and provider updates.
 * Ages 1..60000000 us; all timestamps use the capture clock domain. Invalid
 * fields are zeroed and their bits cleared; partial samples yield NOT_READY.
 * Output/arguments must not overlap processor or provider storage. */
tunuino_telemetry_status tunuino_telemetry_build(tunuino_capture_processor *processor,
    const tunuino_telemetry_sensors *sensors, const tunuino_telemetry_policy *policy,
    uint64_t now_us, tunuino_live_sample *sample);
#endif
