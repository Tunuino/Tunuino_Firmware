/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_ANALOG_H
#define TUNUINO_ANALOG_H
#include <stdbool.h>
#include <stddef.h>
#include "tunuino/telemetry.h"
#define TUNUINO_ANALOG_MAX_POINTS 32u
typedef struct { uint16_t raw; int32_t value; } tunuino_analog_point;
/* Caller-owned immutable calibration: 2..32 strictly increasing raw points.
 * Values use the destination telemetry units; no default sensor curve exists. */
typedef struct {
    const tunuino_analog_point *points;
    size_t count;
    int32_t minimum, maximum;
} tunuino_analog_curve;
typedef enum {
    TUNUINO_ANALOG_OK = 0, TUNUINO_ANALOG_INVALID_SAMPLE,
    TUNUINO_ANALOG_OUT_OF_RANGE, TUNUINO_ANALOG_CONFIGURATION
} tunuino_analog_status;
typedef enum {
    TUNUINO_ANALOG_MAP = 0, TUNUINO_ANALOG_LAMBDA, TUNUINO_ANALOG_COOLANT
} tunuino_analog_channel;
/* Piecewise linear interpolation, nearest integer, ties away from zero.
 * No extrapolation or clamping; errors zero the output. Output must not
 * overlap curve storage. Caller supplies actual driver validity. */
tunuino_analog_status tunuino_analog_convert(const tunuino_analog_curve *curve,
    uint16_t raw, bool valid, int32_t *value);
/* Consumer only, serialized with telemetry_build and source updates.
 * Updates just one channel; on any error clears its validity and value.
 * Timestamp uses the capture clock domain, not receipt/USB time.
 * MAP/lambda must be nonnegative/positive respectively in their full curve.
 * Snapshot must not overlap curve storage. */
tunuino_analog_status tunuino_analog_update(tunuino_telemetry_sensors *sensors,
    tunuino_analog_channel channel, const tunuino_analog_curve *curve,
    uint16_t raw, bool valid, uint64_t time_us);
#endif
