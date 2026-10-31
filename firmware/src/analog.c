/* Copyright (c) 2026 Tunuino. */
#include "tunuino/analog.h"

static bool curve_valid(const tunuino_analog_curve *curve) {
    if (!curve || !curve->points || curve->count < 2u ||
        curve->count > TUNUINO_ANALOG_MAX_POINTS || curve->minimum > curve->maximum) return false;
    for (size_t i = 0; i < curve->count; ++i) {
        if (curve->points[i].value < curve->minimum || curve->points[i].value > curve->maximum ||
            (i && curve->points[i].raw <= curve->points[i - 1u].raw)) return false;
    }
    return true;
}

tunuino_analog_status tunuino_analog_convert(const tunuino_analog_curve *curve,
    uint16_t raw, bool valid, int32_t *value) {
    if (!value) return TUNUINO_ANALOG_CONFIGURATION;
    *value = 0;
    if (!curve_valid(curve)) return TUNUINO_ANALOG_CONFIGURATION;
    if (!valid) return TUNUINO_ANALOG_INVALID_SAMPLE;
    if (raw < curve->points[0].raw || raw > curve->points[curve->count - 1u].raw)
        return TUNUINO_ANALOG_OUT_OF_RANGE;
    for (size_t i = 1; i < curve->count; ++i) {
        const tunuino_analog_point a = curve->points[i - 1u], b = curve->points[i];
        if (raw <= b.raw) {
            const int64_t span = (int64_t)b.raw - a.raw;
            /* All products fit int64_t: signed 32-bit values, 16-bit raw. */
            const int64_t numerator = (int64_t)a.value * span +
                ((int64_t)b.value - a.value) * ((int64_t)raw - a.raw);
            const int64_t rounded = numerator >= 0 ? numerator + span / 2 : numerator - span / 2;
            *value = (int32_t)(rounded / span);
            return TUNUINO_ANALOG_OK;
        }
    }
    return TUNUINO_ANALOG_CONFIGURATION;
}

tunuino_analog_status tunuino_analog_update(tunuino_telemetry_sensors *sensors,
    tunuino_analog_channel channel, const tunuino_analog_curve *curve,
    uint16_t raw, bool valid, uint64_t time_us) {
    if (!sensors) return TUNUINO_ANALOG_CONFIGURATION;
    uint8_t bit;
    switch (channel) {
        case TUNUINO_ANALOG_MAP: bit = TUNUINO_LIVE_MAP_VALID; break;
        case TUNUINO_ANALOG_LAMBDA: bit = TUNUINO_LIVE_LAMBDA_VALID; break;
        case TUNUINO_ANALOG_COOLANT: bit = TUNUINO_LIVE_COOLANT_VALID; break;
        default: return TUNUINO_ANALOG_CONFIGURATION;
    }
    sensors->valid_mask &= (uint8_t)~bit;
    int32_t value = 0;
    tunuino_analog_status status = tunuino_analog_convert(curve, raw, valid, &value);
    if (status == TUNUINO_ANALOG_OK &&
        ((channel == TUNUINO_ANALOG_MAP && curve->minimum < 0) ||
         (channel == TUNUINO_ANALOG_LAMBDA && curve->minimum <= 0))) {
        status = TUNUINO_ANALOG_CONFIGURATION;
    }
    if (status != TUNUINO_ANALOG_OK) value = 0;
    switch (channel) {
        case TUNUINO_ANALOG_MAP:
            sensors->map_milli_kpa = (uint32_t)value; sensors->map_time_us = time_us; break;
        case TUNUINO_ANALOG_LAMBDA:
            sensors->lambda_million = (uint32_t)value; sensors->lambda_time_us = time_us; break;
        case TUNUINO_ANALOG_COOLANT:
            sensors->coolant_milli_c = value; sensors->coolant_time_us = time_us; break;
        default: return TUNUINO_ANALOG_CONFIGURATION;
    }
    if (status == TUNUINO_ANALOG_OK) sensors->valid_mask |= bit;
    return status;
}
