/* Copyright (c) 2026 Tunuino. */
#include "tunuino/telemetry.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static int valid_age(uint32_t age) { return age > 0 && age <= 60000000u; }
static int fresh(uint64_t stamp, uint64_t now, uint32_t age) { return stamp <= now && now - stamp < age; }
tunuino_telemetry_status tunuino_telemetry_build(tunuino_capture_processor *processor,
    const tunuino_telemetry_sensors *s, const tunuino_telemetry_policy *policy,
    uint64_t now, tunuino_live_sample *out) {
    if (out != NULL) { memset(out,0,sizeof(*out)); }
    if (processor == NULL || s == NULL || policy == NULL || out == NULL
        || !valid_age(policy->map_max_age_us) || !valid_age(policy->lambda_max_age_us)
        || !valid_age(policy->coolant_max_age_us) || (s->valid_mask & ~TUNUINO_TELEMETRY_SENSOR_MASK) != 0) {
        return TUNUINO_TELEMETRY_ARGUMENT;
    }
    tunuino_position_reading position;
    tunuino_capture_process_status status = tunuino_capture_processor_read(processor,now,&position);
    if (status == TUNUINO_PROCESS_ARGUMENT) { return TUNUINO_TELEMETRY_ARGUMENT; }
    out->time_ms = now / 1000u;
    if (status == TUNUINO_PROCESS_READY && position.crank_valid) {
        double scaled = round(position.rpm * 1000.0);
        if (isfinite(scaled) && scaled > 0 && scaled <= UINT32_MAX) {
            out->rpm_milli = (uint32_t)scaled; out->valid_mask |= TUNUINO_LIVE_RPM_VALID;
        }
    }
    if ((s->valid_mask & TUNUINO_LIVE_MAP_VALID) && fresh(s->map_time_us,now,policy->map_max_age_us)) {
        out->map_milli_kpa = s->map_milli_kpa; out->valid_mask |= TUNUINO_LIVE_MAP_VALID;
    }
    if ((s->valid_mask & TUNUINO_LIVE_LAMBDA_VALID) && s->lambda_million > 0
        && fresh(s->lambda_time_us,now,policy->lambda_max_age_us)) {
        out->lambda_million = s->lambda_million; out->valid_mask |= TUNUINO_LIVE_LAMBDA_VALID;
    }
    if ((s->valid_mask & TUNUINO_LIVE_COOLANT_VALID) && fresh(s->coolant_time_us,now,policy->coolant_max_age_us)) {
        out->coolant_milli_c = s->coolant_milli_c; out->valid_mask |= TUNUINO_LIVE_COOLANT_VALID;
    }
    return out->valid_mask == TUNUINO_LIVE_ALL_VALID ? TUNUINO_TELEMETRY_READY : TUNUINO_TELEMETRY_NOT_READY;
}
