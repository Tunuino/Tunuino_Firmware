/* Copyright (c) 2026 Tunuino. */
#include "tunuino/crank.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static tunuino_crank_status lose(tunuino_crank *c, tunuino_crank_status reason) {
    c->period_us = c->rpm = 0;
    c->candidate = c->synchronized = c->regular_intervals = 0;
    c->tooth_index = 0;
    c->state = reason;
    return reason;
}
static int nominal(const tunuino_crank *c, double period) {
    return period >= c->minimum_period_us && period <= c->maximum_period_us;
}
static tunuino_crank_status clock_check(tunuino_crank *c, uint64_t now) {
    if (c == NULL || !c->configured) { return TUNUINO_CRANK_ARGUMENT; }
    if (c->state == TUNUINO_CRANK_CLOCK) { return c->state; }
    if (now < c->last_now_us) { return lose(c,TUNUINO_CRANK_CLOCK); }
    c->last_now_us = now;
    return c->state;
}
tunuino_crank_status tunuino_crank_init(tunuino_crank *c, const tunuino_crank_config *config, uint64_t now) {
    if (c == NULL) { return TUNUINO_CRANK_ARGUMENT; }
    memset(c,0,sizeof(*c)); c->state = TUNUINO_CRANK_ARGUMENT;
    if (config == NULL || config->slots < 12 || config->slots > 360 || config->missing < 1 || config->missing > 4
        || !isfinite(config->minimum_rpm) || !isfinite(config->maximum_rpm)
        || config->minimum_rpm < 1 || config->maximum_rpm <= config->minimum_rpm || config->maximum_rpm > 30000
        || config->timeout_us == 0 || config->timeout_us > 60000000u) { return c->state; }
    double low = 60000000.0 / (config->slots * config->maximum_rpm);
    double high = 60000000.0 / (config->slots * config->minimum_rpm);
    if (low < 10 || config->timeout_us < high * (config->missing + 1u) * 1.2) { return c->state; }
    c->config = *config; c->minimum_period_us = low; c->maximum_period_us = high;
    c->last_now_us = now; c->configured = 1; c->state = TUNUINO_CRANK_ACQUIRING;
    return c->state;
}
tunuino_crank_status tunuino_crank_poll(tunuino_crank *c, uint64_t now) {
    tunuino_crank_status status = clock_check(c,now);
    if (status == TUNUINO_CRANK_ARGUMENT || status == TUNUINO_CRANK_CLOCK) { return status; }
    if (c->have_edge && now - c->last_edge_us >= c->config.timeout_us) {
        c->have_edge = 0;
        return lose(c,TUNUINO_CRANK_LOST);
    }
    return c->state;
}
tunuino_crank_status tunuino_crank_edge(tunuino_crank *c, uint64_t now) {
    tunuino_crank_status status = clock_check(c,now);
    if (status == TUNUINO_CRANK_ARGUMENT || status == TUNUINO_CRANK_CLOCK) { return status; }
    if (!c->have_edge) {
        c->have_edge = 1; c->last_edge_us = now;
        c->state = TUNUINO_CRANK_ACQUIRING;
        return c->state;
    }
    uint64_t elapsed = now - c->last_edge_us;
    c->last_edge_us = now;
    if (elapsed == 0 || elapsed >= c->config.timeout_us) { return lose(c,TUNUINO_CRANK_LOST); }
    double period = (double)elapsed;
    if (c->period_us == 0) {
        if (!nominal(c,period)) { return lose(c,TUNUINO_CRANK_LOST); }
        c->period_us = period; c->regular_intervals = 1;
        c->state = TUNUINO_CRANK_ACQUIRING;
        return c->state;
    }
    double ratio = period / c->period_us;
    if (ratio >= 0.8 && ratio <= 1.2 && nominal(c,period)) {
        if (c->candidate) {
            ++c->tooth_index;
            if (c->tooth_index >= c->config.slots - c->config.missing) {
                return lose(c,TUNUINO_CRANK_LOST);
            }
        }
        c->period_us = c->period_us * 0.75 + period * 0.25;
        if (c->regular_intervals < 4) { ++c->regular_intervals; }
        c->rpm = c->synchronized ? 60000000.0 / (c->config.slots * c->period_us) : 0;
        c->state = c->synchronized ? TUNUINO_CRANK_SYNCED : TUNUINO_CRANK_ACQUIRING;
        return c->state;
    }
    double normalized = period / (c->config.missing + 1u);
    double gap_error = fabs(ratio - (c->config.missing + 1u));
    if (c->regular_intervals >= 4 && gap_error <= 0.25 && nominal(c,normalized)) {
        int was_synced = c->synchronized;
        c->synchronized = c->candidate && c->tooth_index == c->config.slots - c->config.missing - 1u;
        c->candidate = 1; c->tooth_index = 0; c->regular_intervals = 0; c->period_us = normalized;
        c->rpm = c->synchronized ? 60000000.0 / (c->config.slots * normalized) : 0;
        c->state = c->synchronized ? TUNUINO_CRANK_SYNCED : was_synced ? TUNUINO_CRANK_LOST : TUNUINO_CRANK_ACQUIRING;
        return c->state;
    }
    return lose(c,TUNUINO_CRANK_LOST);
}
tunuino_crank_status tunuino_crank_read(tunuino_crank *c, uint64_t now, tunuino_crank_reading *reading) {
    if (reading == NULL) { return TUNUINO_CRANK_ARGUMENT; }
    memset(reading,0,sizeof(*reading));
    tunuino_crank_status status = tunuino_crank_poll(c,now);
    if (status == TUNUINO_CRANK_SYNCED) {
        reading->valid = 1; reading->rpm = c->rpm; reading->tooth_index = c->tooth_index;
    }
    return status;
}
