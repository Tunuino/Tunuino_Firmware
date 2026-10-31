/* Copyright (c) 2026 Tunuino. */
#include "tunuino/clock32.h"
#include <stddef.h>

tunuino_clock32_status tunuino_clock32_init(tunuino_clock32 *clock,
    uint32_t first_counter, uint64_t first_us, uint32_t max_step_us) {
    if (clock == NULL) return TUNUINO_CLOCK32_ARGUMENT;
    *clock = (tunuino_clock32){0};
    if (max_step_us == 0 || max_step_us > INT32_MAX) return TUNUINO_CLOCK32_ARGUMENT;
    clock->previous = first_counter;
    clock->time_us = first_us;
    clock->origin_us = first_us;
    clock->max_step_us = max_step_us;
    clock->initialized = 1;
    return TUNUINO_CLOCK32_OK;
}

tunuino_clock32_status tunuino_clock32_capture(const tunuino_clock32 *clock,
    uint32_t captured_counter, uint32_t max_age_us, uint64_t *time_us) {
    if (time_us == NULL) return TUNUINO_CLOCK32_ARGUMENT;
    *time_us = 0;
    if (clock == NULL || !clock->initialized || max_age_us == 0 || max_age_us > INT32_MAX)
        return TUNUINO_CLOCK32_ARGUMENT;
    if (clock->fault) return TUNUINO_CLOCK32_FAULT;
    const uint32_t age = clock->previous - captured_counter;
    if (age > max_age_us || (uint64_t)age > clock->time_us - clock->origin_us)
        return TUNUINO_CLOCK32_REJECTED;
    *time_us = clock->time_us - age;
    return TUNUINO_CLOCK32_OK;
}

tunuino_clock32_status tunuino_clock32_observe(tunuino_clock32 *clock,
    uint32_t counter, uint64_t *time_us) {
    if (time_us == NULL) return TUNUINO_CLOCK32_ARGUMENT;
    *time_us = 0;
    if (clock == NULL || !clock->initialized) return TUNUINO_CLOCK32_ARGUMENT;
    if (clock->fault) return TUNUINO_CLOCK32_FAULT;
    const uint32_t delta = counter - clock->previous;
    if (delta > clock->max_step_us || clock->time_us > UINT64_MAX - delta) {
        clock->fault = 1;
        return TUNUINO_CLOCK32_FAULT;
    }
    clock->previous = counter;
    clock->time_us += delta;
    *time_us = clock->time_us;
    return TUNUINO_CLOCK32_OK;
}
