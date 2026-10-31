/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_CRANK_H
#define TUNUINO_CRANK_H
#include <stdint.h>
/* Experimental uniform missing-tooth wheel, one contiguous gap, one edge per
 * physical tooth. No TDC offset, cam phase, direction or actuator scheduling. */
typedef struct {
    uint16_t slots;
    uint8_t missing;
    double minimum_rpm, maximum_rpm;
    uint32_t timeout_us;
} tunuino_crank_config;
typedef enum {
    TUNUINO_CRANK_ACQUIRING = 0, TUNUINO_CRANK_SYNCED, TUNUINO_CRANK_LOST,
    TUNUINO_CRANK_ARGUMENT, TUNUINO_CRANK_CLOCK
} tunuino_crank_status;
typedef struct {
    uint8_t valid;
    uint16_t tooth_index; /* First physical tooth after the gap is zero. */
    double rpm;
} tunuino_crank_reading;
/* Caller-owned state, initialized before use; fields private. Serialize edge,
 * polling and reading with ISR producers. Do not overlap arguments/state. */
typedef struct {
    tunuino_crank_config config;
    uint64_t last_edge_us, last_now_us;
    double period_us, minimum_period_us, maximum_period_us, rpm;
    uint16_t tooth_index;
    uint8_t have_edge, candidate, synchronized, regular_intervals, configured;
    tunuino_crank_status state;
} tunuino_crank;
tunuino_crank_status tunuino_crank_init(tunuino_crank *crank, const tunuino_crank_config *config, uint64_t now_us);
/* Extended monotonic microsecond timestamps; clock rollback requires init. */
tunuino_crank_status tunuino_crank_edge(tunuino_crank *crank, uint64_t now_us);
tunuino_crank_status tunuino_crank_poll(tunuino_crank *crank, uint64_t now_us);
/* Checks timeout before exposing RPM/phase; invalid readings are zeroed. */
tunuino_crank_status tunuino_crank_read(tunuino_crank *crank, uint64_t now_us, tunuino_crank_reading *reading);
#endif
