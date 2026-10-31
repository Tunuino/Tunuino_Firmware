/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_POSITION_H
#define TUNUINO_POSITION_H
#include "tunuino/crank.h"
/* Experimental one selected cam edge per 720 degrees. Window refers to the
 * index of the last crank tooth, not an interpolated angle or TDC reference. */
typedef struct {
    tunuino_crank_config crank;
    uint16_t cam_window_first, cam_window_last;
    uint8_t cam_revolution_phase;
} tunuino_position_config;
typedef enum {
    TUNUINO_POSITION_ACQUIRING = 0, TUNUINO_POSITION_CRANK_ONLY,
    TUNUINO_POSITION_SYNCED, TUNUINO_POSITION_CAM_LOST,
    TUNUINO_POSITION_CRANK_LOST, TUNUINO_POSITION_ARGUMENT, TUNUINO_POSITION_CLOCK
} tunuino_position_status;
typedef struct {
    uint8_t crank_valid, cycle_valid, revolution_phase;
    uint16_t tooth_index;
    double rpm;
} tunuino_position_reading;
/* Last confirmed selected cam edge, in crank degrees from the first tooth
 * after the gap plus configured revolution parity. This is not a TDC offset. */
typedef struct {
    uint64_t time_us;
    uint32_t angle_centi;
    uint8_t valid;
} tunuino_cam_reading;
/* Caller-owned state. Initialize first and serialize all operations, including
 * capture/ISR producers. Arguments must not overlap this object. */
typedef struct {
    tunuino_crank crank;
    tunuino_position_config config;
    uint8_t configured, crank_valid, cam_candidate, cycle_valid, revolutions_since_cam;
    tunuino_position_status state;
    tunuino_cam_reading cam;
} tunuino_position;
tunuino_position_status tunuino_position_init(tunuino_position *position,
    const tunuino_position_config *config, uint64_t now_us);
tunuino_position_status tunuino_position_crank_edge(tunuino_position *position, uint64_t now_us);
tunuino_position_status tunuino_position_cam_edge(tunuino_position *position, uint64_t now_us);
tunuino_position_status tunuino_position_poll(tunuino_position *position, uint64_t now_us);
tunuino_position_status tunuino_position_read(tunuino_position *position, uint64_t now_us,
    tunuino_position_reading *reading);
tunuino_position_status tunuino_position_cam_read(tunuino_position *position, uint64_t now_us,
    tunuino_cam_reading *reading);
#endif
