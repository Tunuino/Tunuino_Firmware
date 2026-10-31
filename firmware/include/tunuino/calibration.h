/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_CALIBRATION_H
#define TUNUINO_CALIBRATION_H
#include "tunuino/frame.h"

#define TUNUINO_MAP_AXIS_MAX 32u
#define TUNUINO_MAP_FILE_MAX 4368u
typedef enum { TUNUINO_MAP_VE = 1, TUNUINO_MAP_IGNITION = 2 } tunuino_map_kind;
typedef struct {
    tunuino_map_kind kind;
    uint8_t rpm_count, load_count;
    int32_t rpm[TUNUINO_MAP_AXIS_MAX];
    int32_t load_milli_kpa[TUNUINO_MAP_AXIS_MAX];
    /* VE: milli-percent; ignition: milli-degrees BTDC. Rows are load, columns RPM. */
    int32_t cells[TUNUINO_MAP_AXIS_MAX][TUNUINO_MAP_AXIS_MAX];
} tunuino_map;
typedef enum {
    TUNUINO_MAP_OK = 0, TUNUINO_MAP_CLAMPED, TUNUINO_MAP_ARGUMENT,
    TUNUINO_MAP_FORMAT, TUNUINO_MAP_RANGE, TUNUINO_MAP_CRC, TUNUINO_MAP_CAPACITY
} tunuino_map_status;

tunuino_map_status tunuino_map_validate(const tunuino_map *map);
tunuino_map_status tunuino_map_encode(const tunuino_map *map, uint8_t *bytes, size_t capacity, size_t *written);
/* Input and output storage must not overlap. Invalid input leaves output unchanged. */
tunuino_map_status tunuino_map_decode(const uint8_t *bytes, size_t length, tunuino_map *map);
/* Output is percent VE or degrees BTDC. CLAMPED explicitly reports out-of-axis input. */
tunuino_map_status tunuino_map_lookup(const tunuino_map *map, double rpm, double map_kpa, double *value);
/* Offline/refinement utility: 8x8 map -> 16x16 map. Preserves all eight
 * original axis values and grid cells exactly; new knots use interpolation.
 * Does not change ECU page sizes or persistent layout. No in-place operation.
 * Invalid inputs leave the caller-owned destination untouched. */
tunuino_map_status tunuino_map_refine_16(const tunuino_map *source, tunuino_map *destination);
#endif
