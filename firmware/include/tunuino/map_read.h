/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_MAP_READ_H
#define TUNUINO_MAP_READ_H
#include "tunuino/calibration.h"
#define TUNUINO_MAP_CHUNK_MAX 256u
/* Caller owns storage and serializes prepare/reply. Keep immutable during reply.
 * Use a new nonzero generation whenever replacing a snapshot; do not reuse a
 * generation during a connection. This handler never changes the active map. */
typedef struct {
    uint8_t bytes[TUNUINO_MAP_FILE_MAX];
    size_t length;
    uint32_t generation, crc;
    tunuino_map_kind kind;
} tunuino_map_snapshot;
typedef enum {
    TUNUINO_MAP_READ_OK = 0, TUNUINO_MAP_READ_ARGUMENT,
    TUNUINO_MAP_READ_BAD_FRAME, TUNUINO_MAP_READ_BAD_REQUEST,
    TUNUINO_MAP_READ_CAPACITY
} tunuino_map_read_status;
/* Failure invalidates the snapshot. Map and snapshot storage must not overlap. */
tunuino_map_status tunuino_map_snapshot_prepare(tunuino_map_snapshot *snapshot,
    const tunuino_map *map, uint32_t generation);
/* Response storage must not overlap the snapshot. */
tunuino_map_read_status tunuino_map_read_reply(const tunuino_map_snapshot *snapshot,
    const uint8_t *request_packet, size_t request_length,
    uint8_t *response_packet, size_t capacity, size_t *written);
#endif
