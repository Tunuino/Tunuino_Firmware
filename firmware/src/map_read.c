/* Copyright (c) 2026 Tunuino. */
#include "tunuino/map_read.h"
#include <string.h>

static void put(uint8_t *p, uint32_t value, unsigned n) {
    for (unsigned i = 0; i < n; ++i) { p[i] = (uint8_t)(value >> (8u * (n - i - 1u))); }
}
static uint32_t get(const uint8_t *p, unsigned n) {
    uint32_t value = 0;
    for (unsigned i = 0; i < n; ++i) { value = (value << 8) | p[i]; }
    return value;
}
tunuino_map_status tunuino_map_snapshot_prepare(tunuino_map_snapshot *s,
    const tunuino_map *map, uint32_t generation) {
    if (s == NULL) { return TUNUINO_MAP_ARGUMENT; }
    s->length = 0;
    s->generation = 0;
    if (map == NULL || generation == 0) { return TUNUINO_MAP_ARGUMENT; }
    size_t length;
    tunuino_map_status status = tunuino_map_encode(map, s->bytes, sizeof(s->bytes), &length);
    if (status != TUNUINO_MAP_OK) { return status; }
    s->length = length;
    s->kind = map->kind;
    s->crc = tunuino_crc32(s->bytes, length);
    s->generation = generation;
    return TUNUINO_MAP_OK;
}
tunuino_map_read_status tunuino_map_read_reply(const tunuino_map_snapshot *s,
    const uint8_t *request_packet, size_t request_length,
    uint8_t *response_packet, size_t capacity, size_t *written) {
    if (written != NULL) { *written = 0; }
    if (s == NULL || request_packet == NULL || response_packet == NULL || written == NULL) {
        return TUNUINO_MAP_READ_ARGUMENT;
    }
    const uint8_t *q;
    size_t length;
    if (tunuino_frame_decode(request_packet, request_length, &q, &length) != TUNUINO_FRAME_OK) {
        return TUNUINO_MAP_READ_BAD_FRAME;
    }
    if ((length != 11 && length != 19) || memcmp(q, "TUNUINO:M", 9) != 0
        || (q[10] != TUNUINO_MAP_VE && q[10] != TUNUINO_MAP_IGNITION)
        || !((q[9] == 0 && length == 11) || (q[9] == 1 && length == 19))) {
        return TUNUINO_MAP_READ_BAD_REQUEST;
    }
    uint32_t generation = 0, offset = 0, count = 0;
    if (q[9] == 1) {
        generation = get(q + 11, 4);
        offset = get(q + 15, 2);
        count = get(q + 17, 2);
        if (generation == 0 || count == 0 || count > TUNUINO_MAP_CHUNK_MAX) {
            return TUNUINO_MAP_READ_BAD_REQUEST;
        }
    }
    uint8_t body[12 + TUNUINO_MAP_CHUNK_MAX] = {0};
    size_t body_length = 1;
    if (s->generation == 0 || s->length < 48 || s->length > sizeof(s->bytes)
        || s->kind != (tunuino_map_kind)q[10]
        || tunuino_crc32(s->bytes, s->length) != s->crc) {
        body[0] = 1; /* NOT_READY */
    } else if (q[9] == 1 && generation != s->generation) {
        body[0] = 2; /* SNAPSHOT_CHANGED */
    } else {
        body[1] = 1;
        body[2] = q[9];
        body[3] = q[10];
        put(body + 4, s->generation, 4);
        if (q[9] == 0) {
            put(body + 8, (uint32_t)s->length, 2);
            put(body + 10, s->crc, 4);
            body_length = 14;
        } else {
            if (offset >= s->length || count > s->length - offset) {
                return TUNUINO_MAP_READ_BAD_REQUEST;
            }
            put(body + 8, offset, 2);
            put(body + 10, count, 2);
            memcpy(body + 12, s->bytes + offset, count);
            body_length = 12 + count;
        }
    }
    return tunuino_frame_encode(body, body_length, response_packet, capacity, written) == TUNUINO_FRAME_OK
        ? TUNUINO_MAP_READ_OK : TUNUINO_MAP_READ_CAPACITY;
}
