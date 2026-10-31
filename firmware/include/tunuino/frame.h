/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_FRAME_H
#define TUNUINO_FRAME_H

#include <stddef.h>
#include <stdint.h>

#define TUNUINO_MAX_BODY 4096u
#define TUNUINO_FRAME_OVERHEAD 6u

typedef enum {
    TUNUINO_FRAME_OK = 0,
    TUNUINO_FRAME_ARGUMENT,
    TUNUINO_FRAME_LENGTH,
    TUNUINO_FRAME_CAPACITY,
    TUNUINO_FRAME_CRC
} tunuino_frame_status;

uint32_t tunuino_crc32(const uint8_t *bytes, size_t length);
tunuino_frame_status tunuino_frame_encode(const uint8_t *body, size_t length,
    uint8_t *packet, size_t capacity, size_t *written);
/* On success, body points inside packet and is valid for packet's lifetime. */
tunuino_frame_status tunuino_frame_decode(const uint8_t *packet, size_t length,
    const uint8_t **body, size_t *body_length);

#endif
