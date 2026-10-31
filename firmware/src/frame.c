/* Copyright (c) 2026 Tunuino. */
#include "tunuino/frame.h"
#include <string.h>

uint32_t tunuino_crc32(const uint8_t *bytes, size_t length) {
    uint32_t value = UINT32_C(0xFFFFFFFF);
    for (size_t i = 0; i < length; ++i) {
        value ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            value = (value >> 1) ^ ((value & 1u) ? UINT32_C(0xEDB88320) : 0u);
        }
    }
    return ~value;
}

tunuino_frame_status tunuino_frame_encode(const uint8_t *body, size_t length,
    uint8_t *packet, size_t capacity, size_t *written) {
    if (written != NULL) {
        *written = 0;
    }
    if (body == NULL || packet == NULL || written == NULL) {
        return TUNUINO_FRAME_ARGUMENT;
    }
    if (length == 0 || length > TUNUINO_MAX_BODY) {
        return TUNUINO_FRAME_LENGTH;
    }
    if (capacity < length + TUNUINO_FRAME_OVERHEAD) {
        return TUNUINO_FRAME_CAPACITY;
    }
    /* Move first so in-place body encoding is supported. */
    memmove(packet + 2, body, length);
    packet[0] = (uint8_t)(length >> 8);
    packet[1] = (uint8_t)length;
    uint32_t crc = tunuino_crc32(packet + 2, length);
    for (unsigned i = 0; i < 4; ++i) {
        packet[2 + length + i] = (uint8_t)(crc >> (24u - i * 8u));
    }
    *written = length + TUNUINO_FRAME_OVERHEAD;
    return TUNUINO_FRAME_OK;
}

tunuino_frame_status tunuino_frame_decode(const uint8_t *packet, size_t length,
    const uint8_t **body, size_t *body_length) {
    if (body != NULL) {
        *body = NULL;
    }
    if (body_length != NULL) {
        *body_length = 0;
    }
    if (packet == NULL || body == NULL || body_length == NULL) {
        return TUNUINO_FRAME_ARGUMENT;
    }
    if (length < TUNUINO_FRAME_OVERHEAD + 1u) {
        return TUNUINO_FRAME_LENGTH;
    }
    size_t count = ((size_t)packet[0] << 8) | packet[1];
    if (count == 0 || count > TUNUINO_MAX_BODY || length != count + TUNUINO_FRAME_OVERHEAD) {
        return TUNUINO_FRAME_LENGTH;
    }
    uint32_t received = 0;
    for (unsigned i = 0; i < 4; ++i) {
        received = (received << 8) | packet[2 + count + i];
    }
    if (received != tunuino_crc32(packet + 2, count)) {
        return TUNUINO_FRAME_CRC;
    }
    *body = packet + 2;
    *body_length = count;
    return TUNUINO_FRAME_OK;
}
