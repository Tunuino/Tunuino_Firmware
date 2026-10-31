/* Copyright (c) 2026 Tunuino. */
#include "tunuino/identity.h"
#include <string.h>

tunuino_identity_status tunuino_identity_validate(const tunuino_identity *identity) {
    if (identity == NULL) { return TUNUINO_IDENTITY_ARGUMENT; }
    if (identity->family != TUNUINO_PROTEUS && identity->family != TUNUINO_UAEFI) {
        return TUNUINO_IDENTITY_UNCONFIGURED;
    }
    if (identity->mcu_series < TUNUINO_STM32_F4 || identity->mcu_series > TUNUINO_STM32_H7
            || identity->hardware_revision == 0
            || (identity->capabilities & ~TUNUINO_CAPABILITY_MASK) != 0) {
        return TUNUINO_IDENTITY_UNCONFIGURED;
    }
    unsigned nonzero = 0, nonff = 0;
    for (size_t i = 0; i < sizeof(identity->uid); ++i) {
        nonzero |= identity->uid[i];
        nonff |= identity->uid[i] ^ 0xFFu;
    }
    return nonzero != 0 && nonff != 0 ? TUNUINO_IDENTITY_OK : TUNUINO_IDENTITY_UNCONFIGURED;
}

tunuino_identity_status tunuino_identity_reply(const tunuino_identity *identity,
    const uint8_t *request_packet, size_t request_length,
    uint8_t *response_packet, size_t capacity, size_t *written) {
    if (written != NULL) {
        *written = 0;
    }
    if (identity == NULL || request_packet == NULL || response_packet == NULL || written == NULL) {
        return TUNUINO_IDENTITY_ARGUMENT;
    }
    const uint8_t *request = NULL;
    size_t length = 0;
    if (tunuino_frame_decode(request_packet, request_length, &request, &length) != TUNUINO_FRAME_OK) {
        return TUNUINO_IDENTITY_BAD_FRAME;
    }
    static const uint8_t query[] = {'T', 'U', 'N', 'U', 'I', 'N', 'O', '?'};
    if (length != sizeof(query) || memcmp(request, query, sizeof(query)) != 0) {
        return TUNUINO_IDENTITY_BAD_REQUEST;
    }
    if (tunuino_identity_validate(identity) != TUNUINO_IDENTITY_OK) {
        return TUNUINO_IDENTITY_UNCONFIGURED;
    }
    if (capacity < TUNUINO_IDENTITY_BODY_SIZE + TUNUINO_FRAME_OVERHEAD) {
        return TUNUINO_IDENTITY_CAPACITY;
    }
    uint8_t body[TUNUINO_IDENTITY_BODY_SIZE] = {0};
    static const uint8_t magic[] = {'T', 'U', 'N', 'U', 'I', 'N', 'O'};
    memcpy(body + 1, magic, sizeof(magic));
    body[8] = TUNUINO_IDENTITY_PROTOCOL;
    body[9] = (uint8_t)identity->family;
    body[10] = (uint8_t)identity->mcu_series;
    body[11] = (uint8_t)(identity->hardware_revision >> 8);
    body[12] = (uint8_t)identity->hardware_revision;
    body[13] = identity->firmware_major;
    body[14] = identity->firmware_minor;
    body[15] = identity->firmware_patch;
    for (unsigned i = 0; i < 4; ++i) {
        body[16 + i] = (uint8_t)(identity->capabilities >> (24u - 8u * i));
    }
    memcpy(body + 20, identity->uid, sizeof(identity->uid));
    return tunuino_frame_encode(body, sizeof(body), response_packet, capacity, written) == TUNUINO_FRAME_OK
        ? TUNUINO_IDENTITY_OK : TUNUINO_IDENTITY_CAPACITY;
}
