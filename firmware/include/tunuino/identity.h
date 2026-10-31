/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_IDENTITY_H
#define TUNUINO_IDENTITY_H

#include "tunuino/frame.h"

#define TUNUINO_IDENTITY_BODY_SIZE 32u
#define TUNUINO_IDENTITY_PROTOCOL 1u
#define TUNUINO_CAPABILITY_MASK UINT32_C(0x1F)

typedef enum { TUNUINO_PROTEUS = 1, TUNUINO_UAEFI = 2 } tunuino_family;
typedef enum { TUNUINO_STM32_F4 = 1, TUNUINO_STM32_F7 = 2, TUNUINO_STM32_H7 = 3 } tunuino_mcu_series;

typedef struct {
    tunuino_family family;
    tunuino_mcu_series mcu_series;
    uint16_t hardware_revision;
    uint8_t firmware_major;
    uint8_t firmware_minor;
    uint8_t firmware_patch;
    uint32_t capabilities;
    uint8_t uid[12];
} tunuino_identity;

typedef enum {
    TUNUINO_IDENTITY_OK = 0,
    TUNUINO_IDENTITY_ARGUMENT,
    TUNUINO_IDENTITY_BAD_FRAME,
    TUNUINO_IDENTITY_BAD_REQUEST,
    TUNUINO_IDENTITY_UNCONFIGURED,
    TUNUINO_IDENTITY_CAPACITY
} tunuino_identity_status;

/* Shared validation for transports and dispatchers; no inferred defaults. */
tunuino_identity_status tunuino_identity_validate(const tunuino_identity *identity);

/* identity must be a stable snapshot supplied by the board layer.
 * No board defaults, MCU pin assignments or capabilities are inferred here. */
tunuino_identity_status tunuino_identity_reply(const tunuino_identity *identity,
    const uint8_t *request_packet, size_t request_length,
    uint8_t *response_packet, size_t capacity, size_t *written);

#endif
