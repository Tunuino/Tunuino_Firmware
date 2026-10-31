/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_LIVE_DATA_H
#define TUNUINO_LIVE_DATA_H
#include "tunuino/frame.h"

#define TUNUINO_LIVE_BODY_SIZE 28u
#define TUNUINO_LIVE_RPM_VALID 0x01u
#define TUNUINO_LIVE_MAP_VALID 0x02u
#define TUNUINO_LIVE_LAMBDA_VALID 0x04u
#define TUNUINO_LIVE_COOLANT_VALID 0x08u
#define TUNUINO_LIVE_ALL_VALID 0x0Fu

typedef struct {
    uint64_t time_ms;
    uint32_t rpm_milli;
    uint32_t map_milli_kpa;
    uint32_t lambda_million;
    int32_t coolant_milli_c;
    uint8_t valid_mask;
} tunuino_live_sample;

typedef enum {
    TUNUINO_LIVE_OK = 0, TUNUINO_LIVE_ARGUMENT, TUNUINO_LIVE_BAD_FRAME,
    TUNUINO_LIVE_BAD_REQUEST, TUNUINO_LIVE_BAD_SAMPLE, TUNUINO_LIVE_CAPACITY
} tunuino_live_status;

/* The board layer supplies a stable snapshot. Missing signals return NOT_READY,
 * never substituted values. This handler does not acquire physical sensors. */
tunuino_live_status tunuino_live_reply(const tunuino_live_sample *sample,
    const uint8_t *request_packet, size_t request_length,
    uint8_t *response_packet, size_t capacity, size_t *written);
tunuino_live_status tunuino_live_partial_reply(const tunuino_live_sample *sample,
    uint8_t *response_packet, size_t capacity, size_t *written);
#endif
