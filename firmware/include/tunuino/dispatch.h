/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_DISPATCH_H
#define TUNUINO_DISPATCH_H
#include "tunuino/identity.h"
#include "tunuino/live_data.h"
#include "tunuino/map_read.h"

/* Largest response of the current read-only handlers, including framing.
 * Update when introducing a handler with a larger response. */
#define TUNUINO_DISPATCH_RESPONSE_MAX (TUNUINO_MAP_CHUNK_MAX + 18u)

#define TUNUINO_DISPATCH_LIVE UINT32_C(1)
#define TUNUINO_DISPATCH_MAP_READ UINT32_C(2)
#define TUNUINO_DISPATCH_PARTIAL UINT32_C(4)
#define TUNUINO_DISPATCH_IO UINT32_C(8)
struct tunuino_device;
/* Board-owned snapshots. Serialize dispatch with all updates, including DMA/ISR
 * producers. live_mailbox may transfer acquisition data to a USB-owner local
 * live snapshot; serialize mailbox reads into that copy with dispatch. Never
 * point live at a mailbox slot or at storage being written by another task.
 * No acquisition, USB driver, storage or engine actuation here. */
typedef struct {
    const tunuino_identity *identity;
    const tunuino_live_sample *live;
    const tunuino_map_snapshot *ve, *ignition;
    struct tunuino_device *device;
} tunuino_dispatch_context;
typedef enum {
    TUNUINO_DISPATCH_OK = 0, TUNUINO_DISPATCH_ARGUMENT,
    TUNUINO_DISPATCH_BAD_FRAME, TUNUINO_DISPATCH_UNKNOWN_REQUEST,
    TUNUINO_DISPATCH_UNCONFIGURED, TUNUINO_DISPATCH_UNSUPPORTED,
    TUNUINO_DISPATCH_BAD_REQUEST, TUNUINO_DISPATCH_SOURCE_ERROR,
    TUNUINO_DISPATCH_CAPACITY
} tunuino_dispatch_status;
/* Reject capabilities with no implementation or no configured provider.
 * Pointers alone do not enable a capability. Unavailable snapshots may be
 * supplied with generation=0 (maps) or valid_mask=0 (live signals). */
tunuino_dispatch_status tunuino_dispatch_validate(const tunuino_dispatch_context *context);
/* One complete framed request per call. Success produces one complete frame.
 * Errors reset written to zero and produce no transmissible response. The
 * output buffer must not overlap context or any provider snapshot. Request and
 * response buffers may overlap. Caller implements stream assembly and timeout. */
tunuino_dispatch_status tunuino_dispatch_reply(const tunuino_dispatch_context *context,
    const uint8_t *request_packet, size_t request_length,
    uint8_t *response_packet, size_t capacity, size_t *written);
/* Optional plain TunerStudio probes for supported Proteus profiles. Q/S
 * return the signature, F the serial protocol version and V the firmware
 * version. The IO profile additionally answers C (test) and I (local CAN ID).
 * The caller may use these only at a fresh frame boundary. */
tunuino_dispatch_status tunuino_dispatch_plain_reply(const tunuino_dispatch_context *context,
    uint8_t command, uint8_t *response, size_t capacity, size_t *written);
#endif
