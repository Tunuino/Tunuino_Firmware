/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_STREAM_TX_H
#define TUNUINO_STREAM_TX_H
#include "tunuino/frame.h"
#define TUNUINO_TX_TIMEOUT_MAX_MS UINT32_C(60000)
typedef enum {
    TUNUINO_TX_IDLE = 0, TUNUINO_TX_PENDING, TUNUINO_TX_DONE,
    TUNUINO_TX_ARGUMENT, TUNUINO_TX_FRAME, TUNUINO_TX_BUSY,
    TUNUINO_TX_TIMEOUT, TUNUINO_TX_CLOCK, TUNUINO_TX_COUNT, TUNUINO_TX_CLOSED
} tunuino_tx_status;
/* Initialize before use. Single-threaded, caller-owned storage; fields private.
 * Arguments and input/output buffers must not overlap this object. */
typedef struct {
    uint8_t packet[TUNUINO_MAX_BODY + TUNUINO_FRAME_OVERHEAD];
    size_t length, completed;
    uint64_t started_ms, last_ms;
    uint32_t timeout_ms;
    tunuino_tx_status state;
} tunuino_stream_tx;
tunuino_tx_status tunuino_tx_init(tunuino_stream_tx *tx, uint32_t timeout_ms, uint64_t now_ms);
/* Copies one valid complete frame. Cannot replace a pending response. */
tunuino_tx_status tunuino_tx_begin(tunuino_stream_tx *tx, const uint8_t *packet,
    size_t length, uint64_t now_ms);
/* Legacy discovery responses have no envelope. Copies 1..packet-capacity bytes
 * with the same timeout, ownership and physical-completion rules. */
tunuino_tx_status tunuino_tx_begin_plain(tunuino_stream_tx *tx, const uint8_t *bytes,
    size_t length, uint64_t now_ms);
/* View stays valid until completed/abort/init; driver must finish/cancel any
 * asynchronous use before another begin/init. Do not acknowledge queued bytes. */
tunuino_tx_status tunuino_tx_view(tunuino_stream_tx *tx, uint64_t now_ms,
    const uint8_t **bytes, size_t *remaining);
/* Acknowledge only bytes whose transfer has completed, in order. Zero means no
 * progress. Over-reporting poisons the stream; completion never extends time. */
tunuino_tx_status tunuino_tx_advance(tunuino_stream_tx *tx, size_t completed, uint64_t now_ms);
tunuino_tx_status tunuino_tx_poll(tunuino_stream_tx *tx, uint64_t now_ms);
/* Sticky failure; caller must cancel/drain the driver before fresh init. */
void tunuino_tx_abort(tunuino_stream_tx *tx);
#endif
