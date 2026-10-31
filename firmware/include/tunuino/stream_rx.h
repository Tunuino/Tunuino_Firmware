/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_STREAM_RX_H
#define TUNUINO_STREAM_RX_H
#include "tunuino/frame.h"
#define TUNUINO_RX_TIMEOUT_MAX_MS UINT32_C(60000)
typedef enum {
    TUNUINO_RX_MORE = 0, TUNUINO_RX_READY, TUNUINO_RX_ARGUMENT,
    TUNUINO_RX_LENGTH, TUNUINO_RX_CRC, TUNUINO_RX_TIMEOUT,
    TUNUINO_RX_CLOCK, TUNUINO_RX_CLOSED
} tunuino_rx_status;
/* Caller-owned, single-threaded state. Initialize before any other operation.
 * Treat fields as private; input/output arguments must not overlap this object. */
typedef struct {
    uint8_t packet[TUNUINO_MAX_BODY + TUNUINO_FRAME_OVERHEAD];
    size_t used, expected;
    uint64_t started_ms, last_ms;
    uint32_t timeout_ms;
    tunuino_rx_status state;
} tunuino_stream_rx;
/* now_ms is a monotonic, extended 64-bit millisecond clock. */
tunuino_rx_status tunuino_rx_init(tunuino_stream_rx *rx, uint32_t timeout_ms, uint64_t now_ms);
/* Stops after one frame. The caller owns any unconsumed input bytes.
 * A zero-length feed may use NULL bytes; it checks the total frame deadline.
 * A ready frame stays ready until release. Transport errors are sticky. */
tunuino_rx_status tunuino_rx_feed(tunuino_stream_rx *rx, const uint8_t *bytes,
    size_t length, uint64_t now_ms, size_t *consumed);
tunuino_rx_status tunuino_rx_poll(tunuino_stream_rx *rx, uint64_t now_ms);
/* View only: pass to dispatcher with a separate response buffer. */
tunuino_rx_status tunuino_rx_view(const tunuino_stream_rx *rx,
    const uint8_t **packet, size_t *length);
/* Only after the ready request/response is handled. Does not recover errors. */
tunuino_rx_status tunuino_rx_release(tunuino_stream_rx *rx, uint64_t now_ms);
/* Discard partial/ready input on disconnect; only fresh init can reopen. */
void tunuino_rx_abort(tunuino_stream_rx *rx);
#endif
