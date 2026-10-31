/* Copyright (c) 2026 Tunuino. */
#include "tunuino/stream_rx.h"
#include <string.h>

static tunuino_rx_status fail(tunuino_stream_rx *rx, tunuino_rx_status status) {
    rx->used = 0;
    rx->expected = 0;
    rx->state = status;
    return status;
}
static tunuino_rx_status clock_check(tunuino_stream_rx *rx, uint64_t now_ms) {
    if (rx->state != TUNUINO_RX_MORE && rx->state != TUNUINO_RX_READY) { return rx->state; }
    if (now_ms < rx->last_ms) { return fail(rx, TUNUINO_RX_CLOCK); }
    rx->last_ms = now_ms;
    if (rx->state == TUNUINO_RX_MORE && rx->used != 0 && now_ms - rx->started_ms >= rx->timeout_ms) {
        return fail(rx, TUNUINO_RX_TIMEOUT);
    }
    return rx->state;
}
tunuino_rx_status tunuino_rx_init(tunuino_stream_rx *rx, uint32_t timeout_ms, uint64_t now_ms) {
    if (rx == NULL) { return TUNUINO_RX_ARGUMENT; }
    memset(rx, 0, sizeof(*rx));
    rx->state = TUNUINO_RX_ARGUMENT;
    if (timeout_ms == 0 || timeout_ms > TUNUINO_RX_TIMEOUT_MAX_MS) { return rx->state; }
    rx->timeout_ms = timeout_ms;
    rx->last_ms = now_ms;
    rx->state = TUNUINO_RX_MORE;
    return rx->state;
}
tunuino_rx_status tunuino_rx_feed(tunuino_stream_rx *rx, const uint8_t *bytes,
    size_t length, uint64_t now_ms, size_t *consumed) {
    if (consumed != NULL) { *consumed = 0; }
    if (rx == NULL || consumed == NULL || (length != 0 && bytes == NULL)) { return TUNUINO_RX_ARGUMENT; }
    if (clock_check(rx, now_ms) != TUNUINO_RX_MORE || length == 0) { return rx->state; }
    if (rx->used == 0) { rx->started_ms = now_ms; }
    while (*consumed < length) {
        size_t target = rx->used < 2 ? 2 : rx->expected;
        size_t count = target - rx->used;
        if (count > length - *consumed) { count = length - *consumed; }
        memcpy(rx->packet + rx->used, bytes + *consumed, count);
        rx->used += count;
        *consumed += count;
        if (rx->used == 2 && rx->expected == 0) {
            size_t body = ((size_t)rx->packet[0] << 8) | rx->packet[1];
            if (body == 0 || body > TUNUINO_MAX_BODY) { return fail(rx, TUNUINO_RX_LENGTH); }
            rx->expected = body + TUNUINO_FRAME_OVERHEAD;
        }
        if (rx->expected != 0 && rx->used == rx->expected) {
            const uint8_t *body;
            size_t body_length;
            if (tunuino_frame_decode(rx->packet, rx->used, &body, &body_length) != TUNUINO_FRAME_OK) {
                return fail(rx, TUNUINO_RX_CRC);
            }
            rx->state = TUNUINO_RX_READY;
            return rx->state;
        }
    }
    return TUNUINO_RX_MORE;
}
tunuino_rx_status tunuino_rx_poll(tunuino_stream_rx *rx, uint64_t now_ms) {
    size_t consumed;
    return tunuino_rx_feed(rx, NULL, 0, now_ms, &consumed);
}
tunuino_rx_status tunuino_rx_view(const tunuino_stream_rx *rx, const uint8_t **packet, size_t *length) {
    if (packet != NULL) { *packet = NULL; }
    if (length != NULL) { *length = 0; }
    if (rx == NULL || packet == NULL || length == NULL) { return TUNUINO_RX_ARGUMENT; }
    if (rx->state == TUNUINO_RX_READY) { *packet = rx->packet; *length = rx->used; }
    return rx->state;
}
tunuino_rx_status tunuino_rx_release(tunuino_stream_rx *rx, uint64_t now_ms) {
    if (rx == NULL) { return TUNUINO_RX_ARGUMENT; }
    tunuino_rx_status status = clock_check(rx, now_ms);
    if (status != TUNUINO_RX_READY) { return status == TUNUINO_RX_MORE ? TUNUINO_RX_ARGUMENT : status; }
    rx->used = 0;
    rx->expected = 0;
    rx->state = TUNUINO_RX_MORE;
    return rx->state;
}
void tunuino_rx_abort(tunuino_stream_rx *rx) {
    if (rx != NULL) { (void)fail(rx, TUNUINO_RX_CLOSED); }
}
