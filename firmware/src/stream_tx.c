/* Copyright (c) 2026 Tunuino. */
#include "tunuino/stream_tx.h"
#include <string.h>

static tunuino_tx_status fail(tunuino_stream_tx *tx, tunuino_tx_status status) {
    tx->length = 0;
    tx->completed = 0;
    tx->state = status;
    return status;
}
tunuino_tx_status tunuino_tx_poll(tunuino_stream_tx *tx, uint64_t now_ms) {
    if (tx == NULL) { return TUNUINO_TX_ARGUMENT; }
    if (tx->state != TUNUINO_TX_IDLE && tx->state != TUNUINO_TX_PENDING && tx->state != TUNUINO_TX_DONE) {
        return tx->state;
    }
    if (now_ms < tx->last_ms) { return fail(tx, TUNUINO_TX_CLOCK); }
    tx->last_ms = now_ms;
    if (tx->state == TUNUINO_TX_PENDING && now_ms - tx->started_ms >= tx->timeout_ms) {
        return fail(tx, TUNUINO_TX_TIMEOUT);
    }
    return tx->state;
}
tunuino_tx_status tunuino_tx_init(tunuino_stream_tx *tx, uint32_t timeout_ms, uint64_t now_ms) {
    if (tx == NULL) { return TUNUINO_TX_ARGUMENT; }
    memset(tx, 0, sizeof(*tx));
    tx->state = TUNUINO_TX_ARGUMENT;
    if (timeout_ms == 0 || timeout_ms > TUNUINO_TX_TIMEOUT_MAX_MS) { return tx->state; }
    tx->timeout_ms = timeout_ms;
    tx->last_ms = now_ms;
    tx->state = TUNUINO_TX_IDLE;
    return tx->state;
}
tunuino_tx_status tunuino_tx_begin(tunuino_stream_tx *tx, const uint8_t *packet,
    size_t length, uint64_t now_ms) {
    if (tx == NULL || packet == NULL) { return TUNUINO_TX_ARGUMENT; }
    tunuino_tx_status status = tunuino_tx_poll(tx, now_ms);
    if (status == TUNUINO_TX_PENDING) { return TUNUINO_TX_BUSY; }
    if (status != TUNUINO_TX_IDLE && status != TUNUINO_TX_DONE) { return status; }
    const uint8_t *body;
    size_t body_length;
    if (tunuino_frame_decode(packet, length, &body, &body_length) != TUNUINO_FRAME_OK) {
        return TUNUINO_TX_FRAME;
    }
    return tunuino_tx_begin_plain(tx,packet,length,now_ms);
}
tunuino_tx_status tunuino_tx_begin_plain(tunuino_stream_tx *tx, const uint8_t *packet,
    size_t length, uint64_t now_ms) {
    if (tx == NULL || packet == NULL || length == 0 || length > sizeof(tx->packet)) {
        return TUNUINO_TX_ARGUMENT;
    }
    tunuino_tx_status status = tunuino_tx_poll(tx, now_ms);
    if (status == TUNUINO_TX_PENDING) return TUNUINO_TX_BUSY;
    if (status != TUNUINO_TX_IDLE && status != TUNUINO_TX_DONE) return status;
    memcpy(tx->packet, packet, length);
    tx->length = length;
    tx->completed = 0;
    tx->started_ms = now_ms;
    tx->state = TUNUINO_TX_PENDING;
    return tx->state;
}
tunuino_tx_status tunuino_tx_view(tunuino_stream_tx *tx, uint64_t now_ms,
    const uint8_t **bytes, size_t *remaining) {
    if (bytes != NULL) { *bytes = NULL; }
    if (remaining != NULL) { *remaining = 0; }
    if (tx == NULL || bytes == NULL || remaining == NULL) { return TUNUINO_TX_ARGUMENT; }
    tunuino_tx_status status = tunuino_tx_poll(tx, now_ms);
    if (status == TUNUINO_TX_PENDING) {
        *bytes = tx->packet + tx->completed;
        *remaining = tx->length - tx->completed;
    }
    return status;
}
tunuino_tx_status tunuino_tx_advance(tunuino_stream_tx *tx, size_t completed, uint64_t now_ms) {
    tunuino_tx_status status = tunuino_tx_poll(tx, now_ms);
    if (status != TUNUINO_TX_PENDING) {
        return status == TUNUINO_TX_IDLE || status == TUNUINO_TX_DONE ? TUNUINO_TX_ARGUMENT : status;
    }
    if (completed > tx->length - tx->completed) { return fail(tx, TUNUINO_TX_COUNT); }
    tx->completed += completed;
    if (tx->completed == tx->length) { tx->state = TUNUINO_TX_DONE; }
    return tx->state;
}
void tunuino_tx_abort(tunuino_stream_tx *tx) {
    if (tx != NULL) { (void)fail(tx, TUNUINO_TX_CLOSED); }
}
