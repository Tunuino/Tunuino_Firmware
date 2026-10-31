/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_LIVE_MAILBOX_H
#define TUNUINO_LIVE_MAILBOX_H
#include "tunuino/live_data.h"
#include <stdatomic.h>

typedef enum {
    TUNUINO_MAILBOX_READY = 0, TUNUINO_MAILBOX_NOT_READY,
    TUNUINO_MAILBOX_ARGUMENT, TUNUINO_MAILBOX_UNSUPPORTED
} tunuino_live_mailbox_status;
/* Private state: exactly one serialized publisher and one serialized reader.
 * Triple buffering replaces unread samples with the latest; it is NOT a queue.
 * Initialize/reinitialize only with both sides stopped. Never copy this object.
 * Arguments must not overlap it or each other. No DMA writes into its slots.
 * Publisher: publish only. Reader/USB task: read only; serialize read with
 * dispatch using the caller-owned output, never a pointer to an internal slot.
 * No explicit locks, allocation or retry loops in publish/read. Initialization
 * refuses platforms where the index atomic is not lock-free. This alone does
 * not establish wait-freedom, ISR suitability or worst-case execution time on an MCU. */
typedef struct {
    tunuino_live_sample slots[3];
    atomic_uint middle;
    unsigned back, front, have_front, configured;
} tunuino_live_mailbox;
tunuino_live_mailbox_status tunuino_live_mailbox_init(tunuino_live_mailbox *mailbox);
/* Publish every current acquisition snapshot, including invalid snapshots on
 * loss of synchronization/sensors. A healthy publication does not imply signal
 * validation here; that remains with acquisition and the protocol handler. */
tunuino_live_mailbox_status tunuino_live_mailbox_publish(tunuino_live_mailbox *mailbox,
    const tunuino_live_sample *sample);
/* Latest coherent snapshot, or zeroed NOT_READY if absent, invalid, future or
 * age >= max_age_ms. Requires max_age_ms > 0 and monotonic now_ms in the SAME
 * time domain as sample.time_ms. A cached snapshot may be read repeatedly while
 * fresh. The publisher must also update periodically; this cannot cancel an
 * already-built USB reply or detect a fault not yet published. */
tunuino_live_mailbox_status tunuino_live_mailbox_read(tunuino_live_mailbox *mailbox,
    uint64_t now_ms, uint32_t max_age_ms, tunuino_live_sample *sample);
#endif
