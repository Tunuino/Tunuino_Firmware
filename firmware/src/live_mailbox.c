/* Copyright (c) 2026 Tunuino. */
#include "tunuino/live_mailbox.h"
#include <stddef.h>
#include <string.h>

#define DIRTY 4u
#define INDEX 3u

tunuino_live_mailbox_status tunuino_live_mailbox_init(tunuino_live_mailbox *mailbox) {
    if (mailbox == NULL) return TUNUINO_MAILBOX_ARGUMENT;
    mailbox->configured = 0;
    atomic_init(&mailbox->middle,1u);
    if (!atomic_is_lock_free(&mailbox->middle)) return TUNUINO_MAILBOX_UNSUPPORTED;
    memset(mailbox->slots,0,sizeof(mailbox->slots));
    mailbox->front = 0; mailbox->back = 2; mailbox->have_front = 0;
    mailbox->configured = 1;
    return TUNUINO_MAILBOX_READY;
}

tunuino_live_mailbox_status tunuino_live_mailbox_publish(tunuino_live_mailbox *mailbox,
    const tunuino_live_sample *sample) {
    if (mailbox == NULL || !mailbox->configured || sample == NULL)
        return TUNUINO_MAILBOX_ARGUMENT;
    mailbox->slots[mailbox->back] = *sample;
    /* Release the completed slot; acquire the slot retired by the reader or
     * the prior unpublished front candidate. Never acquire the reader's front. */
    mailbox->back = atomic_exchange_explicit(&mailbox->middle,mailbox->back | DIRTY,
        memory_order_acq_rel) & INDEX;
    return TUNUINO_MAILBOX_READY;
}

tunuino_live_mailbox_status tunuino_live_mailbox_read(tunuino_live_mailbox *mailbox,
    uint64_t now_ms, uint32_t max_age_ms, tunuino_live_sample *sample) {
    if (sample == NULL) return TUNUINO_MAILBOX_ARGUMENT;
    memset(sample,0,sizeof(*sample));
    if (mailbox == NULL || !mailbox->configured || max_age_ms == 0)
        return TUNUINO_MAILBOX_ARGUMENT;
    if (atomic_load_explicit(&mailbox->middle,memory_order_acquire) & DIRTY) {
        /* Release the previously read front only after its prior copy finished;
         * acquire the most recent complete publication without a retry loop. */
        mailbox->front = atomic_exchange_explicit(&mailbox->middle,mailbox->front,
            memory_order_acq_rel) & INDEX;
        mailbox->have_front = 1;
    }
    if (!mailbox->have_front) return TUNUINO_MAILBOX_NOT_READY;
    const tunuino_live_sample *front = &mailbox->slots[mailbox->front];
    if (front->valid_mask != TUNUINO_LIVE_ALL_VALID || front->time_ms > now_ms
        || now_ms - front->time_ms >= max_age_ms) return TUNUINO_MAILBOX_NOT_READY;
    *sample = *front;
    return TUNUINO_MAILBOX_READY;
}
