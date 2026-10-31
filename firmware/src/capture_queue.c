/* Copyright (c) 2026 Tunuino. */
#include "tunuino/capture_queue.h"
_Static_assert((TUNUINO_CAPTURE_CAPACITY & (TUNUINO_CAPTURE_CAPACITY - 1u)) == 0,
    "Capacity must divide the unsigned counter modulus");

tunuino_capture_status tunuino_capture_init(tunuino_capture_queue *q) {
    if (q == NULL) { return TUNUINO_CAPTURE_ARGUMENT; }
    atomic_init(&q->written,0); atomic_init(&q->taken,0); atomic_init(&q->fault,TUNUINO_CAPTURE_OK);
    q->have_last = 0; q->last_time_us = 0; q->last_kind = TUNUINO_CAPTURE_CRANK;
    return TUNUINO_CAPTURE_OK;
}
tunuino_capture_status tunuino_capture_state(const tunuino_capture_queue *q) {
    if (q == NULL) { return TUNUINO_CAPTURE_ARGUMENT; }
    return (tunuino_capture_status)atomic_load_explicit(&q->fault,memory_order_acquire);
}
static tunuino_capture_status fail(tunuino_capture_queue *q, tunuino_capture_status status) {
    atomic_store_explicit(&q->fault,(unsigned)status,memory_order_release);
    return status;
}
tunuino_capture_status tunuino_capture_abort(tunuino_capture_queue *q) {
    tunuino_capture_status status = tunuino_capture_state(q);
    return status == TUNUINO_CAPTURE_OK ? fail(q,TUNUINO_CAPTURE_SOURCE_FAULT) : status;
}
tunuino_capture_status tunuino_capture_push(tunuino_capture_queue *q, tunuino_capture_event event) {
    tunuino_capture_status status = tunuino_capture_state(q);
    if (status != TUNUINO_CAPTURE_OK) { return status; }
    if (event.kind != TUNUINO_CAPTURE_CRANK && event.kind != TUNUINO_CAPTURE_CAM) {
        return fail(q,TUNUINO_CAPTURE_ARGUMENT);
    }
    if (q->have_last && (event.time_us < q->last_time_us
        || (event.time_us == q->last_time_us && q->last_kind == TUNUINO_CAPTURE_CAM && event.kind == TUNUINO_CAPTURE_CRANK))) {
        return fail(q,TUNUINO_CAPTURE_ORDER);
    }
    unsigned written = atomic_load_explicit(&q->written,memory_order_relaxed);
    unsigned taken = atomic_load_explicit(&q->taken,memory_order_acquire);
    if (written - taken >= TUNUINO_CAPTURE_CAPACITY) { return fail(q,TUNUINO_CAPTURE_OVERFLOW); }
    q->events[written % TUNUINO_CAPTURE_CAPACITY] = event;
    q->last_time_us = event.time_us; q->last_kind = event.kind; q->have_last = 1;
    atomic_store_explicit(&q->written,written + 1u,memory_order_release);
    return TUNUINO_CAPTURE_OK;
}
tunuino_capture_status tunuino_capture_take(tunuino_capture_queue *q, tunuino_capture_event *event) {
    if (event == NULL) { return TUNUINO_CAPTURE_ARGUMENT; }
    *event = (tunuino_capture_event){0,0};
    tunuino_capture_status status = tunuino_capture_state(q);
    if (status != TUNUINO_CAPTURE_OK) { return status; }
    unsigned taken = atomic_load_explicit(&q->taken,memory_order_relaxed);
    unsigned written = atomic_load_explicit(&q->written,memory_order_acquire);
    if (taken == written) { return TUNUINO_CAPTURE_EMPTY; }
    tunuino_capture_event next = q->events[taken % TUNUINO_CAPTURE_CAPACITY];
    atomic_store_explicit(&q->taken,taken + 1u,memory_order_release);
    status = tunuino_capture_state(q);
    if (status != TUNUINO_CAPTURE_OK) { return status; }
    *event = next;
    return TUNUINO_CAPTURE_OK;
}
size_t tunuino_capture_count(const tunuino_capture_queue *q) {
    if (tunuino_capture_state(q) != TUNUINO_CAPTURE_OK) { return 0; }
    unsigned taken = atomic_load_explicit(&q->taken,memory_order_acquire);
    unsigned written = atomic_load_explicit(&q->written,memory_order_acquire);
    unsigned count = written - taken;
    return count > TUNUINO_CAPTURE_CAPACITY ? TUNUINO_CAPTURE_CAPACITY : count;
}
