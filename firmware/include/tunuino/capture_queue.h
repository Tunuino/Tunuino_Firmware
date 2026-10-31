/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_CAPTURE_QUEUE_H
#define TUNUINO_CAPTURE_QUEUE_H
#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>
#if ATOMIC_INT_LOCK_FREE != 2
#error "Capture queue requires always-lock-free C11 atomic unsigned integers"
#endif
#define TUNUINO_CAPTURE_CAPACITY 64u
typedef enum { TUNUINO_CAPTURE_CRANK = 1, TUNUINO_CAPTURE_CAM = 2 } tunuino_capture_kind;
typedef struct { uint64_t time_us; tunuino_capture_kind kind; } tunuino_capture_event;
typedef enum {
    TUNUINO_CAPTURE_OK = 0, TUNUINO_CAPTURE_EMPTY, TUNUINO_CAPTURE_ARGUMENT,
    TUNUINO_CAPTURE_OVERFLOW, TUNUINO_CAPTURE_ORDER, TUNUINO_CAPTURE_SOURCE_FAULT
} tunuino_capture_status;
/* Exactly one serialized producer and one consumer. No reentrant/nested
 * producers. Initialize/reset only while both sides and hardware are stopped.
 * Fields private. No arguments/output buffers may overlap this object. */
typedef struct {
    tunuino_capture_event events[TUNUINO_CAPTURE_CAPACITY];
    atomic_uint written, taken, fault;
    uint64_t last_time_us;
    tunuino_capture_kind last_kind;
    unsigned have_last;
} tunuino_capture_queue;
tunuino_capture_status tunuino_capture_init(tunuino_capture_queue *queue);
/* Producer only. Nondecreasing extended timestamps; at equal timestamps crank
 * must precede cam. Overflow/order/invalid kind latch a failure, never retry. */
tunuino_capture_status tunuino_capture_push(tunuino_capture_queue *queue, tunuino_capture_event event);
/* Producer only: latch a reported acquisition fault, preserving a prior fault.
 * Pending events are no longer deliverable. Reset only with both sides stopped. */
tunuino_capture_status tunuino_capture_abort(tunuino_capture_queue *queue);
/* Consumer only. EMPTY/errors zero the event. No dequeue after latched fault. */
tunuino_capture_status tunuino_capture_take(tunuino_capture_queue *queue, tunuino_capture_event *event);
tunuino_capture_status tunuino_capture_state(const tunuino_capture_queue *queue);
/* Bounded occupancy snapshot; may overestimate under concurrent activity. */
size_t tunuino_capture_count(const tunuino_capture_queue *queue);
#endif
