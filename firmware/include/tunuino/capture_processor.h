/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_CAPTURE_PROCESSOR_H
#define TUNUINO_CAPTURE_PROCESSOR_H
#include "tunuino/capture_queue.h"
#include "tunuino/position.h"
typedef enum {
    TUNUINO_PROCESS_READY = 0, TUNUINO_PROCESS_NO_SIGNAL,
    TUNUINO_PROCESS_BACKLOG, TUNUINO_PROCESS_FUTURE,
    TUNUINO_PROCESS_QUEUE_FAULT, TUNUINO_PROCESS_CLOCK, TUNUINO_PROCESS_ARGUMENT
} tunuino_capture_process_status;
/* One serialized producer calls push; one serialized consumer calls drain/read.
 * Initialize with both sides/hardware stopped. Fields private except read-only
 * consumer diagnostics state/queue_status/decoder_status. Do not bypass this
 * object to read the internal decoder, whose snapshot may be intentionally hidden.
 * Arguments and output buffers must not overlap this object. */
typedef struct {
    tunuino_capture_queue queue;
    tunuino_position position;
    tunuino_position_config config;
    tunuino_position_reading cached;
    tunuino_cam_reading cached_cam;
    tunuino_capture_event deferred;
    uint64_t last_wall_us, last_event_us, last_crank_us;
    unsigned configured, have_crank, have_deferred, expired;
    tunuino_capture_process_status state;
    tunuino_capture_status queue_status;
    tunuino_position_status decoder_status;
} tunuino_capture_processor;
tunuino_capture_process_status tunuino_capture_processor_init(tunuino_capture_processor *processor,
    const tunuino_position_config *config, uint64_t now_us);
/* Producer only. Does not inspect consumer-owned state. */
tunuino_capture_status tunuino_capture_processor_push(tunuino_capture_processor *processor, tunuino_capture_event event);
/* Consumer only. Budget is 1..64 decoded events, not a deferred future event.
 * Decoder time follows captured timestamps; wall time is only a freshness check. */
tunuino_capture_process_status tunuino_capture_processor_drain(tunuino_capture_processor *processor,
    uint64_t now_us, size_t budget, size_t *processed);
/* Consumer only. Zeroes output unless queue is empty, healthy and fresh. */
tunuino_capture_process_status tunuino_capture_processor_read(tunuino_capture_processor *processor,
    uint64_t now_us, tunuino_position_reading *reading);
/* Same freshness/queue gates, plus the reference timestamp for the tooth. */
tunuino_capture_process_status tunuino_capture_processor_anchor(tunuino_capture_processor *processor,
    uint64_t now_us,tunuino_position_reading *reading,uint64_t *crank_edge_us);
/* Same queue/freshness gates as the position snapshot; consumer only. */
tunuino_capture_process_status tunuino_capture_processor_cam(tunuino_capture_processor *processor,
    uint64_t now_us,tunuino_cam_reading *reading);
#endif
