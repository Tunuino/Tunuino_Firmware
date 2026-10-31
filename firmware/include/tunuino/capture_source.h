/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_CAPTURE_SOURCE_H
#define TUNUINO_CAPTURE_SOURCE_H
#include "tunuino/clock32.h"
#include "tunuino/capture_processor.h"
typedef enum {
    TUNUINO_SOURCE_OK = 0, TUNUINO_SOURCE_CLOCK, TUNUINO_SOURCE_CAPTURE,
    TUNUINO_SOURCE_QUEUE, TUNUINO_SOURCE_LOST, TUNUINO_SOURCE_ARGUMENT
} tunuino_capture_source_status;
/* One serialized producer owns observe/push/lost and the private fields.
 * One consumer may call processor drain/read/telemetry on processor ONLY.
 * No other queue producer. Initialize/reinitialize with both sides and hardware
 * stopped; do not inspect producer fields from the consumer. No overlapping
 * arguments. All clock32 timing/driver requirements still apply. */
typedef struct {
    tunuino_capture_processor processor;
    tunuino_clock32 clock;
    uint32_t max_age_us;
    unsigned configured;
    tunuino_capture_source_status state;
} tunuino_capture_source;
tunuino_capture_source_status tunuino_capture_source_init(tunuino_capture_source *source,
    const tunuino_position_config *config, uint32_t first_counter, uint64_t first_us,
    uint32_t max_step_us, uint32_t max_age_us);
/* Periodic producer observation, also when no edges arrive. Errors zero time.
 * Clock failure aborts the queue. NULL output is a nonfatal argument error. */
tunuino_capture_source_status tunuino_capture_source_observe(tunuino_capture_source *source,
    uint32_t counter, uint64_t *time_us);
/* Latch capture in driver, then observe a coherent current counter, then push
 * captures in chronological order (crank before cam at equal timestamps).
 * Rejected capture or queue failure aborts acquisition until quiescent init. */
tunuino_capture_source_status tunuino_capture_source_push(tunuino_capture_source *source,
    uint32_t captured_counter, tunuino_capture_kind kind);
/* Producer reports overcapture, timer reset, suspend, or a missed timing bound.
 * Latches LOST without publishing an event. No automatic recovery. */
tunuino_capture_source_status tunuino_capture_source_lost(tunuino_capture_source *source);
typedef struct { uint32_t counter; tunuino_capture_kind kind; } tunuino_raw_capture;
/* Producer adapter for one timer snapshot, at most one capture per channel.
 * Driver latches ALL pending captures before sampling current_counter, checks
 * overcapture/reset/timing loss, then calls this function. It must guarantee no
 * older pending capture remains for the next call. No hardware register reads
 * are performed here. Zero captures provides a periodic clock observation.
 * Captures are validated before publishing and sorted by extended time, crank
 * first on ties. Invalid input/capture or reported loss latches acquisition off.
 * Publication is NOT atomic as a batch; queue failure can follow a published
 * event. Consumers must retain the existing queue-fault/freshness checks.
 * time_us is zero on errors; NULL output is a nonfatal argument error.
 * Buffers must not overlap source or each other. */
tunuino_capture_source_status tunuino_capture_source_sample(tunuino_capture_source *source,
    uint32_t current_counter, const tunuino_raw_capture *captures, size_t count,
    unsigned events_lost, uint64_t *time_us);
#endif
