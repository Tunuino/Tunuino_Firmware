/* Copyright (c) 2026 Tunuino. */
#include "tunuino/capture_source.h"
#include <stddef.h>
static tunuino_capture_source_status state(const tunuino_capture_source *source) {
    return source != NULL && source->configured ? source->state : TUNUINO_SOURCE_ARGUMENT;
}
static tunuino_capture_source_status fail(tunuino_capture_source *source, tunuino_capture_source_status status) {
    source->state = status;
    (void)tunuino_capture_abort(&source->processor.queue);
    return status;
}
tunuino_capture_source_status tunuino_capture_source_init(tunuino_capture_source *source,
    const tunuino_position_config *config, uint32_t first_counter, uint64_t first_us,
    uint32_t max_step_us, uint32_t max_age_us) {
    if (source == NULL) return TUNUINO_SOURCE_ARGUMENT;
    source->configured = 0; source->state = TUNUINO_SOURCE_ARGUMENT;
    source->processor.configured = 0;
    if (max_age_us == 0 || max_age_us > INT32_MAX ||
        tunuino_clock32_init(&source->clock,first_counter,first_us,max_step_us) != TUNUINO_CLOCK32_OK)
        return source->state;
    if (tunuino_capture_processor_init(&source->processor,config,first_us) != TUNUINO_PROCESS_NO_SIGNAL)
        return source->state;
    source->max_age_us = max_age_us;
    source->configured = 1; source->state = TUNUINO_SOURCE_OK;
    return source->state;
}
tunuino_capture_source_status tunuino_capture_source_observe(tunuino_capture_source *source,
    uint32_t counter, uint64_t *time_us) {
    if (time_us == NULL) return TUNUINO_SOURCE_ARGUMENT;
    *time_us = 0;
    tunuino_capture_source_status status = state(source);
    if (status != TUNUINO_SOURCE_OK) return status;
    if (tunuino_capture_state(&source->processor.queue) != TUNUINO_CAPTURE_OK)
        return fail(source,TUNUINO_SOURCE_QUEUE);
    if (tunuino_clock32_observe(&source->clock,counter,time_us) != TUNUINO_CLOCK32_OK)
        return fail(source,TUNUINO_SOURCE_CLOCK);
    return TUNUINO_SOURCE_OK;
}
tunuino_capture_source_status tunuino_capture_source_push(tunuino_capture_source *source,
    uint32_t captured_counter, tunuino_capture_kind kind) {
    tunuino_capture_source_status status = state(source);
    if (status != TUNUINO_SOURCE_OK) return status;
    uint64_t time;
    if (tunuino_clock32_capture(&source->clock,captured_counter,source->max_age_us,&time) != TUNUINO_CLOCK32_OK)
        return fail(source,TUNUINO_SOURCE_CAPTURE);
    if (tunuino_capture_processor_push(&source->processor,(tunuino_capture_event){time,kind}) != TUNUINO_CAPTURE_OK)
        return fail(source,TUNUINO_SOURCE_QUEUE);
    return TUNUINO_SOURCE_OK;
}
tunuino_capture_source_status tunuino_capture_source_lost(tunuino_capture_source *source) {
    tunuino_capture_source_status status = state(source);
    return status == TUNUINO_SOURCE_OK ? fail(source,TUNUINO_SOURCE_LOST) : status;
}
tunuino_capture_source_status tunuino_capture_source_sample(tunuino_capture_source *source,
    uint32_t current_counter, const tunuino_raw_capture *captures, size_t count,
    unsigned events_lost, uint64_t *time_us) {
    if (time_us == NULL) return TUNUINO_SOURCE_ARGUMENT;
    *time_us = 0;
    tunuino_capture_source_status status = state(source);
    if (status != TUNUINO_SOURCE_OK) return status;
    if (events_lost) return tunuino_capture_source_lost(source);
    if (count > 2 || (count != 0 && captures == NULL))
        return fail(source,TUNUINO_SOURCE_ARGUMENT);
    uint64_t now;
    status = tunuino_capture_source_observe(source,current_counter,&now);
    if (status != TUNUINO_SOURCE_OK) return status;
    tunuino_capture_event events[2];
    for (size_t i = 0; i < count; ++i) {
        if ((captures[i].kind != TUNUINO_CAPTURE_CRANK && captures[i].kind != TUNUINO_CAPTURE_CAM) ||
            (i != 0 && captures[i].kind == captures[0].kind))
            return fail(source,TUNUINO_SOURCE_ARGUMENT);
        events[i].kind = captures[i].kind;
        if (tunuino_clock32_capture(&source->clock,captures[i].counter,source->max_age_us,
                &events[i].time_us) != TUNUINO_CLOCK32_OK)
            return fail(source,TUNUINO_SOURCE_CAPTURE);
    }
    if (count == 2 && (events[0].time_us > events[1].time_us ||
            (events[0].time_us == events[1].time_us && events[0].kind == TUNUINO_CAPTURE_CAM))) {
        tunuino_capture_event swap = events[0]; events[0] = events[1]; events[1] = swap;
    }
    for (size_t i = 0; i < count; ++i)
        if (tunuino_capture_processor_push(&source->processor,events[i]) != TUNUINO_CAPTURE_OK)
            return fail(source,TUNUINO_SOURCE_QUEUE);
    *time_us = now;
    return TUNUINO_SOURCE_OK;
}
