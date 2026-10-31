/* Copyright (c) 2026 Tunuino. */
#include "tunuino/capture_processor.h"
#include <stddef.h>
#include <string.h>

static tunuino_capture_process_status fail(tunuino_capture_processor *p, tunuino_capture_process_status state) {
    memset(&p->cached,0,sizeof(p->cached));
    p->state = state;
    return state;
}
static tunuino_capture_process_status check(tunuino_capture_processor *p, uint64_t now) {
    if (p == NULL || !p->configured) { return TUNUINO_PROCESS_ARGUMENT; }
    if (p->state == TUNUINO_PROCESS_QUEUE_FAULT || p->state == TUNUINO_PROCESS_CLOCK || p->state == TUNUINO_PROCESS_ARGUMENT) {
        return p->state;
    }
    if (now < p->last_wall_us) { return fail(p,TUNUINO_PROCESS_CLOCK); }
    p->last_wall_us = now;
    p->queue_status = tunuino_capture_state(&p->queue);
    if (p->queue_status != TUNUINO_CAPTURE_OK) { return fail(p,TUNUINO_PROCESS_QUEUE_FAULT); }
    return TUNUINO_PROCESS_READY;
}
static tunuino_capture_process_status refresh(tunuino_capture_processor *p, uint64_t now) {
    tunuino_capture_process_status status = check(p,now);
    if (status != TUNUINO_PROCESS_READY) { return status; }
    if (p->have_deferred || tunuino_capture_count(&p->queue) != 0) {
        p->state = p->have_deferred && p->deferred.time_us > now ? TUNUINO_PROCESS_FUTURE : TUNUINO_PROCESS_BACKLOG;
    } else if (!p->have_crank || now - p->last_crank_us >= p->config.crank.timeout_us) {
        if (p->have_crank && !p->expired) {
            /* Do not advance decoder time to wall time: an ISR may publish a
             * capture older than this read, but newer than the last event. */
            p->decoder_status = tunuino_position_init(&p->position,&p->config,p->last_event_us);
            memset(&p->cached,0,sizeof(p->cached));
            p->expired = 1;
        }
        p->state = TUNUINO_PROCESS_NO_SIGNAL;
    } else {
        p->state = TUNUINO_PROCESS_READY;
    }
    return p->state;
}
tunuino_capture_process_status tunuino_capture_processor_init(tunuino_capture_processor *p,
    const tunuino_position_config *config, uint64_t now) {
    if (p == NULL) { return TUNUINO_PROCESS_ARGUMENT; }
    /* No concurrent producer/consumer allowed during initialization. */
    p->configured = 0; p->state = TUNUINO_PROCESS_ARGUMENT;
    memset(&p->cached,0,sizeof(p->cached));
    if (config == NULL) { return p->state; }
    p->decoder_status = tunuino_position_init(&p->position,config,now);
    if (p->decoder_status != TUNUINO_POSITION_ACQUIRING) { return p->state; }
    p->config = *config;
    p->queue_status = tunuino_capture_init(&p->queue);
    p->last_wall_us = p->last_event_us = now; p->last_crank_us = 0;
    p->have_crank = p->have_deferred = p->expired = 0;
    p->configured = 1; p->state = TUNUINO_PROCESS_NO_SIGNAL;
    return p->state;
}
tunuino_capture_status tunuino_capture_processor_push(tunuino_capture_processor *p, tunuino_capture_event event) {
    if (p == NULL || !p->configured) { return TUNUINO_CAPTURE_ARGUMENT; }
    return tunuino_capture_push(&p->queue,event);
}
tunuino_capture_process_status tunuino_capture_processor_drain(tunuino_capture_processor *p,
    uint64_t now, size_t budget, size_t *processed) {
    if (processed != NULL) { *processed = 0; }
    if (p == NULL || processed == NULL || budget == 0 || budget > TUNUINO_CAPTURE_CAPACITY) { return TUNUINO_PROCESS_ARGUMENT; }
    tunuino_capture_process_status status = check(p,now);
    if (status != TUNUINO_PROCESS_READY) { return status; }
    while (*processed < budget) {
        tunuino_capture_event event;
        if (p->have_deferred) {
            event = p->deferred;
        } else {
            p->queue_status = tunuino_capture_take(&p->queue,&event);
            if (p->queue_status == TUNUINO_CAPTURE_EMPTY) { break; }
            if (p->queue_status != TUNUINO_CAPTURE_OK) { return fail(p,TUNUINO_PROCESS_QUEUE_FAULT); }
        }
        if (event.time_us > now) {
            p->deferred = event; p->have_deferred = 1;
            break;
        }
        p->have_deferred = 0; p->last_event_us = event.time_us;
        if (event.kind == TUNUINO_CAPTURE_CRANK) {
            p->decoder_status = tunuino_position_crank_edge(&p->position,event.time_us);
            p->last_crank_us = event.time_us; p->have_crank = 1; p->expired = 0;
        } else {
            p->decoder_status = tunuino_position_cam_edge(&p->position,event.time_us);
        }
        ++*processed;
        p->decoder_status = tunuino_position_read(&p->position,event.time_us,&p->cached);
        (void)tunuino_position_cam_read(&p->position,event.time_us,&p->cached_cam);
        if (p->decoder_status == TUNUINO_POSITION_CLOCK) { return fail(p,TUNUINO_PROCESS_CLOCK); }
        if (p->decoder_status == TUNUINO_POSITION_ARGUMENT) { return fail(p,TUNUINO_PROCESS_ARGUMENT); }
    }
    return refresh(p,now);
}
tunuino_capture_process_status tunuino_capture_processor_read(tunuino_capture_processor *p,
    uint64_t now, tunuino_position_reading *reading) {
    if (reading == NULL) { return TUNUINO_PROCESS_ARGUMENT; }
    memset(reading,0,sizeof(*reading));
    tunuino_capture_process_status status = refresh(p,now);
    if (status == TUNUINO_PROCESS_READY) {
        /* Recheck a producer fault before exposing the cached snapshot. */
        p->queue_status = tunuino_capture_state(&p->queue);
        if (p->queue_status != TUNUINO_CAPTURE_OK) { return fail(p,TUNUINO_PROCESS_QUEUE_FAULT); }
        *reading = p->cached;
    }
    return status;
}
tunuino_capture_process_status tunuino_capture_processor_anchor(tunuino_capture_processor *p,
    uint64_t now,tunuino_position_reading *reading,uint64_t *edge){
    if(!edge)return TUNUINO_PROCESS_ARGUMENT;
    *edge=0;
    tunuino_capture_process_status status=tunuino_capture_processor_read(p,now,reading);
    if(status==TUNUINO_PROCESS_READY&&reading->crank_valid)*edge=p->last_crank_us;
    return status;
}
tunuino_capture_process_status tunuino_capture_processor_cam(tunuino_capture_processor *p,
    uint64_t now,tunuino_cam_reading *reading) {
    if (!reading) return TUNUINO_PROCESS_ARGUMENT;
    memset(reading,0,sizeof(*reading));
    tunuino_position_reading position;
    tunuino_capture_process_status status=tunuino_capture_processor_read(p,now,&position);
    if (status==TUNUINO_PROCESS_READY && position.cycle_valid) *reading=p->cached_cam;
    return status;
}
