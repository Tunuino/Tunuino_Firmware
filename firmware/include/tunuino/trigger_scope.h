/* Copyright (c) 2026 Tunuino. Read-only trigger edge capture. */
#ifndef TUNUINO_TRIGGER_SCOPE_H
#define TUNUINO_TRIGGER_SCOPE_H
#include "tunuino/capture_queue.h"
#include <stdbool.h>
#define TUNUINO_SCOPE_CAPACITY 256u
typedef struct {
    tunuino_capture_event ring[TUNUINO_SCOPE_CAPACITY],frozen[TUNUINO_SCOPE_CAPACITY];
    unsigned next,count,frozen_count;
    uint32_t token,generation;
    bool valid;
} tunuino_trigger_scope;
/* Zero-initialize once before the first reset; all access is consumer-thread only. */
void tunuino_scope_reset(tunuino_trigger_scope *scope,uint32_t generation);
void tunuino_scope_record(void *scope,tunuino_capture_event event);
/* op0: snapshot; op1 + token U32 + offset U16 + count U8: frozen data.
 * Reply: status, format, valid, total U16, generation U32, token U32,
 * time origin U64, then count*(relative_us U32, edge kind U8). */
bool tunuino_scope_reply(tunuino_trigger_scope *scope,bool healthy,
    const uint8_t *request,size_t length,uint8_t *out,size_t capacity,size_t *written);
#endif
