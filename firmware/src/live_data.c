/* Copyright (c) 2026 Tunuino. */
#include "tunuino/live_data.h"
#include <string.h>

static void put_value(uint8_t *target, uint64_t value, unsigned bytes) {
    for (unsigned i = 0; i < bytes; ++i) {
        target[i] = (uint8_t)(value >> (8u * (bytes - i - 1u)));
    }
}

tunuino_live_status tunuino_live_partial_reply(const tunuino_live_sample *s,
    uint8_t *response,size_t capacity,size_t *written) {
    if(written!=NULL)*written=0;
    if(s==NULL||response==NULL||written==NULL)return TUNUINO_LIVE_ARGUMENT;
    if((s->valid_mask & ~TUNUINO_LIVE_ALL_VALID)!=0 || s->time_ms>>63)return TUNUINO_LIVE_BAD_SAMPLE;
    uint8_t body[28]={0,2,0,0};body[2]=s->valid_mask;
    put_value(body+4,s->time_ms,8);
    if(s->valid_mask&TUNUINO_LIVE_RPM_VALID)put_value(body+12,s->rpm_milli,4);
    if(s->valid_mask&TUNUINO_LIVE_MAP_VALID)put_value(body+16,s->map_milli_kpa,4);
    if(s->valid_mask&TUNUINO_LIVE_LAMBDA_VALID)put_value(body+20,s->lambda_million,4);
    if(s->valid_mask&TUNUINO_LIVE_COOLANT_VALID)put_value(body+24,(uint32_t)s->coolant_milli_c,4);
    return tunuino_frame_encode(body,sizeof(body),response,capacity,written)==TUNUINO_FRAME_OK?TUNUINO_LIVE_OK:TUNUINO_LIVE_CAPACITY;
}

tunuino_live_status tunuino_live_reply(const tunuino_live_sample *sample,
    const uint8_t *request_packet, size_t request_length,
    uint8_t *response_packet, size_t capacity, size_t *written) {
    if (written != NULL) { *written = 0; }
    if (sample == NULL || request_packet == NULL || response_packet == NULL || written == NULL) {
        return TUNUINO_LIVE_ARGUMENT;
    }
    const uint8_t *request;
    size_t length;
    if (tunuino_frame_decode(request_packet, request_length, &request, &length) != TUNUINO_FRAME_OK) {
        return TUNUINO_LIVE_BAD_FRAME;
    }
    static const uint8_t query[] = "TUNUINO:L";
    if (length != sizeof(query) - 1u || memcmp(query, request, length) != 0) {
        return TUNUINO_LIVE_BAD_REQUEST;
    }
    if ((sample->valid_mask & ~TUNUINO_LIVE_ALL_VALID) != 0 || (sample->time_ms >> 63) != 0) {
        return TUNUINO_LIVE_BAD_SAMPLE;
    }
    uint8_t body[TUNUINO_LIVE_BODY_SIZE] = {0};
    size_t body_size = sizeof(body);
    if (sample->valid_mask != TUNUINO_LIVE_ALL_VALID) {
        body[0] = 1; /* NOT_READY */
        body_size = 1;
    } else {
        body[1] = 1; /* live-data format version */
        body[2] = TUNUINO_LIVE_ALL_VALID;
        put_value(body + 4, sample->time_ms, 8);
        put_value(body + 12, sample->rpm_milli, 4);
        put_value(body + 16, sample->map_milli_kpa, 4);
        put_value(body + 20, sample->lambda_million, 4);
        put_value(body + 24, (uint32_t)sample->coolant_milli_c, 4);
    }
    return tunuino_frame_encode(body, body_size, response_packet, capacity, written) == TUNUINO_FRAME_OK
        ? TUNUINO_LIVE_OK : TUNUINO_LIVE_CAPACITY;
}
