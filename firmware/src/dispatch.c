/* Copyright (c) 2026 Tunuino. */
#include "tunuino/dispatch.h"
#include "tunuino/bench_tunerstudio.h"
#include "tunuino/device.h"
#include <string.h>

static int bench_ts_enabled(const tunuino_identity *id) {
    return id->family == TUNUINO_PROTEUS && id->mcu_series == TUNUINO_STM32_F4
        && id->hardware_revision == 7 && id->capabilities == 0
        && id->firmware_major == TUNUINO_BENCH_MAJOR
        && id->firmware_minor == TUNUINO_BENCH_MINOR
        && id->firmware_patch == TUNUINO_BENCH_PATCH;
}

tunuino_dispatch_status tunuino_dispatch_plain_reply(const tunuino_dispatch_context *c,
    uint8_t command, uint8_t *response, size_t capacity, size_t *written) {
    if (written != NULL) *written = 0;
    if (response == NULL || written == NULL) return TUNUINO_DISPATCH_ARGUMENT;
    tunuino_dispatch_status status = tunuino_dispatch_validate(c);
    if (status != TUNUINO_DISPATCH_OK) return status;
    if (c->device!=NULL) {
        if (command == 'I' || command == 'C') {
            size_t size = command == 'I' ? 1u : 2u;
            if (capacity < size) return TUNUINO_DISPATCH_CAPACITY;
            response[0] = 0; /* Local CAN ID 0 or successful test response. */
            if (command == 'C') response[1] = 0xFF;
            *written = size;
            return TUNUINO_DISPATCH_OK;
        }
        const char *text;
        switch(command) {
            case 'Q':case 'S':text=TUNUINO_IO_SIGNATURE;break;
            case 'V':text=TUNUINO_IO_VERSION;break;
            case 'F':text=TUNUINO_IO_TS_SERIAL_VERSION;break;
            default:return TUNUINO_DISPATCH_UNKNOWN_REQUEST;
        }
        size_t count=command=='F'?3:strlen(text)+1;
        if(capacity<count)return TUNUINO_DISPATCH_CAPACITY;
        memcpy(response,text,count);*written=count;return TUNUINO_DISPATCH_OK;
    }
    if (!bench_ts_enabled(c->identity)) return TUNUINO_DISPATCH_UNKNOWN_REQUEST;
    const char *text;
    size_t length;
    switch (command) {
        case 'Q': case 'S': text = TUNUINO_TS_SIGNATURE; length = sizeof(TUNUINO_TS_SIGNATURE); break;
        case 'V': text = TUNUINO_BENCH_VERSION; length = sizeof(TUNUINO_BENCH_VERSION); break;
        case 'F': text = "001"; length = 3; break;
        default: return TUNUINO_DISPATCH_UNKNOWN_REQUEST;
    }
    if (capacity < length) return TUNUINO_DISPATCH_CAPACITY;
    memcpy(response, text, length);
    *written = length;
    return TUNUINO_DISPATCH_OK;
}

static tunuino_dispatch_status bench_ts_reply(const tunuino_dispatch_context *c,
    const uint8_t *q, size_t length, uint8_t *response, size_t capacity, size_t *written) {
    uint8_t body[1 + sizeof(TUNUINO_TS_SIGNATURE) + sizeof(TUNUINO_BENCH_VERSION)] = {0};
    size_t size = 1;
    if (length == 1 && (q[0] == 'Q' || q[0] == 'S' || q[0] == 'V' || q[0] == 'F')) {
        tunuino_dispatch_status status = tunuino_dispatch_plain_reply(c,q[0],body+1,sizeof(body)-1,&size);
        if (status != TUNUINO_DISPATCH_OK) return status;
        ++size; /* Envelope status byte precedes the response data. */
    } else if (length == 1 && q[0] == 'A') {
        /* Reuse the native identity serializer. After the envelope status byte
         * this is exactly the 31-byte, big-endian INI output-channel block. */
        uint8_t request[14];
        size_t count;
        if (tunuino_frame_encode((const uint8_t *)"TUNUINO?",8,request,sizeof(request),&count) != TUNUINO_FRAME_OK)
            return TUNUINO_DISPATCH_SOURCE_ERROR;
        tunuino_identity_status status = tunuino_identity_reply(c->identity,request,count,response,capacity,written);
        return status == TUNUINO_IDENTITY_OK ? TUNUINO_DISPATCH_OK
            : status == TUNUINO_IDENTITY_CAPACITY ? TUNUINO_DISPATCH_CAPACITY : TUNUINO_DISPATCH_SOURCE_ERROR;
    } else if (length == 1 && q[0] == 'R') {
        body[1] = 1; size = 2; /* One immutable schema byte, not engine calibration. */
    } else {
        body[0] = 0x83; /* MS envelope: unrecognized/unsupported command. No write or burn. */
    }
    return tunuino_frame_encode(body,size,response,capacity,written) == TUNUINO_FRAME_OK
        ? TUNUINO_DISPATCH_OK : TUNUINO_DISPATCH_CAPACITY;
}

tunuino_dispatch_status tunuino_dispatch_validate(const tunuino_dispatch_context *c) {
    if (c == NULL || c->identity == NULL) { return TUNUINO_DISPATCH_ARGUMENT; }
    if (tunuino_identity_validate(c->identity) != TUNUINO_IDENTITY_OK) {
        return TUNUINO_DISPATCH_UNCONFIGURED;
    }
    uint32_t caps = c->identity->capabilities;
    if ((caps & ~(TUNUINO_DISPATCH_LIVE | TUNUINO_DISPATCH_MAP_READ | TUNUINO_DISPATCH_PARTIAL | TUNUINO_DISPATCH_IO)) != 0
        || ((caps & TUNUINO_DISPATCH_LIVE) != 0 && c->live == NULL)) {
        return TUNUINO_DISPATCH_UNCONFIGURED;
    }
    if ((caps & (TUNUINO_DISPATCH_PARTIAL|TUNUINO_DISPATCH_IO))!=0
        && (c->device==NULL || caps!=TUNUINO_IO_CAPABILITIES || c->identity->family!=TUNUINO_PROTEUS
            ||c->identity->mcu_series!=TUNUINO_STM32_F4||c->identity->hardware_revision!=7
            ||c->identity->firmware_major!=0||c->identity->firmware_minor!=2||c->identity->firmware_patch!=0))
        return TUNUINO_DISPATCH_UNCONFIGURED;
    if (c->device!=NULL && caps!=TUNUINO_IO_CAPABILITIES)return TUNUINO_DISPATCH_UNCONFIGURED;
    if ((caps & TUNUINO_DISPATCH_MAP_READ) != 0) {
        if ((c->ve == NULL && c->ignition == NULL)
            || (c->ve != NULL && c->ve->generation != 0 && c->ve->kind != TUNUINO_MAP_VE)
            || (c->ignition != NULL && c->ignition->generation != 0 && c->ignition->kind != TUNUINO_MAP_IGNITION)) {
            return TUNUINO_DISPATCH_UNCONFIGURED;
        }
    }
    return TUNUINO_DISPATCH_OK;
}

tunuino_dispatch_status tunuino_dispatch_reply(const tunuino_dispatch_context *c,
    const uint8_t *request_packet, size_t request_length,
    uint8_t *response_packet, size_t capacity, size_t *written) {
    if (written != NULL) { *written = 0; }
    if (c == NULL || request_packet == NULL || response_packet == NULL || written == NULL) {
        return TUNUINO_DISPATCH_ARGUMENT;
    }
    const uint8_t *q;
    size_t length;
    if (tunuino_frame_decode(request_packet, request_length, &q, &length) != TUNUINO_FRAME_OK) {
        return TUNUINO_DISPATCH_BAD_FRAME;
    }
    tunuino_dispatch_status config = tunuino_dispatch_validate(c);
    if (config != TUNUINO_DISPATCH_OK) { return config; }
    if (length == 8 && memcmp(q, "TUNUINO?", 8) == 0) {
        tunuino_identity_status status = tunuino_identity_reply(c->identity, request_packet,
            request_length, response_packet, capacity, written);
        return status == TUNUINO_IDENTITY_OK ? TUNUINO_DISPATCH_OK
            : status == TUNUINO_IDENTITY_CAPACITY ? TUNUINO_DISPATCH_CAPACITY : TUNUINO_DISPATCH_SOURCE_ERROR;
    }
    if (length==9 && memcmp(q,"TUNUINO:P",9)==0 && (c->identity->capabilities&TUNUINO_DISPATCH_PARTIAL)) {
        tunuino_live_status status=tunuino_live_partial_reply(c->live,response_packet,capacity,written);
        return status==TUNUINO_LIVE_OK?TUNUINO_DISPATCH_OK:status==TUNUINO_LIVE_CAPACITY?TUNUINO_DISPATCH_CAPACITY:TUNUINO_DISPATCH_SOURCE_ERROR;
    }
    if (length == 9 && memcmp(q, "TUNUINO:L", 9) == 0) {
        if ((c->identity->capabilities & TUNUINO_DISPATCH_LIVE) == 0) {
            return TUNUINO_DISPATCH_UNSUPPORTED;
        }
        tunuino_live_status status = tunuino_live_reply(c->live, request_packet,
            request_length, response_packet, capacity, written);
        return status == TUNUINO_LIVE_OK ? TUNUINO_DISPATCH_OK
            : status == TUNUINO_LIVE_CAPACITY ? TUNUINO_DISPATCH_CAPACITY : TUNUINO_DISPATCH_SOURCE_ERROR;
    }
    if (length >= 9 && memcmp(q, "TUNUINO:M", 9) == 0) {
        if ((c->identity->capabilities & TUNUINO_DISPATCH_MAP_READ) == 0) {
            return TUNUINO_DISPATCH_UNSUPPORTED;
        }
        const tunuino_map_snapshot *snapshot = length >= 11 && q[10] == TUNUINO_MAP_IGNITION
            ? c->ignition : c->ve;
        /* A snapshot of the other kind yields NOT_READY for a missing map.
         * The map handler still validates the entire request before replying. */
        if (snapshot == NULL) { snapshot = c->ve != NULL ? c->ve : c->ignition; }
        tunuino_map_read_status status = tunuino_map_read_reply(snapshot, request_packet,
            request_length, response_packet, capacity, written);
        return status == TUNUINO_MAP_READ_OK ? TUNUINO_DISPATCH_OK
            : status == TUNUINO_MAP_READ_CAPACITY ? TUNUINO_DISPATCH_CAPACITY
            : status == TUNUINO_MAP_READ_BAD_REQUEST ? TUNUINO_DISPATCH_BAD_REQUEST : TUNUINO_DISPATCH_SOURCE_ERROR;
    }
    if (c->device!=NULL) return tunuino_device_reply(c->device,c->identity,q,length,response_packet,capacity,written);
    if (bench_ts_enabled(c->identity)) {
        return bench_ts_reply(c,q,length,response_packet,capacity,written);
    }
    return TUNUINO_DISPATCH_UNKNOWN_REQUEST;
}
