/* Copyright (c) 2026 Tunuino. */
#include "tunuino/dispatch.h"
#include "tunuino/device.h"
#include <string.h>

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
    return TUNUINO_DISPATCH_UNKNOWN_REQUEST;
}

tunuino_dispatch_status tunuino_dispatch_validate(const tunuino_dispatch_context *c) {
    if (c == NULL || c->identity == NULL) { return TUNUINO_DISPATCH_ARGUMENT; }
    if (tunuino_identity_validate(c->identity) != TUNUINO_IDENTITY_OK) {
        return TUNUINO_DISPATCH_UNCONFIGURED;
    }
    uint32_t caps = c->identity->capabilities;
    if((caps&TUNUINO_DISPATCH_VTEC)&&!(caps&TUNUINO_DISPATCH_AFR_TARGET))return TUNUINO_DISPATCH_UNCONFIGURED;
    uint32_t io_caps=caps&~(TUNUINO_DISPATCH_LAMBDA_GUARD|TUNUINO_DISPATCH_IGNITION_LOCK|TUNUINO_DISPATCH_PRESSURE|TUNUINO_DISPATCH_INPUT_FILTER|TUNUINO_DISPATCH_PRIMARY_INPUT|TUNUINO_DISPATCH_CAM_INPUT|TUNUINO_DISPATCH_VSS|TUNUINO_DISPATCH_CAM2|TUNUINO_DISPATCH_AFR_TARGET|TUNUINO_DISPATCH_VTEC);
    if ((caps & ~(TUNUINO_DISPATCH_LIVE | TUNUINO_DISPATCH_MAP_READ | TUNUINO_DISPATCH_PARTIAL | TUNUINO_DISPATCH_IO | TUNUINO_DISPATCH_SCOPE | TUNUINO_DISPATCH_SERVICE | TUNUINO_DISPATCH_LAMBDA_GUARD | TUNUINO_DISPATCH_IGNITION_LOCK | TUNUINO_DISPATCH_PRESSURE|TUNUINO_DISPATCH_INPUT_FILTER|TUNUINO_DISPATCH_PRIMARY_INPUT|TUNUINO_DISPATCH_CAM_INPUT|TUNUINO_DISPATCH_VSS|TUNUINO_DISPATCH_CAM2|TUNUINO_DISPATCH_AFR_TARGET|TUNUINO_DISPATCH_VTEC)) != 0
        || ((caps & TUNUINO_DISPATCH_LIVE) != 0 && c->live == NULL)) {
        return TUNUINO_DISPATCH_UNCONFIGURED;
    }
    if ((caps & (TUNUINO_DISPATCH_PARTIAL|TUNUINO_DISPATCH_IO))!=0
        && (c->device==NULL || (io_caps!=UINT32_C(15)&&io_caps!=UINT32_C(31)&&io_caps!=UINT32_C(63)) || c->identity->family!=TUNUINO_PROTEUS
            ||c->identity->mcu_series!=TUNUINO_STM32_F4||c->identity->hardware_revision!=7
            ||c->identity->firmware_major!=0||c->identity->firmware_minor!=2||c->identity->firmware_patch!=0))
        return TUNUINO_DISPATCH_UNCONFIGURED;
    if (c->device!=NULL && (io_caps!=UINT32_C(15)&&io_caps!=UINT32_C(31)&&io_caps!=UINT32_C(63)))return TUNUINO_DISPATCH_UNCONFIGURED;
    if((caps&(TUNUINO_DISPATCH_LAMBDA_GUARD|TUNUINO_DISPATCH_IGNITION_LOCK|TUNUINO_DISPATCH_PRESSURE|TUNUINO_DISPATCH_INPUT_FILTER|TUNUINO_DISPATCH_PRIMARY_INPUT|TUNUINO_DISPATCH_CAM_INPUT|TUNUINO_DISPATCH_VSS|TUNUINO_DISPATCH_CAM2|TUNUINO_DISPATCH_AFR_TARGET|TUNUINO_DISPATCH_VTEC))!=0&&(caps&UINT32_C(15))!=UINT32_C(15))return TUNUINO_DISPATCH_UNCONFIGURED;
    if((caps&TUNUINO_DISPATCH_SCOPE)!=0&&(c->device==NULL||c->device->scope_command==NULL))return TUNUINO_DISPATCH_UNCONFIGURED;
    if((caps&TUNUINO_DISPATCH_SERVICE)!=0&&(c->device==NULL||c->device->service_command==NULL))return TUNUINO_DISPATCH_UNCONFIGURED;
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
    return TUNUINO_DISPATCH_UNKNOWN_REQUEST;
}
