/* Copyright (c) 2026 Tunuino. */
#include "tunuino/dispatch.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static tunuino_identity identity = {
    TUNUINO_PROTEUS, TUNUINO_STM32_F4, 1, 0, 1, 0, 3,
    {1,2,3,4,5,6,7,8,9,10,11,12}
};
static tunuino_live_sample live = {1234, 950000, 100000, 1000000, -12345, 15};
static tunuino_map map;
static tunuino_map_snapshot ve, ignition;
static tunuino_dispatch_context context = {&identity, &live, &ve, &ignition,NULL};
static uint8_t request[64], response[300], expected[300];
static size_t request_length, written;
static void query(const uint8_t *body, size_t length) {
    assert(tunuino_frame_encode(body, length, request, sizeof(request), &request_length) == TUNUINO_FRAME_OK);
}
static void text_query(const char *body) { query((const uint8_t *)body, strlen(body)); }
static void map_query(unsigned op, unsigned kind) {
    uint8_t body[19] = {'T','U','N','U','I','N','O',':','M',0,0, 0,0,0,7, 0,0,0,16};
    body[9] = (uint8_t)op; body[10] = (uint8_t)kind;
    query(body, op == 0 ? 11 : 19);
}
static void status(tunuino_dispatch_status desired) {
    written = 999;
    memset(response, 0xA5, sizeof(response));
    assert(tunuino_dispatch_reply(&context, request, request_length, response, sizeof(response), &written) == desired);
    if (desired != TUNUINO_DISPATCH_OK) {
        assert(written == 0);
        for (size_t i = 0; i < sizeof(response); ++i) { assert(response[i] == 0xA5); }
    } else { assert(written > 6); }
}
static void same_as_handler(size_t expected_length) {
    status(TUNUINO_DISPATCH_OK);
    assert(written == expected_length && memcmp(response, expected, written) == 0);
    uint8_t overlap[300];
    memcpy(overlap, request, request_length);
    size_t count;
    assert(tunuino_dispatch_reply(&context, overlap, request_length, overlap, sizeof(overlap), &count) == TUNUINO_DISPATCH_OK);
    assert(count == written && memcmp(overlap, expected, count) == 0);
    for (size_t capacity = 0; capacity < expected_length; ++capacity) {
        count = 99;
        assert(tunuino_dispatch_reply(&context, request, request_length, response, capacity, &count) == TUNUINO_DISPATCH_CAPACITY);
        assert(count == 0);
    }
}
static uint8_t one_byte(void) {
    status(TUNUINO_DISPATCH_OK);
    const uint8_t *body; size_t length;
    assert(tunuino_frame_decode(response, written, &body, &length) == TUNUINO_FRAME_OK && length == 1);
    return body[0];
}
int main(void) {
    assert(tunuino_identity_validate(NULL) == TUNUINO_IDENTITY_ARGUMENT);
    assert(tunuino_dispatch_validate(NULL) == TUNUINO_DISPATCH_ARGUMENT);
    assert(tunuino_dispatch_validate(&context) == TUNUINO_DISPATCH_OK);
    map.rpm_count = map.load_count = 2;
    map.rpm[0] = 1000; map.rpm[1] = 3000;
    map.load_milli_kpa[0] = 50000; map.load_milli_kpa[1] = 100000;
    for (unsigned r = 0; r < 2; ++r) {
        for (unsigned c = 0; c < 2; ++c) { map.cells[r][c] = 10000; }
    }
    map.kind = TUNUINO_MAP_VE;
    assert(tunuino_map_snapshot_prepare(&ve, &map, 7) == TUNUINO_MAP_OK);
    map.kind = TUNUINO_MAP_IGNITION;
    map.cells[0][0] = -10000;
    assert(tunuino_map_snapshot_prepare(&ignition, &map, 7) == TUNUINO_MAP_OK);
    uint32_t ve_crc = ve.crc, ignition_crc = ignition.crc;
    size_t expected_length;
    text_query("TUNUINO?");
    assert(tunuino_identity_reply(&identity, request, request_length, expected, sizeof(expected), &expected_length) == TUNUINO_IDENTITY_OK);
    same_as_handler(expected_length);
    text_query("TUNUINO:L");
    assert(tunuino_live_reply(&live, request, request_length, expected, sizeof(expected), &expected_length) == TUNUINO_LIVE_OK);
    same_as_handler(expected_length);
    for (unsigned kind = 1; kind <= 2; ++kind) {
        for (unsigned op = 0; op <= 1; ++op) {
            map_query(op, kind);
            assert(tunuino_map_read_reply(kind == 1 ? &ve : &ignition, request, request_length,
                expected, sizeof(expected), &expected_length) == TUNUINO_MAP_READ_OK);
            same_as_handler(expected_length);
        }
    }
    assert(tunuino_crc32(ve.bytes, ve.length) == ve_crc && tunuino_crc32(ignition.bytes, ignition.length) == ignition_crc);
    /* A missing kind cannot leak the bytes of the other map, in either direction. */
    context.ignition = NULL;
    map_query(0, 2); assert(one_byte() == 1);
    map_query(1, 2); assert(one_byte() == 1);
    context.ignition = &ignition; context.ve = NULL;
    map_query(0, 1); assert(one_byte() == 1);
    map_query(1, 1); assert(one_byte() == 1);
    context.ve = &ve;
    ve.generation = 8;
    map_query(1, 1); assert(one_byte() == 2);
    ve.generation = 0;
    map_query(0, 1); assert(one_byte() == 1);
    ve.generation = 7;
    live.valid_mask = 0;
    text_query("TUNUINO:L"); assert(one_byte() == 1);
    live.valid_mask = 16; status(TUNUINO_DISPATCH_SOURCE_ERROR);
    live.valid_mask = 15;
    for (unsigned caps = 0; caps <= 3; ++caps) {
        identity.capabilities = caps;
        text_query("TUNUINO?"); status(TUNUINO_DISPATCH_OK);
        text_query("TUNUINO:L"); status((caps & 1) ? TUNUINO_DISPATCH_OK : TUNUINO_DISPATCH_UNSUPPORTED);
        map_query(0, 1); status((caps & 2) ? TUNUINO_DISPATCH_OK : TUNUINO_DISPATCH_UNSUPPORTED);
    }
    for (unsigned caps = 4; caps <= 16; caps *= 2) {
        identity.capabilities = caps;
        text_query("TUNUINO?"); status(TUNUINO_DISPATCH_UNCONFIGURED);
    }
    identity.capabilities = 3;
    context.live = NULL; status(TUNUINO_DISPATCH_UNCONFIGURED); context.live = &live;
    context.ve = context.ignition = NULL; status(TUNUINO_DISPATCH_UNCONFIGURED);
    context.ve = &ignition; context.ignition = &ve; status(TUNUINO_DISPATCH_UNCONFIGURED);
    context.ve = &ve; context.ignition = &ignition;
    identity.hardware_revision = 0; status(TUNUINO_DISPATCH_UNCONFIGURED); identity.hardware_revision = 1;
    identity.uid[0] = 0;
    for (unsigned i = 1; i < 12; ++i) { identity.uid[i] = 0; }
    status(TUNUINO_DISPATCH_UNCONFIGURED);
    for (unsigned i = 0; i < 12; ++i) { identity.uid[i] = (uint8_t)(i + 1); }
    context.identity = NULL; status(TUNUINO_DISPATCH_ARGUMENT); context.identity = &identity;
    for (unsigned length = 1; length <= 10; ++length) {
        uint8_t body[10] = {0}; query(body, length); status(TUNUINO_DISPATCH_UNKNOWN_REQUEST);
    }
    text_query("TUNUINO?extra"); status(TUNUINO_DISPATCH_UNKNOWN_REQUEST);
    text_query("TUNUINO:Lextra"); status(TUNUINO_DISPATCH_UNKNOWN_REQUEST);
    text_query("TUNUINO:M"); status(TUNUINO_DISPATCH_BAD_REQUEST);
    map_query(0, 3); status(TUNUINO_DISPATCH_BAD_REQUEST);
    map_query(2, 1); status(TUNUINO_DISPATCH_BAD_REQUEST);
    map_query(1, 1); request[request_length - 1] ^= 1; status(TUNUINO_DISPATCH_BAD_FRAME);
    text_query("TUNUINO?");
    size_t complete = request_length;
    for (size_t length = 0; length < complete; ++length) {
        request_length = length; status(TUNUINO_DISPATCH_BAD_FRAME);
    }
    request_length = complete; request[request_length++] = 0; status(TUNUINO_DISPATCH_BAD_FRAME);
    written = 99;
    assert(tunuino_dispatch_reply(NULL, request, request_length, response, sizeof(response), &written) == TUNUINO_DISPATCH_ARGUMENT && written == 0);
    assert(tunuino_dispatch_reply(&context, NULL, 0, response, sizeof(response), &written) == TUNUINO_DISPATCH_ARGUMENT);
    assert(tunuino_dispatch_reply(&context, request, request_length, NULL, 0, &written) == TUNUINO_DISPATCH_ARGUMENT);
    assert(tunuino_dispatch_reply(&context, request, request_length, response, sizeof(response), NULL) == TUNUINO_DISPATCH_ARGUMENT);
    puts("C dispatcher routing, capability/provider gates, isolation, overlap and malformed-request checks passed");
    return 0;
}
