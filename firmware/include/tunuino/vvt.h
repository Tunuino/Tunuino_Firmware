/* Copyright (c) 2026 Tunuino. One selected CAM 1 edge per 720 degrees. */
#ifndef TUNUINO_VVT_H
#define TUNUINO_VVT_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint8_t enabled,channel,direction,oil_required,source[7];
    uint16_t hz,min_duty,max_duty,steady,kp,ki,kd;
    uint32_t reference;
    int16_t minimum,maximum,target,clt_min;
    uint16_t rpm_min,rpm_hysteresis,clt_hysteresis,delay_ms,max_age_ms,tracking_ms,tolerance,oil_on,oil_off;
} tunuino_vvt_config;
typedef struct {
    uint32_t last,elapsed,tracking;
    uint64_t edge;
    int64_t integral;
    int16_t angle,previous;
    uint16_t duty;
    uint8_t sampled,active,latched,reason,feedback;
} tunuino_vvt_state;
enum {TUNUINO_VVT_ACTIVE=0,TUNUINO_VVT_DISABLED,TUNUINO_VVT_DISARMED,TUNUINO_VVT_STOPPED,
    TUNUINO_VVT_RPM,TUNUINO_VVT_TEMPERATURE,TUNUINO_VVT_DELAY,TUNUINO_VVT_OIL,
    TUNUINO_VVT_FEEDBACK,TUNUINO_VVT_RANGE,TUNUINO_VVT_TRACKING,TUNUINO_VVT_LATCHED};
/* Page 6 extension: bytes 64..122, reserved tail remains zero. */
bool tunuino_vvt_decode(const uint8_t page[256],tunuino_vvt_config *out);
bool tunuino_vvt_window(const tunuino_vvt_config *c,uint16_t slots,uint16_t missing,uint16_t first,uint16_t last,uint8_t phase);
void tunuino_vvt_update(const tunuino_vvt_config *c,tunuino_vvt_state *s,uint32_t now,
    bool armed,bool running,uint32_t rpm,bool coolant_valid,int32_t coolant,
    bool oil_valid,uint32_t oil,bool cam_valid,uint64_t snapshot_us,uint64_t edge_us,uint32_t raw_angle);
#endif
