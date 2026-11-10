/* Copyright (c) 2026 Tunuino. One selected CAM 1 edge per 720 degrees. */
#ifndef TUNUINO_VVT_H
#define TUNUINO_VVT_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint8_t enabled,channel,direction,oil_required,source[7],table_mode,target_invalid;
    uint8_t advanced,invert,override_dc,override_target;
    int16_t i_min,i_max,output_min,output_max,overridden_target;
    uint16_t overridden_dc,deadband;
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
    TUNUINO_VVT_FEEDBACK,TUNUINO_VVT_RANGE,TUNUINO_VVT_TRACKING,TUNUINO_VVT_LATCHED,TUNUINO_VVT_TARGET};
/* Page 6 extension: v1 bytes 64..122; v2 adds target source at 123; v3 adds overrides/PID limits at 124..141. */
bool tunuino_vvt_decode(const uint8_t page[256],tunuino_vvt_config *out);
bool tunuino_vvt_window(const tunuino_vvt_config *c,uint16_t slots,uint16_t missing,uint16_t first,uint16_t last,uint8_t phase);
void tunuino_vvt_resolve_target(tunuino_vvt_config *c,const uint8_t table[576],bool table_valid,bool map_valid,uint32_t rpm,uint32_t map);
uint16_t tunuino_vvt_pwm_duty(const tunuino_vvt_config *c,const tunuino_vvt_state *s);
void tunuino_vvt_update(const tunuino_vvt_config *c,tunuino_vvt_state *s,uint32_t now,
    bool armed,bool running,uint32_t rpm,bool coolant_valid,int32_t coolant,
    bool oil_valid,uint32_t oil,bool cam_valid,uint64_t snapshot_us,uint64_t edge_us,uint32_t raw_angle);
#endif
