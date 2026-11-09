/* Copyright (c) 2026 Tunuino. Discrete cam-profile switching, independent of VVT. */
#ifndef TUNUINO_VTEC_H
#define TUNUINO_VTEC_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint8_t enabled,channel,oil_required;
    uint16_t rpm_on,rpm_off,rpm_max,tps_on,tps_off,map_on,map_off;
    int16_t clt_min,clt_max;
    uint16_t delay_ms,oil_on,oil_off;
} tunuino_vtec_config;
typedef struct {
    uint32_t last,elapsed;
    uint8_t sampled,active,latched,reason;
} tunuino_vtec_state;
enum {TUNUINO_VTEC_ACTIVE=0,TUNUINO_VTEC_DISABLED,TUNUINO_VTEC_DISARMED,
    TUNUINO_VTEC_STOPPED,TUNUINO_VTEC_TEMPERATURE,TUNUINO_VTEC_DELAY,
    TUNUINO_VTEC_RPM,TUNUINO_VTEC_TPS,TUNUINO_VTEC_MAP,
    TUNUINO_VTEC_OIL,TUNUINO_VTEC_SENSOR,TUNUINO_VTEC_LATCHED};
bool tunuino_vtec_decode(const uint8_t page[256],tunuino_vtec_config *out);
/* Oil thresholds are deci-kPa, CLT centi-C, TPS centi-percent, MAP centi-kPa.
 * Required sensor/pressure failure while active latches this function OFF until
 * disarm/rearm. Other engine controls are not armed or modified by this module. */
void tunuino_vtec_update(const tunuino_vtec_config *c,tunuino_vtec_state *s,
    uint32_t now,bool armed,bool running,uint32_t rpm,bool tps_valid,uint16_t tps,
    bool map_valid,uint32_t map,bool clt_valid,int32_t clt,bool oil_valid,uint32_t oil);
#endif
