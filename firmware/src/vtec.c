/* Copyright (c) 2026 Tunuino. */
#include "tunuino/vtec.h"
#include "tunuino/vvt.h"
#include <string.h>
static uint16_t u(const uint8_t *p){return (uint16_t)((uint16_t)p[0]<<8|p[1]);}
bool tunuino_vtec_decode(const uint8_t p[256],tunuino_vtec_config *out){
    if(!p||!out)return false;
    tunuino_vtec_config c={0};
    if(!p[0]){for(unsigned i=1;i<256;++i)if(p[i])return false;*out=c;return true;}
    if((p[0]!=1&&p[0]!=2)||p[1]>1||p[2]>16||p[3]>1)return false;
    for(unsigned i=30;i<(p[0]==2?64u:256u);++i)if(p[i])return false;
    if(p[0]==2){tunuino_vvt_config v;if(!tunuino_vvt_decode(p,&v))return false;}
    c.enabled=p[1];c.channel=p[2];c.oil_required=p[3];
    c.rpm_on=u(p+4);c.rpm_off=u(p+6);c.rpm_max=u(p+8);
    c.tps_on=u(p+10);c.tps_off=u(p+12);c.map_on=u(p+14);c.map_off=u(p+16);
    c.clt_min=(int16_t)u(p+18);c.clt_max=(int16_t)u(p+20);
    c.delay_ms=u(p+22);c.oil_on=u(p+24);c.oil_off=u(p+26);
    if(p[28]||p[29]||c.rpm_off<500||c.rpm_off>=c.rpm_on||c.rpm_on>=c.rpm_max||c.rpm_max>20000
        ||c.tps_off>c.tps_on||c.tps_on>10000||c.map_off>c.map_on||c.map_on>50000||c.map_off<1000
        ||c.clt_min<0||c.clt_min>=c.clt_max||c.clt_max>15000||c.delay_ms>60000
        ||c.oil_off>c.oil_on||c.oil_on>20000||(c.oil_required&&!c.oil_off)||(c.enabled&&!c.channel))return false;
    *out=c;return true;
}
void tunuino_vtec_update(const tunuino_vtec_config *c,tunuino_vtec_state *s,
    uint32_t now,bool armed,bool running,uint32_t rpm,bool tv,uint16_t tps,
    bool mv,uint32_t map,bool cv,int32_t clt,bool ov,uint32_t oil){
    if(!c->enabled||!armed){memset(s,0,sizeof(*s));s->reason=c->enabled?TUNUINO_VTEC_DISARMED:TUNUINO_VTEC_DISABLED;return;}
    const bool was_active=s->active!=0;s->active=0;
    uint32_t dt=s->sampled?now-s->last:0;s->last=now;s->sampled=1;
    if(!running){s->elapsed=0;s->sampled=0;s->reason=s->latched?TUNUINO_VTEC_LATCHED:TUNUINO_VTEC_STOPPED;return;}
    // No accumulated activation delay across a stalled update loop.
    if(dt>100000u){s->elapsed=0;if(was_active)s->latched=1;}
    else {uint32_t limit=(uint32_t)c->delay_ms*1000u;s->elapsed=s->elapsed>=limit||dt>=limit-s->elapsed?limit:s->elapsed+dt;}
    if(s->latched){s->reason=TUNUINO_VTEC_LATCHED;return;}
    if(!tv||!mv||!cv||(c->oil_required&&!ov)){
        if(was_active){s->latched=1;}s->reason=TUNUINO_VTEC_SENSOR;return;
    }
    if(c->oil_required&&oil<(uint32_t)(was_active?c->oil_off:c->oil_on)*100u){
        if(was_active){s->latched=1;}s->reason=TUNUINO_VTEC_OIL;return;
    }
    if(clt<(int32_t)c->clt_min*10||clt>(int32_t)c->clt_max*10){s->reason=TUNUINO_VTEC_TEMPERATURE;return;}
    if(s->elapsed<(uint32_t)c->delay_ms*1000u){s->reason=TUNUINO_VTEC_DELAY;return;}
    if(rpm<(uint32_t)(was_active?c->rpm_off:c->rpm_on)*1000u||rpm>=(uint32_t)c->rpm_max*1000u){s->reason=TUNUINO_VTEC_RPM;return;}
    if(tps<(was_active?c->tps_off:c->tps_on)){s->reason=TUNUINO_VTEC_TPS;return;}
    if(map<(uint32_t)(was_active?c->map_off:c->map_on)*10u){s->reason=TUNUINO_VTEC_MAP;return;}
    s->active=1;s->reason=TUNUINO_VTEC_ACTIVE;
}
