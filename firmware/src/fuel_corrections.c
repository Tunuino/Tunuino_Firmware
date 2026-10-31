/* Copyright (c) 2026 Tunuino. */
#include "tunuino/fuel_corrections.h"
#include <string.h>
static uint16_t u16(const uint8_t *p){return (uint16_t)((uint16_t)p[0]<<8|p[1]);}
static int16_t s16(const uint8_t *p){uint16_t v=u16(p);return (int16_t)(v<32768?(int32_t)v:(int32_t)v-65536);}
bool tunuino_fuel_corrections_decode(const uint8_t p[25],tunuino_fuel_corrections *out){
    if(!p||!out||p[0]>1)return false;
    tunuino_fuel_corrections c={0};
    if(!p[0]){bool empty=true;for(unsigned i=1;i<25;++i)if(p[i])empty=false;if(empty){*out=c;return true;}}
    c.enabled=p[0];c.cranking_rpm=u16(p+1);c.cranking_percent=u16(p+3);
    c.afterstart_percent=u16(p+5);c.afterstart_ms=u16(p+7);c.cranking_advance=s16(p+21);
    c.fixed_cranking_advance=p[23];
    if(c.cranking_rpm<100||c.cranking_rpm>1000||c.cranking_percent<5000||c.cranking_percent>50000
        ||c.afterstart_percent<10000||c.afterstart_percent>30000||c.afterstart_ms>60000
        ||(c.afterstart_percent!=10000&&!c.afterstart_ms)
        ||c.cranking_advance< -1000||c.cranking_advance>2000||p[23]>1||p[24])return false;
    for(unsigned i=0;i<3;++i){
        c.temperature[i]=s16(p+9+4*i);c.warmup_percent[i]=u16(p+11+4*i);
        if(c.temperature[i]< -4000||c.temperature[i]>15000||c.warmup_percent[i]<5000||c.warmup_percent[i]>50000
            ||(i&&c.temperature[i]<=c.temperature[i-1]))return false;
    }
    *out=c;return true;
}
static uint32_t warmup(const tunuino_fuel_corrections *c,int32_t temperature){
    if(temperature<=(int32_t)c->temperature[0]*10)return c->warmup_percent[0];
    if(temperature>=(int32_t)c->temperature[2]*10)return c->warmup_percent[2];
    unsigned i=temperature<(int32_t)c->temperature[1]*10?0u:1u;
    int32_t first=(int32_t)c->temperature[i]*10,last=(int32_t)c->temperature[i+1]*10;
    uint64_t sum=(uint64_t)c->warmup_percent[i]*(uint32_t)(last-temperature)
        +(uint64_t)c->warmup_percent[i+1]*(uint32_t)(temperature-first);
    return (uint32_t)((sum+(uint32_t)(last-first)/2u)/(uint32_t)(last-first));
}
bool tunuino_fuel_corrections_apply(const tunuino_fuel_corrections *c,tunuino_fuel_correction_state *s,
    uint32_t now,uint32_t rpm,bool clt_valid,int32_t clt,uint32_t *percent,int16_t *advance){
    if(!c||!s||!percent||!advance)return false;
    *percent=10000;
    if(!c->enabled){memset(s,0,sizeof(*s));return true;}
    if(!clt_valid||clt< -40000||clt>150000||!rpm)return false;
    if(rpm<(uint32_t)c->cranking_rpm*1000u){
        s->running=s->afterstart_active=0;*percent=c->cranking_percent;
        if(c->fixed_cranking_advance)*advance=c->cranking_advance;
        return true;
    }
    if(!s->running){s->running=1;s->started_at=now;s->afterstart_active=c->afterstart_ms!=0;}
    uint32_t ase=10000,elapsed=now-s->started_at;
    uint32_t duration=(uint32_t)c->afterstart_ms*1000u;
    if(s->afterstart_active){
        if(elapsed>=duration)s->afterstart_active=0;
        else ase+=(uint32_t)((uint64_t)(c->afterstart_percent-10000u)*(duration-elapsed)/duration);
    }
    *percent=(uint32_t)(((uint64_t)warmup(c,clt)*ase+5000u)/10000u);
    return true;
}
