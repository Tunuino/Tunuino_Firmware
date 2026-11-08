/* Copyright (c) 2026 Tunuino. Single digital VSS acquisition, no actuation. */
#ifndef TUNUINO_VSS_H
#define TUNUINO_VSS_H
#include "tunuino/primary_input.h"
#include <string.h>
#define TUNUINO_VSS_OFFSET 82u
#define TUNUINO_VSS_SIZE 14u
static inline uint16_t tunuino_vss_u16(const uint8_t *p){return (uint16_t)((uint16_t)p[0]<<8|p[1]);}
static inline uint32_t tunuino_vss_u32(const uint8_t *p){return (uint32_t)tunuino_vss_u16(p)<<16|tunuino_vss_u16(p+2);}
/* V0 all zero disabled. V1: version, input (0 disabled/1 digital1), edge,
 * blanking level, U32 milli-km/h per Hz, U16 timeout ms, four reserved zeros. */
static inline bool tunuino_vss_validate(const uint8_t *p){
 if(!p)return false;
 bool empty=true;for(unsigned i=0;i<14;++i)if(p[i])empty=false;
 if(empty)return true;
 if(p[0]!=1||p[1]>1||p[2]>1||p[3]>3||tunuino_vss_u32(p+4)>1000000||tunuino_vss_u16(p+8)<100||tunuino_vss_u16(p+8)>5000)return false;
 for(unsigned i=10;i<14;++i)if(p[i])return false;
 return true;
}
typedef struct {tunuino_primary_input filter;uint32_t count,period;} tunuino_vss_capture;
typedef struct {uint64_t edge_us,last_us;uint32_t count;bool have_edge,paired;} tunuino_vss_tracker;
typedef struct {uint32_t frequency_milli,speed_milli,period_us,age_ms,count;uint8_t valid,reason;} tunuino_vss_reading;
static inline void tunuino_vss_reset(tunuino_vss_capture *c,tunuino_vss_tracker *s,unsigned level){memset(c,0,sizeof(*c));memset(s,0,sizeof(*s));tunuino_primary_input_reset(&c->filter,(uint8_t)level);}
/* ISR producer only; caller snapshots under the same interrupt lock. */
static inline void tunuino_vss_edge(tunuino_vss_capture *c,uint32_t counter){
 const bool previous=c->filter.ready;const uint32_t last=c->filter.last;
 if(!tunuino_primary_input_accept(&c->filter,counter,true))return;
 c->period=previous?(uint32_t)(counter-last):0;++c->count;
}
/* Reasons: 0 OK, 1 disabled, 2 waiting for pair, 3 stale, 4 frequency/range,
 * 5 backwards clock, 7 frequency available but speed ratio uncalibrated. Counter wrap cannot revive an expired signal. */
static inline tunuino_vss_reading tunuino_vss_sample(tunuino_vss_tracker *s,const tunuino_vss_capture *c,const uint8_t *p,uint64_t now){
 tunuino_vss_reading r={0};r.reason=1;
 if(!tunuino_vss_validate(p)||p[0]!=1||!p[1])return r;
 if(now<s->last_us){memset(s,0,sizeof(*s));r.reason=5;return r;}s->last_us=now;r.count=c->count;r.reason=2;
 const uint64_t timeout=(uint64_t)tunuino_vss_u16(p+8)*1000u;
 if(c->count!=s->count){const uint32_t increment=c->count-s->count,age=(uint32_t)((uint32_t)now-c->filter.last);s->count=c->count;
  if(age>now||age>timeout){s->have_edge=false;r.reason=3;return r;}
  const uint64_t edge=now-age;const bool pair=increment>=2||(s->have_edge&&edge>=s->edge_us&&edge-s->edge_us<=timeout);
  s->edge_us=edge;s->have_edge=true;s->paired=pair;if(!pair)return r;
 }else if(!s->have_edge||!s->paired)return r;
 const uint64_t age=now-s->edge_us;r.age_ms=age/1000u>UINT32_MAX?UINT32_MAX:(uint32_t)(age/1000u);
 if(age>timeout){r.reason=3;return r;}
 if(c->period<100||c->period>timeout){r.reason=4;return r;}
 r.frequency_milli=UINT32_C(1000000000)/c->period;r.period_us=c->period;r.valid=1;
 if(!tunuino_vss_u32(p+4)){r.reason=7;return r;}
 const uint64_t speed=(uint64_t)r.frequency_milli*tunuino_vss_u32(p+4)/1000u;
 if(speed>500000u){r.frequency_milli=0;r.valid=0;r.reason=4;return r;}
 r.speed_milli=(uint32_t)speed;r.period_us=c->period;r.valid=3;r.reason=0;return r;
}
#endif
