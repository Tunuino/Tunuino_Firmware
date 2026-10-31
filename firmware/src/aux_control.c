/* Copyright (c) 2026 Tunuino. */
#include "tunuino/aux_control.h"
#include "tunuino/engine_features.h"
#include <string.h>
static uint16_t u16(const uint8_t *p){return (uint16_t)((uint16_t)p[0]<<8|p[1]);}
static int16_t s16(const uint8_t *p){uint16_t n=u16(p);return (int16_t)(n<32768?(int32_t)n:(int32_t)n-65536);}
bool tunuino_aux_decode(const uint8_t p[256],uint32_t used,bool have_profile,tunuino_aux_config *out){
    if(!p||!out||p[0]>1)return false;
    tunuino_aux_config c={0};
    if(!p[0]){for(unsigned i=1;i<256;++i)if(p[i])return false;*out=c;return true;}
    tunuino_feature_config features;if(!tunuino_features_decode(p,&features))return false;
    c.idle_mode=p[1];c.idle_channel=p[2];c.boost_mode=p[3];c.boost_channel=p[4];c.tps_channel=p[5];
    c.tps_closed=u16(p+6);c.tps_open=u16(p+8);c.idle_hz=u16(p+10);c.boost_hz=u16(p+12);
    c.idle_min=u16(p+14);c.idle_max=u16(p+16);c.cranking_duty=u16(p+18);c.idle_tps=u16(p+20);c.idle_max_rpm=u16(p+22);
    c.idle_p=u16(p+24);c.idle_i=u16(p+26);c.cranking_rpm=u16(p+46);
    c.boost_min=u16(p+48);c.boost_max=u16(p+50);c.boost_base=u16(p+52);c.boost_target=u16(p+54);
    c.boost_cut=u16(p+56);c.boost_min_rpm=u16(p+58);c.boost_tps=u16(p+60);c.boost_p=u16(p+62);c.boost_i=u16(p+64);
    c.tps_tolerance=u16(p+66);
    if(c.idle_mode>2||c.boost_mode>2||c.idle_channel>16||c.boost_channel>16||c.tps_channel>11
        ||c.tps_closed>4095||c.tps_open>4095||c.tps_closed==c.tps_open
        ||c.idle_hz<10||c.idle_hz>300||c.boost_hz<10||c.boost_hz>100
        ||c.idle_min>c.idle_max||c.idle_max>10000||c.cranking_duty<c.idle_min||c.cranking_duty>c.idle_max||c.idle_tps>2000
        ||c.idle_max_rpm<500||c.idle_max_rpm>3000||c.idle_p>5000||c.idle_i>5000
        ||c.cranking_rpm<100||c.cranking_rpm>1000
        ||c.boost_min>c.boost_max||c.boost_max>9500||c.boost_base<c.boost_min||c.boost_base>c.boost_max
        ||c.boost_target<10000||c.boost_target>=c.boost_cut||c.boost_cut>50000
        ||c.boost_min_rpm<500||c.boost_min_rpm>8000||c.boost_tps>10000||c.boost_p>5000||c.boost_i>5000||c.tps_tolerance>1000)return false;
    for(unsigned i=0;i<3;++i){
        c.temperature[i]=s16(p+28+2*i);c.target_rpm[i]=u16(p+34+2*i);c.idle_base[i]=u16(p+40+2*i);
        if(c.temperature[i]< -4000||c.temperature[i]>15000||(i&&c.temperature[i]<=c.temperature[i-1])
            ||c.target_rpm[i]<500||c.target_rpm[i]>2000||c.target_rpm[i]>=c.idle_max_rpm||c.target_rpm[i]<=c.cranking_rpm
            ||c.idle_base[i]<c.idle_min||c.idle_base[i]>c.idle_max)return false;
    }
    const uint8_t modes[2]={c.idle_mode,c.boost_mode},channels[2]={c.idle_channel,c.boost_channel};
    for(unsigned i=0;i<2;++i)if(modes[i]){
        if(!have_profile||!c.tps_channel||!channels[i]||(used&(UINT32_C(1)<<(channels[i]-1))))return false;
        used|=UINT32_C(1)<<(channels[i]-1);
    }
    *out=c;return true;
}
bool tunuino_aux_tps(const tunuino_aux_config *c,bool adc,const uint16_t raw[16],uint16_t *percent){
    if(!c||!adc||!raw||!percent||!c->tps_channel)return false;
    int32_t value=raw[c->tps_channel-1],low=c->tps_closed,high=c->tps_open;
    int32_t minimum=low<high?low:high,maximum=low>high?low:high;
    int32_t tolerance=(maximum-minimum)*c->tps_tolerance/10000;
    if(value<0||value>4095||value<minimum-tolerance||value>maximum+tolerance)return false;
    if(value<minimum)value=minimum;
    if(value>maximum)value=maximum;
    *percent=(uint16_t)((value-low)*10000/(high-low));return true;
}
void tunuino_aux_reset(tunuino_aux_control *c){
    memset(&c->idle,0,sizeof(c->idle));memset(&c->boost,0,sizeof(c->boost));
    c->idle_duty=c->boost_duty=c->idle_target=c->tps=0;c->tps_valid=c->idle_active=c->boost_active=0;
}
static uint16_t curve(const int16_t x[3],const uint16_t y[3],int32_t input){
    if(input<=(int32_t)x[0]*10)return y[0];
    if(input>=(int32_t)x[2]*10)return y[2];
    unsigned i=input<(int32_t)x[1]*10?0u:1u;int32_t a=(int32_t)x[i]*10,b=(int32_t)x[i+1]*10;
    return (uint16_t)(((uint64_t)y[i]*(uint32_t)(b-input)+(uint64_t)y[i+1]*(uint32_t)(input-a)+(uint32_t)(b-a)/2u)/(uint32_t)(b-a));
}
static uint16_t pi(tunuino_pi_state *s,uint32_t now,int32_t error,unsigned denominator,
    uint16_t base,uint16_t kp,uint16_t ki,uint16_t min,uint16_t max){
    uint32_t dt=s->active?now-s->updated:0;
    if(s->active&&dt<20000)return UINT16_MAX;
    if(dt>100000)dt=100000;
    s->active=1;s->updated=now;
    int64_t p=(int64_t)kp*error/denominator;
    int64_t delta=(int64_t)ki*error*dt/denominator;
    int64_t candidate=s->integral+delta;
    if(candidate>INT64_C(10000000000))candidate=INT64_C(10000000000);
    if(candidate< -INT64_C(10000000000))candidate= -INT64_C(10000000000);
    int64_t output=(int64_t)base+p+candidate/1000000;
    if(!((output>max&&delta>0)||(output<min&&delta<0)))s->integral=candidate;
    output=(int64_t)base+p+s->integral/1000000;
    return (uint16_t)(output<min?min:output>max?max:output);
}
bool tunuino_aux_update(tunuino_aux_control *a,uint32_t now,bool armed,bool running,uint32_t rpm,
    bool tps_valid,uint16_t tps,bool clt_valid,int32_t clt,bool map_valid,uint32_t map){
    const tunuino_aux_config *c=&a->config;
    if(!armed){tunuino_aux_reset(a);return true;}
    a->tps_valid=tps_valid;a->tps=tps_valid?tps:0;
    if(c->boost_mode&&map_valid&&map>=(uint32_t)c->boost_cut*10u){tunuino_aux_reset(a);return false;}
    bool idle=c->idle_mode&&tps_valid&&clt_valid&&clt>= -40000&&clt<=150000&&tps<=c->idle_tps&&rpm<(uint32_t)c->idle_max_rpm*1000u;
    a->idle_active=idle;
    if(!idle){memset(&a->idle,0,sizeof(a->idle));a->idle_duty=a->idle_target=0;}
    else{
        a->idle_target=curve(c->temperature,c->target_rpm,clt);
        uint16_t base=curve(c->temperature,c->idle_base,clt);
        if(!running||rpm<(uint32_t)c->cranking_rpm*1000u){a->idle_duty=c->cranking_duty;memset(&a->idle,0,sizeof(a->idle));}
        else if(c->idle_mode==1){a->idle_duty=base;memset(&a->idle,0,sizeof(a->idle));}
        else{uint16_t duty=pi(&a->idle,now,(int32_t)a->idle_target-(int32_t)(rpm/1000u),100,base,c->idle_p,c->idle_i,c->idle_min,c->idle_max);if(duty!=UINT16_MAX)a->idle_duty=duty;}
    }
    bool boost=c->boost_mode&&running&&tps_valid&&map_valid&&tps>=c->boost_tps&&rpm>=(uint32_t)c->boost_min_rpm*1000u;
    a->boost_active=boost;
    if(!boost){memset(&a->boost,0,sizeof(a->boost));a->boost_duty=0;}
    else if(c->boost_mode==1){a->boost_duty=c->boost_base;memset(&a->boost,0,sizeof(a->boost));}
    else{uint16_t duty=pi(&a->boost,now,(int32_t)c->boost_target-(int32_t)(map/10u),1000,c->boost_base,c->boost_p,c->boost_i,c->boost_min,c->boost_max);if(duty!=UINT16_MAX)a->boost_duty=duty;}
    return true;
}
