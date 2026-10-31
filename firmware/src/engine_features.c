/* Copyright (c) 2026 Tunuino. Fuel corrections precede injector latency. */
#include "tunuino/engine_features.h"
#include <string.h>
static uint16_t u(const uint8_t *p){return (uint16_t)((uint16_t)p[0]<<8|p[1]);}
static int16_t s(const uint8_t *p){uint16_t n=u(p);return (int16_t)(n<32768?(int32_t)n:(int32_t)n-65536);}
bool tunuino_features_decode(const uint8_t p[256],tunuino_feature_config *out){
    if(!p||!out)return false;
    tunuino_feature_config c={0};
    if(!p[68]){for(unsigned i=69;i<256;++i)if(p[i])return false;*out=c;return true;}
    if(p[68]!=1||p[69])return false;
    for(unsigned i=240;i<256;++i)if(p[i])return false;
    c.flags=u(p+70);c.iat_channel=p[72];c.voltage_channel=p[73];c.launch_channel=p[74];c.flex_channel=p[75];
    if(c.flags>2047||c.iat_channel>4||c.voltage_channel>11||c.launch_channel>11||c.flex_channel>11)return false;
    for(unsigned i=0;i<3;++i){
        c.iat_raw[i]=u(p+76+2*i);c.iat_temperature[i]=s(p+82+2*i);
        c.voltage_axis[i]=u(p+96+2*i);c.deadtime[i]=u(p+102+2*i);c.dwell[i]=u(p+108+2*i);
        c.target_map[i]=u(p+114+2*i);c.target_lambda[i]=u(p+120+2*i);
        if(c.iat_raw[i]>4095||c.iat_temperature[i]<-4000||c.iat_temperature[i]>15000
            ||c.voltage_axis[i]<500||c.voltage_axis[i]>2000||c.deadtime[i]>5000
            ||c.dwell[i]<100||c.dwell[i]>5000||c.target_map[i]<1000||c.target_map[i]>50000
            ||c.target_lambda[i]<650||c.target_lambda[i]>1300)return false;
        if(i&&(c.iat_raw[i]<=c.iat_raw[i-1]||c.voltage_axis[i]<=c.voltage_axis[i-1]||c.target_map[i]<=c.target_map[i-1]))return false;
    }
    if(c.iat_temperature[0]==c.iat_temperature[1]&&c.iat_temperature[1]==c.iat_temperature[2])return false;
    for(unsigned i=0;i<2;++i){c.voltage_raw[i]=u(p+88+2*i);c.voltage_value[i]=u(p+92+2*i);c.flex_raw[i]=u(p+168+2*i);}
    if(c.voltage_raw[0]>=c.voltage_raw[1]||c.voltage_raw[1]>4095||c.voltage_value[0]>=c.voltage_value[1]||c.voltage_value[1]>2400
        ||c.flex_raw[0]>=c.flex_raw[1]||c.flex_raw[1]>4095)return false;
    c.lambda_p=u(p+126);c.lambda_i=u(p+128);c.lambda_limit=u(p+130);c.lambda_clt=s(p+132);
    c.lambda_min_rpm=u(p+134);c.lambda_max_rpm=u(p+136);c.lambda_tps=u(p+138);c.lambda_delay=u(p+140);
    c.accel_rate=u(p+142);c.accel_us=u(p+144);c.accel_ms=u(p+146);
    c.dfco_cut=u(p+148);c.dfco_resume=u(p+150);c.dfco_tps=u(p+152);c.dfco_clt=s(p+154);c.dfco_delay=u(p+156);
    c.limit_cut=u(p+158);c.limit_resume=u(p+160);c.launch_cut=u(p+162);c.launch_resume=u(p+164);c.launch_raw=u(p+166);
    c.flex_fuel=u(p+172);c.flex_advance=s(p+174);c.iat_reference=s(p+176);c.iat_retard_start=s(p+180);c.iat_retard=u(p+182);
    c.transient_delay=u(p+178);
    c.protection_clt=s(p+184);c.protection_min_v=u(p+186);c.protection_max_v=u(p+188);c.launch_advance=s(p+190);
    if(c.lambda_p>20000||c.lambda_i>20000||c.lambda_limit>2500||c.lambda_clt<0||c.lambda_clt>12000
        ||c.lambda_min_rpm<500||c.lambda_min_rpm>=c.lambda_max_rpm||c.lambda_max_rpm>12000||c.lambda_tps>10000||c.lambda_delay>60000
        ||c.accel_rate<100||c.accel_rate>50000||c.accel_us>5000||c.accel_ms<10||c.accel_ms>2000
        ||c.dfco_resume<500||c.dfco_cut<=c.dfco_resume||c.dfco_cut>12000||c.dfco_tps>1000||c.dfco_clt<0||c.dfco_clt>12000||c.dfco_delay>5000
        ||c.limit_resume<500||c.limit_cut<=c.limit_resume||c.limit_cut>12000
        ||c.launch_resume<500||c.launch_cut<=c.launch_resume||c.launch_cut>c.limit_cut||c.launch_raw<100||c.launch_raw>3995
        ||c.flex_fuel<10000||c.flex_fuel>25000||c.flex_advance< -1000||c.flex_advance>1000||c.iat_reference< -4000||c.iat_reference>10000
        ||c.iat_retard_start<0||c.iat_retard_start>12000||c.iat_retard>500||c.protection_clt<8000||c.protection_clt>15000
        ||c.protection_min_v<500||c.protection_max_v>2000||c.protection_min_v>=c.protection_max_v||c.launch_advance< -1000||c.launch_advance>2000||c.transient_delay>5000)return false;
    if((c.flags&TUNUINO_FEATURE_IAT)&&!c.iat_channel)return false;
    if((c.flags&(TUNUINO_FEATURE_VOLTAGE|TUNUINO_FEATURE_PROTECTION))&&!c.voltage_channel)return false;
    if((c.flags&TUNUINO_FEATURE_LAUNCH)&&(!(c.flags&TUNUINO_FEATURE_LIMIT)||!c.launch_channel))return false;
    if((c.flags&TUNUINO_FEATURE_FLEX)&&!c.flex_channel)return false;
    if((c.flags&TUNUINO_FEATURE_LAMBDA)&&!(c.flags&TUNUINO_FEATURE_TARGET))return false;
    for(unsigned i=0;i<12;++i){c.fuel_trim[i]=s(p+192+2*i);c.spark_trim[i]=s(p+216+2*i);
        if(c.fuel_trim[i]<-2500||c.fuel_trim[i]>2500||c.spark_trim[i]<-1000||c.spark_trim[i]>1000)return false;}
    *out=c;return true;
}
static int32_t lerp(int32_t a,int32_t b,int32_t x,int32_t x0,int32_t x1){return a+(int32_t)((int64_t)(b-a)*(x-x0)/(x1-x0));}
static uint16_t curve(const uint16_t x[3],const uint16_t y[3],uint32_t value){
    if(value<=x[0])return y[0];
    if(value>=x[2])return y[2];
    unsigned i=value<x[1]?0u:1u;return (uint16_t)lerp(y[i],y[i+1],(int32_t)value,x[i],x[i+1]);
}
void tunuino_features_sample(const tunuino_feature_config *c,bool adc,const uint16_t raw[16],bool tv,uint16_t tps,bool lv,uint32_t lambda,tunuino_feature_inputs *out){
    memset(out,0,sizeof(*out));out->tps_valid=tv;out->tps=tps;out->lambda_valid=lv;out->lambda=lambda;
    if(!adc||!raw)return;
    if(c->iat_channel){uint16_t n=raw[11+c->iat_channel];
        if(n>=c->iat_raw[0]&&n<=c->iat_raw[2]){unsigned i=n<c->iat_raw[1]?0u:1u;
            out->iat=lerp(c->iat_temperature[i],c->iat_temperature[i+1],n,c->iat_raw[i],c->iat_raw[i+1]);out->iat_valid=true;}}
    if(c->voltage_channel){uint16_t n=raw[c->voltage_channel-1];
        if(n>=c->voltage_raw[0]&&n<=c->voltage_raw[1]){out->voltage=(uint16_t)lerp(c->voltage_value[0],c->voltage_value[1],n,c->voltage_raw[0],c->voltage_raw[1]);out->voltage_valid=true;}}
    if(c->launch_channel){uint16_t n=raw[c->launch_channel-1];out->launch_valid=n>=100&&n<=3995;out->launch=out->launch_valid&&n>=c->launch_raw;}
    if(c->flex_channel){uint16_t n=raw[c->flex_channel-1];if(n>=c->flex_raw[0]&&n<=c->flex_raw[1]){out->ethanol=(uint16_t)lerp(0,10000,n,c->flex_raw[0],c->flex_raw[1]);out->flex_valid=true;}}
}
static uint32_t elapsed(uint32_t old,uint32_t dt,uint32_t maximum){return dt>=maximum-old?maximum:old+dt;}
bool tunuino_features_update(const tunuino_feature_config *c,tunuino_feature_state *s,const tunuino_feature_inputs *in,
    uint32_t now,uint32_t rpm,uint32_t map,bool cv,int32_t clt,uint16_t crank,uint16_t dead,uint16_t dwell){
    uint32_t dt=s->sampled?now-s->last:0;bool sample=!s->sampled||dt>=20000;
    s->multiplier=10000;s->extra_us=0;s->advance_delta=0;s->dead_us=dead;s->dwell_us=dwell;s->fuel_cut=0;s->closed_loop=0;s->protection=0;
    if(!c->flags)return true;
    if((c->flags&TUNUINO_FEATURE_IAT)&&!in->iat_valid)return false;
    if((c->flags&TUNUINO_FEATURE_VOLTAGE)&&!in->voltage_valid)return false;
    if((c->flags&TUNUINO_FEATURE_FLEX)&&!in->flex_valid)return false;
    if((c->flags&(TUNUINO_FEATURE_ACCEL|TUNUINO_FEATURE_DFCO))&&!in->tps_valid)return false;
    if((c->flags&TUNUINO_FEATURE_LAUNCH)&&!in->launch_valid)return false;
    if((c->flags&TUNUINO_FEATURE_PROTECTION)&&(!cv||!in->voltage_valid||clt>=(int32_t)c->protection_clt*10
        ||in->voltage<c->protection_min_v||in->voltage>c->protection_max_v)){s->protection=1;return false;}
    if(c->flags&TUNUINO_FEATURE_IAT){
        s->multiplier=(uint32_t)(27315+c->iat_reference)*10000u/(uint32_t)(27315+in->iat);
        int32_t hot=in->iat-c->iat_retard_start;
        if(hot>0){int32_t retard=(int32_t)((int64_t)hot*c->iat_retard/1000);s->advance_delta=(int16_t)-(retard>1500?1500:retard);}
    }
    if(c->flags&TUNUINO_FEATURE_VOLTAGE){s->dead_us=curve(c->voltage_axis,c->deadtime,in->voltage);s->dwell_us=curve(c->voltage_axis,c->dwell,in->voltage);}
    if(c->flags&TUNUINO_FEATURE_TARGET){s->target=curve(c->target_map,c->target_lambda,map/10u);
        s->multiplier=s->multiplier*1000u/s->target;}
    else s->target=1000;
    if(c->flags&TUNUINO_FEATURE_FLEX){s->multiplier=s->multiplier*(10000u+(uint32_t)(c->flex_fuel-10000u)*in->ethanol/10000u)/10000u;
        s->advance_delta+=(int16_t)((int32_t)c->flex_advance*in->ethanol/10000);}
    if(c->flags&TUNUINO_FEATURE_LIMIT){
        if(rpm>=(uint32_t)c->limit_cut*1000u)s->limited=1;
        else if(rpm<=(uint32_t)c->limit_resume*1000u)s->limited=0;
    }else s->limited=0;
    bool launch=(c->flags&TUNUINO_FEATURE_LAUNCH)&&in->launch;
    if(launch){if(rpm>=(uint32_t)c->launch_cut*1000u)s->launch_limited=1;else if(rpm<=(uint32_t)c->launch_resume*1000u)s->launch_limited=0;}
    else s->launch_limited=0;
    bool running=rpm>=(uint32_t)crank*1000u;
    if(sample){
        uint32_t step=dt>100000?100000:dt; /* No accumulated control jump after a stall. */
        if(!running){s->running=s->run_delay_done=0;s->run_elapsed=0;s->accel_active=s->dfco_wait=s->dfco=0;}
        else if(!s->running){s->running=1;s->run_elapsed=0;}
        else if(!s->run_delay_done){s->run_elapsed=elapsed(s->run_elapsed,step,(uint32_t)c->lambda_delay*1000u);s->run_delay_done=s->run_elapsed>=(uint32_t)c->lambda_delay*1000u;}
        if(s->accel_active){s->accel_elapsed=elapsed(s->accel_elapsed,step,(uint32_t)c->accel_ms*1000u);if(s->accel_elapsed>=(uint32_t)c->accel_ms*1000u)s->accel_active=0;}
        if((c->flags&TUNUINO_FEATURE_ACCEL)&&running&&in->tps_valid&&s->sampled&&dt<=100000&&dt>0
            &&in->tps>s->previous_tps&&(uint64_t)(in->tps-s->previous_tps)*1000000u>=(uint64_t)c->accel_rate*dt){s->accel_active=1;s->accel_elapsed=0;}
        s->previous_tps=in->tps;
        bool dfco=(c->flags&TUNUINO_FEATURE_DFCO)&&running&&cv&&clt>=(int32_t)c->dfco_clt*10&&in->tps_valid&&in->tps<=c->dfco_tps;
        if(!dfco||rpm<=(uint32_t)c->dfco_resume*1000u){s->dfco=s->dfco_wait=0;s->dfco_elapsed=0;}
        else if(!s->dfco&&rpm>=(uint32_t)c->dfco_cut*1000u){
            if(!s->dfco_wait){s->dfco_wait=1;s->dfco_elapsed=0;}
            else s->dfco_elapsed=elapsed(s->dfco_elapsed,step,(uint32_t)c->dfco_delay*1000u);
            if(s->dfco_elapsed>=(uint32_t)c->dfco_delay*1000u)s->dfco=1;
        }else if(!s->dfco){s->dfco_wait=0;s->dfco_elapsed=0;}
        s->sampled=1;s->last=now;
    }
    s->fuel_cut=s->limited||s->launch_limited||s->dfco;
    if(s->fuel_cut||s->accel_active||launch){s->inhibit=1;s->quiet_elapsed=0;}
    else if(s->inhibit&&sample){s->quiet_elapsed=elapsed(s->quiet_elapsed,dt>100000?100000:dt,(uint32_t)c->transient_delay*1000u);if(s->quiet_elapsed>=(uint32_t)c->transient_delay*1000u)s->inhibit=0;}
    if(s->accel_active&&!s->fuel_cut)s->extra_us=(uint16_t)((uint64_t)c->accel_us*((uint32_t)c->accel_ms*1000u-s->accel_elapsed)/((uint32_t)c->accel_ms*1000u));
    bool closed=(c->flags&TUNUINO_FEATURE_LAMBDA)&&s->run_delay_done&&running&&cv&&clt>=(int32_t)c->lambda_clt*10
        &&in->tps_valid&&in->tps<=c->lambda_tps&&rpm>=(uint32_t)c->lambda_min_rpm*1000u&&rpm<=(uint32_t)c->lambda_max_rpm*1000u
        &&in->lambda_valid&&in->lambda>=500000&&in->lambda<=2000000&&!s->fuel_cut&&!s->accel_active&&!launch&&!s->inhibit;
    if(!closed){s->integral=0;s->lambda_trim=0;}
    else if(sample){
        int32_t error=(int32_t)(in->lambda/1000u)-s->target;
        int64_t change=(int64_t)c->lambda_i*error*(dt>100000?100000:dt)/1000;
        int64_t bound=(int64_t)c->lambda_limit*1000000;
        int64_t candidate=s->integral+change;if(candidate>bound)candidate=bound;if(candidate< -bound)candidate= -bound;
        int32_t p=(int32_t)c->lambda_p*error/1000;
        int64_t v=p+candidate/1000000;
        if(!((v>c->lambda_limit&&change>0)||(v< -(int32_t)c->lambda_limit&&change<0)))s->integral=candidate;
        int64_t trim=p+s->integral/1000000;s->lambda_trim=(int32_t)(trim>c->lambda_limit?c->lambda_limit:trim< -(int32_t)c->lambda_limit?-(int32_t)c->lambda_limit:trim);
    }
    s->closed_loop=closed;s->multiplier=s->multiplier*(uint32_t)(10000+s->lambda_trim)/10000u;
    return true;
}
