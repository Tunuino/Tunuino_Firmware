#include "tunuino/engine_control.h"
#include "tunuino/device.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void put(uint8_t *p,unsigned at,unsigned v){p[at]=(uint8_t)(v>>8);p[at+1]=(uint8_t)v;}
static void extension(uint8_t p[256]){
    memset(p,0,256);p[68]=1;
    const unsigned raw[]={0,2048,4095},temp[]={12000,4000,63536},volts[]={800,1200,1600},map[]={2000,10000,20000};
    for(unsigned i=0;i<3;++i){put(p,76+2*i,raw[i]);put(p,82+2*i,temp[i]);put(p,96+2*i,volts[i]);put(p,102+2*i,1000);put(p,108+2*i,2000);put(p,114+2*i,map[i]);put(p,120+2*i,1000);}
    put(p,90,4095);put(p,94,2000);put(p,134,500);put(p,136,6000);put(p,142,1000);put(p,146,100);put(p,148,2000);put(p,150,1500);put(p,158,7000);put(p,160,6500);put(p,162,4000);put(p,164,3500);put(p,166,2000);put(p,170,4095);put(p,172,10000);put(p,184,11000);put(p,186,800);put(p,188,1600);
}
static tunuino_feature_config c;
static tunuino_feature_state state;
static tunuino_feature_inputs input;
static uint32_t now;
static bool tick(uint32_t dt,uint32_t rpm,uint32_t map){now+=dt;return tunuino_features_update(&c,&state,&input,now,rpm,map,true,80000,400,1000,2000);}
static void reset(void){memset(&state,0,sizeof(state));memset(&input,0,sizeof(input));input.tps_valid=input.lambda_valid=true;input.tps=5000;input.lambda=1200000;now=0;}
static void start(void){assert(tick(0,3000000,150000));assert(tick(100000,3000000,150000));assert(tick(100000,3000000,150000));assert(state.guard_wait&&!state.protection);}
static void output(void *u,unsigned channel,bool active){(void)u;(void)channel;(void)active;}
int main(void){
    uint8_t p[256];extension(p);assert(tunuino_features_decode(p,&c));p[240]=1;put(p,242,1100);put(p,244,100);put(p,246,2500);put(p,248,10000);put(p,250,6000);put(p,252,200);put(p,254,2000);assert(tunuino_features_decode(p,&c));assert(!c.flags);put(p,70,TUNUINO_FEATURE_LAMBDA_GUARD);assert(tunuino_features_decode(p,&c));
    uint8_t valid[256];memcpy(valid,p,256);const unsigned at[]={242,244,246,248,250,252,254};const unsigned invalid[]={2001,19,499,999,12001,60001,10001};
    for(unsigned i=0;i<7;++i){put(p,at[i],invalid[i]);assert(!tunuino_features_decode(p,&c));memcpy(p,valid,256);}
    p[240]=0;assert(!tunuino_features_decode(p,&c));memcpy(p,valid,256);p[241]=1;assert(!tunuino_features_decode(p,&c));assert(tunuino_features_decode(valid,&c));
    /* Full device validation requires an explicit profile and calibrated inputs. */
    tunuino_device d;tunuino_device_init(&d,NULL,NULL,NULL);uint8_t config[TUNUINO_IO_SETTINGS_SIZE];memcpy(config,d.active,sizeof(config));
    config[6]=1;config[7]=2;config[8]=1;put(config,14,1000);put(config,16,25000);put(config,22,500);put(config,24,2000);put(config,64,63536);put(config,66,4000);put(config,68,10000);
    uint8_t *profile=config+128;profile[0]=1;profile[1]=1;put(profile,2,5000);put(profile,4,2000);put(profile,6,1000);put(profile,8,15000);put(profile,12,9000);put(profile,14,8500);
    uint8_t *aux=config+TUNUINO_IO_AUX_OFFSET;memcpy(aux,valid,256);aux[0]=1;aux[5]=3;put(aux,8,4095);put(aux,10,100);put(aux,12,30);put(aux,16,10000);put(aux,20,500);put(aux,22,2000);
    const unsigned temps[]={63536,4000,10000},rpms[]={1000,900,850};for(unsigned i=0;i<3;++i){put(aux,28+2*i,temps[i]);put(aux,34+2*i,rpms[i]);}
    put(aux,46,400);put(aux,50,9500);put(aux,54,10000);put(aux,56,20000);put(aux,58,1000);put(aux,60,1000);put(aux,66,200);
    assert(tunuino_device_validation_errors(config,sizeof(config))==0);aux[5]=0;assert(tunuino_device_validation_errors(config,sizeof(config))&TUNUINO_CONFIG_AUX);aux[5]=3;
    config[7]=0;assert(tunuino_device_validation_errors(config,sizeof(config))&TUNUINO_CONFIG_AUX);config[7]=2;profile[0]=0;assert(tunuino_device_validation_errors(config,sizeof(config))&TUNUINO_CONFIG_AUX);
    reset();start();assert(tick(80000,3000000,150000));assert(!tick(20000,3000000,150000)&&state.protection);
    reset();start();input.lambda=1100000;assert(tick(100000,3000000,150000)&&!state.guard_wait);input.lambda=1200000;assert(tick(20000,3000000,150000));assert(tick(80000,3000000,150000));assert(!tick(20000,3000000,150000));
    /* Out of load/rpm/TPS gates and a stopped engine reset the pending delay. */
    reset();start();assert(tick(100000,2000000,150000)&&!state.guard_wait);assert(tick(100000,3000000,90000)&&!state.guard_wait);input.tps=1000;assert(tick(100000,3000000,150000)&&!state.guard_wait);input.tps=5000;assert(tick(100000,0,150000)&&!state.guard_run);
    reset();start();input.lambda_valid=false;assert(!tick(100000,3000000,150000)&&state.protection);
    reset();start();input.tps_valid=false;assert(!tick(100000,3000000,150000)&&state.protection);
    reset();start();state.inhibit=1;c.transient_delay=1000;assert(tick(100000,3000000,150000)&&!state.guard_wait);state.inhibit=0;
    c.flags|=TUNUINO_FEATURE_LIMIT;c.limit_cut=2800;c.limit_resume=2700;assert(tick(100000,3000000,150000)&&state.fuel_cut&&!state.guard_wait);c.flags=TUNUINO_FEATURE_LAMBDA_GUARD;
    reset();now=UINT32_MAX-150000;start();assert(!tick(100000,3000000,150000)&&state.protection);
    reset();start();/* A long scheduling pause contributes at most 100 ms. */c.guard_delay=500;assert(tick(10000000,3000000,150000)&&state.guard_elapsed==100000);c.guard_delay=100;
    /* Trip through the real engine controller: all outputs stop and the fault
     * remains when later samples recover. No automatic rearming. */
    tunuino_engine_control e;tunuino_engine_control_init(&e,output,NULL);e.configured=1;e.generation=1;e.missing=1;e.profile.operation=1;e.profile.cylinders=1;e.profile.required_fuel_us=1000;e.profile.dead_us=1000;e.profile.dwell_us=2000;e.profile.maximum_injection_us=15000;e.features=c;e.feature_inputs=input;e.feature_inputs.lambda=1200000;
    assert(tunuino_engine_control_arm(&e,19000,1));e.feature_state.sampled=1;e.feature_state.last=0;e.feature_state.guard_running=1;e.feature_state.guard_run=200000;e.feature_state.guard_wait=1;e.feature_state.guard_elapsed=80000;
    tunuino_position_reading r={0};r.crank_valid=true;r.rpm=3000;
    tunuino_engine_control_update(&e,20000,20000,&r,36,1,true,10000,1000,150000,true,80000);
    assert(!e.outputs.armed&&e.outputs.fault==TUNUINO_OUTPUT_PROTECTION&&e.mode==3);
    e.feature_inputs.lambda=1000000;tunuino_engine_control_update(&e,40000,40000,&r,36,1,true,10000,1000,150000,true,80000);assert(!e.outputs.armed&&e.outputs.fault==TUNUINO_OUTPUT_PROTECTION);
    puts("Lambda guard decoding, delay, load gates, sensor faults, clock wrap and retained real-controller stop passed (host simulation)");return 0;
}
