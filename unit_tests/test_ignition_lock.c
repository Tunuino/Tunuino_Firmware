#include "tunuino/device.h"
#include "tunuino/engine_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static bool high;static unsigned falls;static uint32_t falling[8],clock_us;
static void pin(void *user,unsigned channel,bool active){(void)user;if(channel!=16)return;if(high&&!active&&falls<8)falling[falls++]=clock_us;high=active;}
static void prepare(tunuino_engine_control *e){
    tunuino_engine_control_init(e,pin,NULL);e->configured=1;e->generation=1;e->missing=1;
    e->profile.operation=1;e->profile.cylinders=1;e->profile.required_fuel_us=1000;e->profile.dead_us=1000;e->profile.dwell_us=2000;e->profile.maximum_injection_us=15000;
}
static void pulse(int angle_centi,unsigned rpm){
    const unsigned revolution=60000000/rpm;
    tunuino_engine_control e;prepare(&e);uint8_t lock[5]={1,1,0,(uint8_t)((uint16_t)angle_centi>>8),(uint8_t)angle_centi};assert(tunuino_engine_control_lock(&e,lock));assert(!e.outputs.armed);
    e.features.flags=TUNUINO_FEATURE_TRIMS|TUNUINO_FEATURE_LAUNCH|TUNUINO_FEATURE_LIMIT|TUNUINO_FEATURE_IAT;
    e.feature_inputs.iat_valid=true;e.feature_inputs.iat=8000;e.features.iat_reference=2000;e.features.iat_retard_start=3000;e.features.iat_retard=100;
    e.features.limit_cut=7000;e.features.limit_resume=6500;e.features.launch_cut=4000;e.features.launch_resume=3500;e.features.launch_advance=-500;e.features.spark_trim[0]=700;
    e.feature_inputs.launch_valid=e.feature_inputs.launch=true;
    e.fueling.enabled=1;e.fueling.cranking_rpm=500;e.fueling.fixed_cranking_advance=1;e.fueling.cranking_advance=2000;e.fueling.cranking_percent=e.fueling.afterstart_percent=10000;
    e.fueling.temperature[0]=-2000;e.fueling.temperature[1]=4000;e.fueling.temperature[2]=10000;for(unsigned i=0;i<3;++i)e.fueling.warmup_percent[i]=10000;
    high=false;falls=0;clock_us=0;assert(tunuino_engine_control_arm(&e,0,1));assert(!tunuino_engine_control_lock(&e,lock));
    for(clock_us=0;clock_us<=2*revolution;clock_us+=100){
        unsigned tooth=clock_us*36/revolution;uint32_t edge=tooth*revolution/36;
        tunuino_position_reading position={0};position.crank_valid=true;position.rpm=rpm;position.tooth_index=(uint16_t)(tooth%36);
        tunuino_engine_control_update(&e,clock_us,edge,&position,36,1,true,10000,3500,100000,true,80000);
        assert(e.outputs.armed&&!e.outputs.fault);tunuino_outputs_poll(&e.outputs,clock_us);
    }
    assert(falls>=1);uint32_t expected=(uint32_t)((uint64_t)((36000-angle_centi)%36000)*revolution/36000);if(!expected||expected<=2010)expected+=revolution;
    assert(falling[0]>=expected-150&&falling[0]<=expected+150);
    tunuino_engine_control_stop(&e,0);assert(!e.outputs.armed);
}
int main(void){
    bool enabled;int16_t angle;uint8_t bytes[5]={0};assert(tunuino_ignition_lock_decode(bytes,&enabled,&angle)&&!enabled&&angle==0);
    bytes[0]=1;bytes[1]=1;bytes[3]=3;bytes[4]=232;assert(tunuino_ignition_lock_decode(bytes,&enabled,&angle)&&enabled&&angle==1000);
    bytes[2]=1;assert(!tunuino_ignition_lock_decode(bytes,&enabled,&angle));bytes[2]=0;bytes[0]=0;assert(!tunuino_ignition_lock_decode(bytes,&enabled,&angle));bytes[0]=2;assert(!tunuino_ignition_lock_decode(bytes,&enabled,&angle));bytes[0]=1;bytes[1]=2;assert(!tunuino_ignition_lock_decode(bytes,&enabled,&angle));bytes[1]=1;bytes[3]=23;bytes[4]=113;assert(!tunuino_ignition_lock_decode(bytes,&enabled,&angle));
    tunuino_device d;tunuino_device_init(&d,NULL,NULL,NULL);d.active[27]=1;assert(tunuino_device_validate(d.active,sizeof(d.active)));d.active[28]=1;assert(tunuino_device_validation_errors(d.active,sizeof(d.active))&TUNUINO_CONFIG_TRIGGER);
    tunuino_engine_control e;prepare(&e);for(unsigned operation=0;operation<5;++operation){e.profile.operation=(uint8_t)operation;e.features.flags=TUNUINO_FEATURE_TRIMS;for(unsigned i=0;i<12;++i){e.features.spark_trim[i]=(int16_t)(i*10);e.ignition_locked=true;e.locked_angle=1000;assert(tunuino_engine_spark_advance(&e,i,-500)==1000);e.ignition_locked=false;assert(tunuino_engine_spark_advance(&e,i,-500)==-500+(int32_t)i*10);}}
    pulse(0,1000);pulse(1000,1000);pulse(-1000,1000);pulse(6000,1000);pulse(1000,300);
    puts("Ignition lock decode, legacy format, capability-independent validation, all cylinder slots/modes and actual coil pulse timing at 0/10/-10/60 degrees passed (host simulation)");return 0;
}
