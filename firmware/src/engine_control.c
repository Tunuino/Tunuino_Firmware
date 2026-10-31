/* Copyright (c) 2026 Tunuino. */
#include "tunuino/engine_control.h"
#include <math.h>
#include <string.h>
static uint32_t angle(int64_t value){int64_t a=value%72000;return (uint32_t)(a<0?a+72000:a);}
void tunuino_engine_control_init(tunuino_engine_control *e,tunuino_output_write write,void *user){
    memset(e,0,sizeof(*e));tunuino_outputs_init(&e->outputs,write,user);
}
void tunuino_engine_control_stop(tunuino_engine_control *e,uint8_t fault){
    tunuino_outputs_stop(&e->outputs,fault);e->mode=fault?3:0;e->have_phase=0;e->fan_on=0;e->last_injection_us=0;
    memset(&e->fuel_state,0,sizeof(e->fuel_state));
    memset(&e->feature_state,0,sizeof(e->feature_state));
}
bool tunuino_engine_control_configure(tunuino_engine_control *e,const uint8_t p[128],uint16_t slots,uint8_t missing,uint32_t generation){
    tunuino_engine_profile profile;
    if(e->outputs.armed||!generation||!tunuino_engine_profile_decode(p,slots,missing,&profile))return false;
    e->profile=profile;e->configured=1;e->generation=generation;e->missing=missing;return true;
}
bool tunuino_engine_control_arm(tunuino_engine_control *e,uint32_t now,uint32_t generation){
    if(e->outputs.armed||!e->configured||generation!=e->generation)return false;
    tunuino_outputs_arm(&e->outputs,now,20000);e->mode=1;e->armed_at=e->last_now=now;
    e->cycle=0;e->have_phase=0;e->fan_on=0;e->last_injection_us=0;
    memset(&e->fuel_state,0,sizeof(e->fuel_state));
    memset(&e->feature_state,0,sizeof(e->feature_state));
    for(unsigned i=0;i<28;++i)e->scheduled_cycle[i]=UINT32_MAX;
    return e->outputs.armed!=0;
}
static void auxiliary(tunuino_engine_control *e,uint32_t now,bool crank,bool clt_valid,int32_t clt){
    const tunuino_engine_profile *p=&e->profile;
    bool prime=(uint32_t)(now-e->armed_at)<(uint32_t)p->prime_ms*1000u;
    if(p->pump)(void)tunuino_outputs_level(&e->outputs,p->pump-1,prime||crank);
    if(p->relay)(void)tunuino_outputs_level(&e->outputs,p->relay-1,true);
    if(!clt_valid||clt>=(int32_t)p->fan_on_centi*10)e->fan_on=1;
    else if(clt<=(int32_t)p->fan_off_centi*10)e->fan_on=0;
    if(p->fan)(void)tunuino_outputs_level(&e->outputs,p->fan-1,e->fan_on!=0);
}
static bool schedule(tunuino_engine_control *e,unsigned ch,uint32_t target,uint32_t duration,
    uint32_t current,uint32_t revolution,uint32_t now,bool first,uint32_t period){
    uint32_t cycle=e->cycle+(target<=current?1u:0u);
    if(e->scheduled_cycle[ch]==cycle)return true;
    uint32_t delta=(target+period-current)%period;
    if(!delta)delta=period;
    uint32_t until=(uint32_t)(((uint64_t)delta*revolution+18000)/36000u);
    if(until<=duration+10u){
        if(first){e->scheduled_cycle[ch]=cycle;return true;}
        return false;
    }
    uint32_t start=until-duration;
    if(start>3000u)return true;
    if(!tunuino_outputs_pulse(&e->outputs,ch,now,now+start,duration))return false;
    e->scheduled_cycle[ch]=cycle;return true;
}
void tunuino_engine_control_update(tunuino_engine_control *e,uint32_t now,uint32_t edge,
    const tunuino_position_reading *r,uint16_t slots,uint32_t generation,
    bool valid,uint16_t ve,int16_t advance,uint32_t map,bool clt_valid,int32_t clt){
    if(!e->outputs.armed)return;
    if(!tunuino_outputs_keepalive(&e->outputs,now,20000)){e->mode=3;return;}
    if(generation!=e->generation){tunuino_engine_control_stop(e,TUNUINO_OUTPUT_CALIBRATION);e->configured=0;return;}
    bool crank=r&&r->crank_valid;
    bool phased=e->profile.operation==0;
    uint32_t period=phased?72000u:36000u;
    if(!r||(phased&&!r->cycle_valid)||!crank||!valid){
        if(e->have_phase){tunuino_engine_control_stop(e,TUNUINO_OUTPUT_SYNC);return;}
        auxiliary(e,now,crank,clt_valid,clt);return;
    }
    if(!clt_valid||slots<4||slots>120||!isfinite(r->rpm)||r->rpm<30||r->rpm>20000
        ||(phased&&r->revolution_phase>1)||r->tooth_index>=slots||ve==0||ve>30000
        ||advance< -9000||advance>9000||map==0||map>500000){
        tunuino_engine_control_stop(e,TUNUINO_OUTPUT_CALIBRATION);return;
    }
    uint32_t revolution=(uint32_t)(60000000.0/r->rpm+0.5),age=now-edge;
    uint32_t maximum_age=(uint32_t)((uint64_t)revolution*(e->missing+1u)*2u/slots);
    if(age>maximum_age||age>revolution/2u){tunuino_engine_control_stop(e,TUNUINO_OUTPUT_SYNC);return;}
    uint32_t current=angle((int64_t)e->profile.tooth_zero_centi+(phased?(uint32_t)r->revolution_phase*36000u:0u)
        +(uint32_t)r->tooth_index*36000u/slots+(uint64_t)age*36000u/revolution)%period;
    bool first=!e->have_phase;
    if(!first){
        uint32_t delta=(current+period-e->last_angle)%period;
        if((uint32_t)(now-e->last_now)>revolution||delta>period/2u){
            if(current<e->last_angle&&e->last_angle-current<100u)current=e->last_angle;
            else {tunuino_engine_control_stop(e,TUNUINO_OUTPUT_SYNC);return;}
        }
        if(current<e->last_angle)++e->cycle;
    }
    e->have_phase=1;e->mode=2;e->last_angle=current;e->last_now=now;
    if(e->profile.operation>=3&&(e->features.flags&TUNUINO_FEATURE_TRIMS)){
        for(unsigned i=0;i<e->profile.cylinders;++i)for(unsigned j=0;j<i;++j)
            if(e->profile.cylinder[i].coil==e->profile.cylinder[j].coil&&e->features.spark_trim[i]!=e->features.spark_trim[j]){
                tunuino_engine_control_stop(e,TUNUINO_OUTPUT_CALIBRATION);return;
            }
    }
    uint64_t fuel=(uint64_t)e->profile.required_fuel_us*ve*map;
    uint32_t multiplier;
    if(!tunuino_fuel_corrections_apply(&e->fueling,&e->fuel_state,now,(uint32_t)(r->rpm*1000+0.5),
        clt_valid,clt,&multiplier,&advance)){
        tunuino_engine_control_stop(e,TUNUINO_OUTPUT_CALIBRATION);return;
    }
    if(!tunuino_features_update(&e->features,&e->feature_state,&e->feature_inputs,now,(uint32_t)(r->rpm*1000+0.5),map,
        clt_valid,clt,e->fueling.enabled?e->fueling.cranking_rpm:400,e->profile.dead_us,e->profile.dwell_us)){
        uint8_t fault=e->feature_state.protection?TUNUINO_OUTPUT_PROTECTION:TUNUINO_OUTPUT_CALIBRATION;
        tunuino_engine_control_stop(e,fault);return;
    }
    const tunuino_feature_state *f=&e->feature_state;
    int32_t adjusted=(int32_t)advance+f->advance_delta;
    if((e->features.flags&TUNUINO_FEATURE_LAUNCH)&&e->feature_inputs.launch&&adjusted>e->features.launch_advance)adjusted=e->features.launch_advance;
    uint64_t base=(fuel+500000000u)/1000000000u;
    uint64_t corrected=((base*multiplier+5000u)/10000u*f->multiplier+5000u)/10000u+f->extra_us;
    /* ReqFuel and all enrichments represent fuel per four-stroke cycle.
     * Two events each cycle split effective fuel; dead time belongs to each event. */
    if(!phased)corrected=(corrected+1u)/2u;
    uint32_t duty_limit=(uint32_t)((uint64_t)revolution*(phased?16u:8u)/10u);
    uint32_t injection=(uint32_t)corrected+f->dead_us;
    if((!f->fuel_cut&&(injection==0||injection>e->profile.maximum_injection_us
        ||injection>duty_limit))||f->dwell_us>duty_limit){
        tunuino_engine_control_stop(e,TUNUINO_OUTPUT_CALIBRATION);return;
    }
    e->last_injection_us=f->fuel_cut?0:injection;
    for(unsigned i=0;i<e->profile.cylinders;++i){const tunuino_cylinder *c=&e->profile.cylinder[i];
        uint32_t cylinder_fuel=(uint32_t)corrected;
        int32_t spark=adjusted;
        if(e->features.flags&TUNUINO_FEATURE_TRIMS){cylinder_fuel=(uint32_t)((corrected*(uint32_t)(10000+e->features.fuel_trim[i])+5000)/10000);spark+=e->features.spark_trim[i];}
        cylinder_fuel+=f->dead_us;
        if(spark< -9000||spark>9000||(!f->fuel_cut&&(cylinder_fuel>e->profile.maximum_injection_us||cylinder_fuel>duty_limit))){tunuino_engine_control_stop(e,TUNUINO_OUTPUT_CALIBRATION);return;}
        if(f->fuel_cut)tunuino_outputs_cancel_injector(&e->outputs,c->injector);
        if((!f->fuel_cut&&!schedule(e,c->injector,angle(((e->profile.operation==2||e->profile.operation==4)?0:(int64_t)c->tdc_centi)+e->profile.injection_end_centi)%period,cylinder_fuel,current,revolution,now,first,period))
            ||!schedule(e,16+c->coil,angle((int64_t)c->tdc_centi-spark)%period,f->dwell_us,current,revolution,now,first,period)){
            tunuino_engine_control_stop(e,TUNUINO_OUTPUT_LATE);return;
        }
    }
    auxiliary(e,now,crank,clt_valid,clt);
}
