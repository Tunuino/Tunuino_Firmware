/* Copyright (c) 2026 Tunuino. Proteus v0.7 F427 only.
 * PCB: Tunuino/proteus 383558bf7ca1c53ad8bbd8fdc9dd3a4187013c46.
 * VNLD5160 low-side inputs and MIC4427 non-inverting ignition inputs.
 * IGN is a logic output for an external ignition stage, not a coil power sink.
 * TIM2 is the reference counter; GPT5 provides independent edge/lease IRQs. */
#include "ch.h"
#include "hal.h"
#include "board_io.h"
#include "board_outputs.h"
#include "tunuino/engine_control.h"
#include "tunuino/aux_control.h"
#include "tunuino/vtec.h"
#include "tunuino/vvt.h"
#include <string.h>
typedef struct {ioportid_t port;unsigned pad;} pin;
static const pin pins[28]={
    {GPIOD,7},{GPIOG,9},{GPIOG,10},{GPIOG,11},{GPIOG,12},{GPIOG,13},{GPIOG,14},{GPIOB,4},
    {GPIOB,5},{GPIOB,6},{GPIOB,7},{GPIOB,9},{GPIOB,8},{GPIOE,0},{GPIOE,1},{GPIOE,2},
    {GPIOD,4},{GPIOD,3},{GPIOC,9},{GPIOC,8},{GPIOC,7},{GPIOG,8},{GPIOG,7},{GPIOG,6},
    {GPIOG,5},{GPIOG,4},{GPIOG,3},{GPIOG,2}
};
static tunuino_engine_control engine;
static tunuino_aux_control auxiliary;
static tunuino_vtec_config vtec_config;
static tunuino_vtec_state vtec_state;
static tunuino_vvt_config vvt_config;
static tunuino_vvt_state vvt_state;
static bool timer_ready;
static uint32_t last_host_seen;
static bool autonomous,persisted_profile,boot_pending;
static uint32_t loaded_generation;
static void write_pin(void *user,unsigned channel,bool high){
    (void)user;if(high)palSetPad(pins[channel].port,pins[channel].pad);
    else palClearPad(pins[channel].port,pins[channel].pad);
}
static void timer_schedule(void){
    if(!timer_ready)return;
    if(GPTD5.state!=GPT_READY)gptStopTimerI(&GPTD5);
    uint32_t delay=tunuino_outputs_next(&engine.outputs,TIM2->CNT);
    if(delay)gptStartOneShotI(&GPTD5,delay<5?5:delay);
}
static void alarm_callback(GPTDriver *driver){
    (void)driver;chSysLockFromISR();
    tunuino_outputs_poll(&engine.outputs,TIM2->CNT);
    if(engine.outputs.fault)engine.mode=3;
    timer_schedule();chSysUnlockFromISR();
}
static const GPTConfig timer_config={.frequency=1000000,.callback=alarm_callback,.cr2=0,.dier=0};
static void put32(uint8_t *p,uint32_t v){p[0]=(uint8_t)(v>>24);p[1]=(uint8_t)(v>>16);p[2]=(uint8_t)(v>>8);p[3]=(uint8_t)v;}
static uint32_t u32(const uint8_t *p){return (uint32_t)tunuino_device_u16(p)<<16|tunuino_device_u16(p+2);}
static void put16(uint8_t *p,uint16_t v){p[0]=(uint8_t)(v>>8);p[1]=(uint8_t)v;}
static bool arm(uint32_t now,uint32_t generation){
    bool okay=tunuino_engine_control_arm(&engine,now,generation);
    if(okay){memset(&vvt_state,0,sizeof(vvt_state));vvt_state.reason=vvt_config.enabled?TUNUINO_VVT_STOPPED:TUNUINO_VVT_DISABLED;tunuino_aux_reset(&auxiliary);memset(&vtec_state,0,sizeof(vtec_state));vtec_state.reason=vtec_config.enabled?TUNUINO_VTEC_STOPPED:TUNUINO_VTEC_DISABLED;}
    return okay;
}
static bool command(void *user,const uint8_t *q,size_t length,uint8_t *out,size_t capacity,size_t *written){
    tunuino_device *d=user;bool okay=false;
    if(loaded_generation!=d->generation)board_outputs_restore(d,false);
    if(length==1&&q[0]==10&&capacity>=40){
        memset(out,0,40);out[1]=1;out[2]=vvt_config.enabled;out[3]=vvt_state.active;
        out[4]=vvt_state.reason;out[5]=vvt_state.latched;out[6]=vvt_config.channel;
        out[7]=(uint8_t)(!d->dirty&&loaded_generation==d->generation);
        put32(out+8,d->generation);put16(out+12,(uint16_t)vvt_config.target);
        bool feedback=d->active[TUNUINO_IO_VTEC_OFFSET]==2&&d->active[TUNUINO_IO_VTEC_OFFSET+64]==1
            &&!d->dirty&&(d->position_flags&7)==7&&d->position_time_us>=d->cam.time_us
            &&d->position_time_us-d->cam.time_us<=(uint64_t)vvt_config.max_age_ms*1000u;
        int32_t measured=(int32_t)vvt_config.reference-(int32_t)d->cam.angle_centi;
        if(measured>36000)measured-=72000;
        if(measured< -36000)measured+=72000;
        feedback=feedback&&measured>=INT16_MIN&&measured<=INT16_MAX;
        if(feedback)put16(out+14,(uint16_t)(int16_t)measured);
        put16(out+16,vvt_state.duty);out[18]=feedback;
        put32(out+20,vvt_state.elapsed/1000u);put32(out+24,vvt_state.tracking/1000u);
        if((d->position_flags&7)==7&&d->position_time_us>=d->cam.time_us){uint64_t age=d->position_time_us-d->cam.time_us;put32(out+28,age>UINT32_MAX?UINT32_MAX:(uint32_t)age);}
        put32(out+32,d->pressure_milli_kpa[1]);out[36]=(d->pressure_valid_mask&2)?1:0;
        *written=40;return true;
    }
    if(length==1&&q[0]==9&&capacity>=24){
        memset(out,0,24);out[1]=1;out[2]=vtec_config.enabled;out[3]=vtec_state.active;
        out[4]=vtec_state.reason;out[5]=vtec_state.latched;out[6]=vtec_config.channel;
        out[7]=(uint8_t)(!d->dirty&&loaded_generation==d->generation);
        put32(out+8,vtec_state.elapsed/1000u);put32(out+12,d->generation);
        put32(out+16,d->pressure_milli_kpa[1]);out[20]=(d->pressure_valid_mask&2)?1:0;
        *written=24;return true;
    }
    if(length==1&&q[0]==8&&capacity>=40){
        uint16_t tps=0;bool valid=tunuino_aux_tps(&auxiliary.config,d->adc_valid,d->raw,&tps);
        tunuino_feature_inputs in;tunuino_features_sample(&engine.features,d->adc_valid,d->raw,valid,tps,
            (d->live.valid_mask&TUNUINO_LIVE_LAMBDA_VALID)!=0,d->live.lambda_million,&in);
        const tunuino_feature_state *f=&engine.feature_state;
        memset(out,0,40);out[1]=1;out[2]=d->active[TUNUINO_IO_AUX_OFFSET+68]!=0;
        out[3]=(uint8_t)((in.tps_valid?1:0)|(in.iat_valid?2:0)|(in.voltage_valid?4:0)|(in.flex_valid?8:0)|(in.launch_valid?16:0)|(in.lambda_valid?32:0));
        put16(out+4,engine.features.flags);put16(out+6,in.tps);put16(out+8,(uint16_t)(int16_t)in.iat);put16(out+10,in.voltage);put16(out+12,in.ethanol);
        put16(out+14,f->target);put32(out+16,f->multiplier);put16(out+20,(uint16_t)(int16_t)f->lambda_trim);
        put16(out+22,f->extra_us);put16(out+24,f->dead_us);put16(out+26,f->dwell_us);
        out[28]=(uint8_t)((f->dfco?1:0)|(f->limited?2:0)|(f->launch_limited?4:0)|(f->closed_loop?8:0)|(f->accel_active?16:0)|((engine.outputs.fault&TUNUINO_OUTPUT_PROTECTION)?32:0));
        put16(out+30,(uint16_t)f->advance_delta);put32(out+32,d->generation);*written=40;return true;
    }
    if(length==1&&q[0]==7&&capacity>=32){
        uint16_t tps=0;bool valid=tunuino_aux_tps(&auxiliary.config,d->adc_valid,d->raw,&tps);
        memset(out,0,32);out[1]=1;out[2]=auxiliary.config.idle_mode;out[3]=auxiliary.config.boost_mode;
        out[4]=valid;out[5]=d->active[TUNUINO_IO_AUX_OFFSET]!=0;
        put16(out+6,valid?tps:0);put16(out+8,auxiliary.idle_duty);put16(out+10,auxiliary.boost_duty);
        put16(out+12,auxiliary.idle_target);put16(out+14,auxiliary.config.boost_target);
        put32(out+16,d->generation);put16(out+20,auxiliary.config.boost_cut);
        out[22]=auxiliary.boost_active;out[23]=auxiliary.idle_active;
        uint32_t mask=0;if(auxiliary.config.idle_mode)mask|=UINT32_C(1)<<(auxiliary.config.idle_channel-1);
        if(auxiliary.config.boost_mode)mask|=UINT32_C(1)<<(auxiliary.config.boost_channel-1);
        put32(out+24,mask);*written=32;return true;
    }
    if(length==1&&q[0]==0&&capacity>=24){
        last_host_seen=TIM2->CNT;
        memset(out,0,24);chSysLock();
        out[1]=1;out[2]=engine.mode;out[3]=engine.outputs.fault;
        put32(out+4,engine.outputs.active_mask);put32(out+8,tunuino_outputs_pending(&engine.outputs));
        out[12]=engine.configured;out[13]=engine.profile.cylinders;out[14]=timer_ready;
        out[15]=(uint8_t)((persisted_profile?1u:0u)|(autonomous?2u:0u));
        put32(out+16,engine.generation);put32(out+20,engine.last_injection_us);
        chSysUnlock();*written=24;return true;
    }
    if(length==129&&q[0]==1&&!d->dirty&&!auxiliary.config.idle_mode&&!auxiliary.config.boost_mode&&!engine.features.flags&&!vtec_config.enabled&&!vvt_config.enabled){
        chSysLock();okay=tunuino_engine_control_configure(&engine,q+1,d->active[3],d->active[4],d->generation);chSysUnlock();
        if(okay){
            (void)tunuino_fuel_corrections_decode(d->active+103,&engine.fueling);
            autonomous=persisted_profile=boot_pending=false;
            board_io_phase(d,engine.profile.cam_first,engine.profile.cam_last,engine.profile.cam_phase);
        }
    }else if(length==9&&q[0]==2&&memcmp(q+1,"ARM!",4)==0&&timer_ready&&!d->dirty
        &&d->persisted&&d->active[2]&&u32(q+5)==d->generation){
        chSysLock();last_host_seen=TIM2->CNT;okay=arm(TIM2->CNT,d->generation);timer_schedule();chSysUnlock();
    }else if(length==5&&q[0]==6&&memcmp(q+1,"RUN!",4)==0&&timer_ready
        &&persisted_profile&&!d->dirty&&d->persisted&&d->active[2]){
        chSysLock();last_host_seen=TIM2->CNT;okay=arm(TIM2->CNT,d->generation);timer_schedule();chSysUnlock();
    }else if(length==1&&q[0]==3){board_outputs_stop();okay=true;
    }else if(length==8&&q[0]==4&&memcmp(q+4,"TEST",4)==0&&timer_ready
        &&!d->trigger_valid&&!(d->live.valid_mask&TUNUINO_LIVE_RPM_VALID)){
        chSysLock();
        if(!engine.outputs.armed){
            tunuino_outputs_arm(&engine.outputs,TIM2->CNT,100000);
            okay=tunuino_outputs_pulse(&engine.outputs,q[1],TIM2->CNT,TIM2->CNT+1000,tunuino_device_u16(q+2));
            if(okay)engine.mode=4;else tunuino_engine_control_stop(&engine,0);
            timer_schedule();
        }
        chSysUnlock();
    }else if(length==8&&q[0]==5&&memcmp(q+4,"AUX!",4)==0&&timer_ready
        &&!d->trigger_valid&&!(d->live.valid_mask&TUNUINO_LIVE_RPM_VALID)){
        chSysLock();
        unsigned selected=(unsigned)q[1]+1u;
        bool assigned=engine.configured&&!d->dirty&&engine.generation==d->generation
            &&(selected==engine.profile.pump||selected==engine.profile.fan||selected==engine.profile.relay);
        if(assigned){
            okay=tunuino_outputs_aux_test(&engine.outputs,q[1],TIM2->CNT,(uint32_t)tunuino_device_u16(q+2)*1000u);
            if(okay)engine.mode=4;
            timer_schedule();
        }
        chSysUnlock();
    }
    if(okay&&capacity){out[0]=0;*written=1;return true;}
    return false;
}
void board_outputs_start(tunuino_device *d){
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIODEN|RCC_AHB1ENR_GPIOEEN|RCC_AHB1ENR_GPIOGEN;
    (void)RCC->AHB1ENR;
    /* Write inactive latches before changing pin mode: no startup pulse. */
    for(unsigned i=0;i<28;++i){palClearPad(pins[i].port,pins[i].pad);palSetPadMode(pins[i].port,pins[i].pad,PAL_MODE_OUTPUT_PUSHPULL);}
    tunuino_engine_control_init(&engine,write_pin,NULL);
    memset(&auxiliary,0,sizeof(auxiliary));
    memset(&vvt_config,0,sizeof(vvt_config));memset(&vvt_state,0,sizeof(vvt_state));vvt_state.reason=TUNUINO_VVT_DISABLED;
    memset(&vtec_config,0,sizeof(vtec_config));memset(&vtec_state,0,sizeof(vtec_state));vtec_state.reason=TUNUINO_VTEC_DISABLED;
    memset(&engine.features,0,sizeof(engine.features));memset(&engine.feature_inputs,0,sizeof(engine.feature_inputs));
    autonomous=persisted_profile=boot_pending=false;loaded_generation=d->generation;
    timer_ready=gptStart(&GPTD5,&timer_config)==MSG_OK;
    d->output_command=command;d->output_user=d;
    d->configuration_writable=board_outputs_idle;
}
void board_outputs_restore(tunuino_device *d,bool boot){
    board_outputs_stop();loaded_generation=d->generation;
    memset(&auxiliary,0,sizeof(auxiliary));
    memset(&vvt_config,0,sizeof(vvt_config));memset(&vvt_state,0,sizeof(vvt_state));vvt_state.reason=TUNUINO_VVT_DISABLED;
    memset(&vtec_config,0,sizeof(vtec_config));memset(&vtec_state,0,sizeof(vtec_state));vtec_state.reason=TUNUINO_VTEC_DISABLED;
    memset(&engine.features,0,sizeof(engine.features));memset(&engine.feature_inputs,0,sizeof(engine.feature_inputs));
    autonomous=persisted_profile=false;engine.configured=0;engine.ignition_locked=false;engine.locked_angle=0;
    if(!d->persisted||d->dirty||!tunuino_device_validate(d->active,sizeof(d->active)))return;
    (void)tunuino_aux_decode(d->active+TUNUINO_IO_AUX_OFFSET,0,true,&auxiliary.config);
    (void)tunuino_vtec_decode(d->active+TUNUINO_IO_VTEC_OFFSET,&vtec_config);
    (void)tunuino_vvt_decode(d->active+TUNUINO_IO_VTEC_OFFSET,&vvt_config);
    vvt_state.reason=vvt_config.enabled?TUNUINO_VVT_DISARMED:TUNUINO_VVT_DISABLED;
    vtec_state.reason=vtec_config.enabled?TUNUINO_VTEC_DISARMED:TUNUINO_VTEC_DISABLED;
    (void)tunuino_features_decode(d->active+TUNUINO_IO_AUX_OFFSET,&engine.features);
    engine.features.target_table=d->active+TUNUINO_IO_AFR_OFFSET;
    chSysLock();
    bool okay=tunuino_engine_control_configure(&engine,d->active+TUNUINO_ENGINE_PROFILE_OFFSET,
        d->active[3],d->active[4],d->generation);
    chSysUnlock();
    if(!okay||!tunuino_engine_control_lock(&engine,d->active+27))return;
    (void)tunuino_fuel_corrections_decode(d->active+103,&engine.fueling);
    board_io_phase(d,engine.profile.cam_first,engine.profile.cam_last,engine.profile.cam_phase);
    persisted_profile=true;autonomous=d->active[TUNUINO_ENGINE_POLICY_OFFSET]==1;
    /* Only power-up may auto-arm. Saving, reconnecting, STOP and faults never do. */
    boot_pending=boot&&autonomous;
}
bool board_outputs_autonomous(void){return autonomous;}
void board_outputs_stop(void){
    boot_pending=false;
    chSysLock();tunuino_engine_control_stop(&engine,0);timer_schedule();chSysUnlock();
    memset(&vvt_state,0,sizeof(vvt_state));vvt_state.reason=vvt_config.enabled?TUNUINO_VVT_DISARMED:TUNUINO_VVT_DISABLED;
    tunuino_aux_reset(&auxiliary);memset(&vtec_state,0,sizeof(vtec_state));vtec_state.reason=vtec_config.enabled?TUNUINO_VTEC_DISARMED:TUNUINO_VTEC_DISABLED;
}
bool board_outputs_idle(void){bool idle;chSysLock();idle=!engine.outputs.armed;chSysUnlock();return idle;}
void board_outputs_update(tunuino_device *d,const tunuino_position_reading *position,uint32_t edge,uint32_t now){
    if(loaded_generation!=d->generation)board_outputs_restore(d,false);
    uint16_t ve=0;int16_t advance=0;
    bool valid=!d->dirty&&d->adc_valid&&d->trigger_valid;
    if(valid)valid=tunuino_device_engine_values(d,&ve,&advance);
    uint16_t tps=0;bool tps_valid=tunuino_aux_tps(&auxiliary.config,d->adc_valid,d->raw,&tps);
    tunuino_features_sample(&engine.features,d->adc_valid,d->raw,tps_valid,tps,
        (d->live.valid_mask&TUNUINO_LIVE_LAMBDA_VALID)!=0,d->live.lambda_million,&engine.feature_inputs);
    chSysLock();
    if(boot_pending){
        boot_pending=false;
        if(timer_ready&&!d->dirty)(void)arm(now,d->generation);
    }
    if(engine.configured&&engine.generation!=d->generation){
        tunuino_engine_control_stop(&engine,TUNUINO_OUTPUT_CALIBRATION);engine.configured=0;
    }
    if(engine.mode==4){
        if(d->trigger_valid||(d->live.valid_mask&TUNUINO_LIVE_RPM_VALID))tunuino_engine_control_stop(&engine,TUNUINO_OUTPUT_SYNC);
        else tunuino_outputs_poll(&engine.outputs,now);
        if(!tunuino_outputs_pending(&engine.outputs))tunuino_engine_control_stop(&engine,0);
    }else if(engine.outputs.armed){
        if(!autonomous&&(uint32_t)(now-last_host_seen)>2000000u)tunuino_engine_control_stop(&engine,TUNUINO_OUTPUT_WATCHDOG);
        else if(d->dirty)tunuino_engine_control_stop(&engine,TUNUINO_OUTPUT_CALIBRATION);
        else tunuino_engine_control_update(&engine,now,edge,position,d->active[3],d->generation,
            valid,ve,advance,d->live.map_milli_kpa,(d->live.valid_mask&TUNUINO_LIVE_COOLANT_VALID)!=0,d->live.coolant_milli_c);
    }
    bool active=engine.outputs.armed&&(engine.mode==1||engine.mode==2);
    if(!tunuino_aux_update(&auxiliary,now,active,engine.mode==2,d->live.rpm_milli,tps_valid,tps,
        (d->live.valid_mask&TUNUINO_LIVE_COOLANT_VALID)!=0,d->live.coolant_milli_c,
        (d->live.valid_mask&TUNUINO_LIVE_MAP_VALID)!=0,d->live.map_milli_kpa)){
        tunuino_engine_control_stop(&engine,TUNUINO_OUTPUT_OVERBOOST);
    }else if(active){
        const tunuino_aux_config *c=&auxiliary.config;
        bool okay=(!c->idle_mode||tunuino_outputs_pwm(&engine.outputs,c->idle_channel-1,now,c->idle_hz,auxiliary.idle_duty))
            &&(!c->boost_mode||tunuino_outputs_pwm(&engine.outputs,c->boost_channel-1,now,c->boost_hz,auxiliary.boost_duty));
        if(!okay)tunuino_engine_control_stop(&engine,TUNUINO_OUTPUT_CALIBRATION);
    }
    tunuino_vtec_update(&vtec_config,&vtec_state,now,engine.outputs.armed!=0,
        engine.mode==2&&!d->dirty&&d->trigger_valid,d->live.rpm_milli,tps_valid,tps,
        (d->live.valid_mask&TUNUINO_LIVE_MAP_VALID)!=0,d->live.map_milli_kpa,
        (d->live.valid_mask&TUNUINO_LIVE_COOLANT_VALID)!=0,d->live.coolant_milli_c,
        !d->dirty&&d->pressure_generation==d->generation&&(d->pressure_valid_mask&2),d->pressure_milli_kpa[1]);
    if(vtec_config.enabled&&engine.outputs.armed&&!tunuino_outputs_level(&engine.outputs,vtec_config.channel-1,vtec_state.active!=0))
        tunuino_engine_control_stop(&engine,TUNUINO_OUTPUT_CALIBRATION);
    tunuino_vvt_update(&vvt_config,&vvt_state,now,engine.outputs.armed!=0,
        engine.mode==2&&!d->dirty&&d->trigger_valid,d->live.rpm_milli,
        (d->live.valid_mask&TUNUINO_LIVE_COOLANT_VALID)!=0,d->live.coolant_milli_c,
        !d->dirty&&d->pressure_generation==d->generation&&(d->pressure_valid_mask&2),d->pressure_milli_kpa[1],
        !d->dirty&&(d->position_flags&7)==7,d->position_time_us,d->cam.time_us,d->cam.angle_centi);
    if(vvt_config.enabled&&engine.outputs.armed&&(engine.mode==1||engine.mode==2)
        &&!tunuino_outputs_pwm(&engine.outputs,vvt_config.channel-1,now,vvt_config.hz,vvt_state.duty))
        tunuino_engine_control_stop(&engine,TUNUINO_OUTPUT_CALIBRATION);
    timer_schedule();chSysUnlock();
}

uint8_t board_outputs_fault(void){return engine.outputs.fault;}
