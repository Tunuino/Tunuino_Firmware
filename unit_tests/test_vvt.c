#include "tunuino/vvt.h"
#include "tunuino/vvt_target.h"
#include "tunuino/vtec.h"
#include "tunuino/device.h"
#include "tunuino/outputs.h"
#include "tunuino/position.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void put(uint8_t *p,unsigned at,unsigned n){p[at]=(uint8_t)(n>>8);p[at+1]=(uint8_t)n;}
static void defaults(uint8_t p[256]){
    memset(p,0,256);p[0]=2;p[64]=1;p[65]=1;p[66]=10;
    const unsigned at[]={4,6,8,10,12,14,16,18,20,22,24,26,68,70,72,74,76,78,80,84,86,88,90,94,96,98,100,102,104,106,108,112,114};
    const unsigned v[]={5000,4800,8000,4000,3500,6000,5500,7000,11000,10000,2000,1500,200,0,8000,4000,1000,0,0,9000,0,4000,2000,1500,100,7000,500,100,250,1000,500,2000,1500};
    for(unsigned i=0;i<sizeof(at)/sizeof(at[0]);++i)put(p,at[i],v[i]);
}
static tunuino_vvt_config config;static tunuino_vvt_state state;static uint32_t now;static uint64_t wall;
static void reset(void){memset(&state,0,sizeof(state));now=0;wall=1000000;}
static void tick(uint32_t dt,uint32_t raw,bool valid){now+=dt;wall+=dt;tunuino_vvt_update(&config,&state,now,true,true,2000000,true,80000,true,250000,valid,wall,wall,raw);}
static void engage(void){tick(0,9000,true);assert(!state.active);tick(100000,9000,true);assert(state.active&&state.duty==6000);}
static uint32_t levels;
static void pin(void *u,unsigned ch,bool high){(void)u;if(high)levels|=UINT32_C(1)<<ch;else levels&=~(UINT32_C(1)<<ch);}
int main(void){
    uint8_t p[256];defaults(p);assert(tunuino_vvt_decode(p,&config));tunuino_vtec_config vtec;assert(tunuino_vtec_decode(p,&vtec)&&!vtec.enabled);
    assert(tunuino_vvt_window(&config,36,1,0,12,0));assert(!tunuino_vvt_window(&config,36,1,0,0,0));assert(!tunuino_vvt_window(&config,36,1,0,34,1));
    uint8_t bad[256];const unsigned offsets[]={68,70,72,74,76,78,80,86,88,90,94,96,98,100,102,104,106,108,112,114};
    const unsigned invalid[]={9,8000,9501,8001,20001,20001,5001,56535,9001,4001,499,1401,12001,2001,60001,19,99,0,20001,2001};
    for(unsigned i=0;i<20;++i){memcpy(bad,p,256);put(bad,offsets[i],invalid[i]);assert(!tunuino_vvt_decode(bad,&config));}
    const unsigned bytes[]={64,65,66,67,92,93,110,111,123,255};for(unsigned i=0;i<10;++i){memcpy(bad,p,256);bad[bytes[i]]=255;assert(!tunuino_vvt_decode(bad,&config));}
    memcpy(bad,p,256);put(bad,82,1);put(bad,84,6464);assert(!tunuino_vvt_decode(bad,&config));assert(tunuino_vvt_decode(p,&config));
    reset();engage();tick(100000,8000,true);assert(state.active&&state.angle==1000&&state.duty==5000);tick(100000,7000,true);assert(state.angle==2000&&state.duty==4000);
    uint64_t previous=state.edge;int64_t integral=state.integral;uint16_t duty=state.duty;tunuino_vvt_update(&config,&state,now+1000,true,true,2000000,true,80000,true,250000,true,wall+1000,wall,7000);assert(state.edge==previous&&state.integral==integral&&state.duty==duty);
    reset();engage();tick(20000,9000,false);assert(!state.active&&state.latched&&state.duty==0);tick(20000,7000,true);assert(!state.active&&state.reason==TUNUINO_VVT_LATCHED);
    tunuino_vvt_update(&config,&state,now,false,false,0,false,0,false,0,false,0,0,0);assert(!state.latched&&!state.active);
    reset();engage();tunuino_vvt_update(&config,&state,now+1000,true,true,2000000,true,80000,true,250000,true,wall+250001,wall,9000);assert(state.latched&&!state.active);
    reset();engage();tick(20000,4000,true);assert(!state.active&&state.latched&&state.reason==TUNUINO_VVT_RANGE);
    reset();engage();for(unsigned i=0;i<10;++i)tick(100000,9000,true);assert(!state.active&&state.latched&&state.reason==TUNUINO_VVT_TRACKING);
    reset();engage();tick(100001,9000,true);assert(!state.active&&state.latched);
    reset();config.oil_required=1;engage();tunuino_vvt_update(&config,&state,now+20000,true,true,2000000,true,80000,true,149000,true,wall+20000,wall+20000,9000);assert(state.latched&&!state.active&&state.reason==TUNUINO_VVT_OIL);config.oil_required=0;
    reset();config.direction=1;tick(0,9000,true);tick(100000,9000,true);assert(state.active&&state.duty==2000);config.direction=0;
    reset();config.ki=1000;config.tracking_ms=5000;engage();tick(100000,9000,true);assert(state.duty==6200&&state.integral==200000000);config.ki=0;
    reset();now=UINT32_MAX-50000;engage();assert(state.active);
    tunuino_outputs output;tunuino_outputs_init(&output,pin,NULL);tunuino_outputs_arm(&output,0,100000);assert(tunuino_outputs_pwm(&output,9,0,config.hz,6000));tunuino_outputs_poll(&output,1000);assert(levels==(1u<<9));tunuino_outputs_poll(&output,4000);assert(!levels);tunuino_outputs_poll(&output,100000);assert(!levels&&!output.armed);
    tunuino_device d;tunuino_device_init(&d,NULL,NULL,NULL);uint8_t all[TUNUINO_IO_SETTINGS_SIZE];memcpy(all,d.active,sizeof(all));memcpy(all+TUNUINO_IO_VTEC_OFFSET,p,256);
    assert(tunuino_device_validation_errors(all,sizeof(all))&TUNUINO_CONFIG_VVT);all[TUNUINO_IO_VTEC_OFFSET+65]=0;assert(tunuino_device_validate(all,sizeof(all)));
    assert(tunuino_device_restore(&d,all,sizeof(all)));assert(d.active[TUNUINO_IO_VTEC_OFFSET]==2&&!d.active[TUNUINO_IO_VTEC_OFFSET+65]);
    // Real crank/CAM decoder: phase advances while the tooth window remains valid.
    tunuino_position position;tunuino_position_config source={{36,1,30,12000,500000},0,12,0};
    assert(tunuino_position_init(&position,&source,0)==TUNUINO_POSITION_ACQUIRING);
    defaults(p);assert(tunuino_vvt_decode(p,&config));config.delay_ms=0;memset(&state,0,sizeof(state));
    for(unsigned revolution=0;revolution<9;++revolution)for(unsigned tooth=0;tooth<35;++tooth){
        uint64_t time=1000u+(revolution*36u+tooth)*1000u;tunuino_position_crank_edge(&position,time);
        if((revolution==2&&tooth==9)||(revolution==4&&tooth==8)||(revolution==6&&tooth==7)){
            tunuino_position_cam_edge(&position,time);tunuino_cam_reading cam;tunuino_position_cam_read(&position,time,&cam);
            if(revolution>=4){assert(cam.valid&&cam.angle_centi==tooth*1000u);tunuino_vvt_update(&config,&state,(uint32_t)time,true,true,1666667,true,80000,true,250000,true,time,cam.time_us,cam.angle_centi);assert(state.active&&state.duty==(revolution==4?5000:4000));}
        }
    }
    tunuino_cam_reading missing;tunuino_position_cam_read(&position,323000,&missing);assert(!missing.valid);
    tunuino_vvt_update(&config,&state,323000,true,true,1666667,true,80000,true,250000,false,323000,0,0);assert(!state.active&&state.latched);
    // Signed bilinear RPM/MAP targets, including negative travel and clamped edges.
    uint8_t table[576]={0};assert(tunuino_vvt_target_empty(table));int16_t target;
    assert(!tunuino_vvt_target_lookup(table,0,0,&target));
    for(unsigned i=0;i<16;++i){put(table,2*i,500+500*i);put(table,32+2*i,2000+1000*i);}
    for(unsigned r=0;r<16;++r)for(unsigned c=0;c<16;++c)put(table,64+2*(r*16+c),(unsigned)(int16_t)(-3000+200*(int)r+200*(int)c));
    assert(tunuino_vvt_target_validate(table)&&tunuino_vvt_target_limits(table,-3000,3000));
    assert(!tunuino_vvt_target_limits(table,0,3000));
    assert(tunuino_vvt_target_lookup(table,750000,25000,&target)&&target==-2800);
    assert(tunuino_vvt_target_lookup(table,8000000,170000,&target)&&target==3000);
    assert(tunuino_vvt_target_lookup(table,0,0,&target)&&target==-3000);
    assert(tunuino_vvt_target_lookup(table,UINT32_MAX,UINT32_MAX,&target)&&target==3000);
    put(table,2,500);assert(!tunuino_vvt_target_validate(table));put(table,2,1000);
    defaults(p);p[64]=2;p[123]=1;assert(tunuino_vvt_decode(p,&config)&&config.table_mode);p[64]=1;assert(!tunuino_vvt_decode(p,&config));p[64]=2;
    assert(tunuino_vvt_decode(p,&config));config.delay_ms=0;
    for(unsigned i=0;i<256;++i)put(table,64+2*i,2000);
    tunuino_vvt_resolve_target(&config,table,true,true,2000000,100000);assert(!config.target_invalid&&config.target==2000);
    reset();tick(0,9000,true);assert(state.active&&state.duty==6000);
    for(unsigned i=0;i<256;++i)put(table,64+2*i,1000);
    tunuino_vvt_resolve_target(&config,table,true,true,2000000,100000);tick(50000,9000,true);assert(state.active&&state.duty==5000);
    tunuino_vvt_resolve_target(&config,table,true,false,2000000,100000);tick(1000,9000,true);assert(!state.active&&state.latched&&state.reason==TUNUINO_VVT_TARGET&&state.duty==0);
    tunuino_vvt_resolve_target(&config,table,true,true,2000000,100000);tick(1000,9000,true);assert(!state.active&&state.reason==TUNUINO_VVT_LATCHED);
    tunuino_vvt_resolve_target(&config,table,false,true,2000000,100000);assert(config.target_invalid);
    memcpy(all,d.active,sizeof(all));memcpy(all+TUNUINO_IO_VTEC_OFFSET,p,256);all[TUNUINO_IO_VTEC_OFFSET+65]=0;
    memcpy(all+TUNUINO_IO_VVT_TARGET_OFFSET,table,576);assert(tunuino_device_validation_errors(all,sizeof(all))&TUNUINO_CONFIG_VVT); // MAP is not configured.
    all[6]=1;put(all,16,25000);assert(tunuino_device_validate(all,sizeof(all)));
    put(all,TUNUINO_IO_VVT_TARGET_OFFSET+64,5000);assert(tunuino_device_validation_errors(all,sizeof(all))&TUNUINO_CONFIG_VVT);
    assert(!tunuino_device_validate(all,TUNUINO_IO_V7_SIZE));
    defaults(p);p[65]=0;memcpy(all,d.active,sizeof(all));memcpy(all+TUNUINO_IO_VTEC_OFFSET,p,256);assert(tunuino_device_restore(&d,all,TUNUINO_IO_V7_SIZE));assert(tunuino_vvt_target_empty(d.active+TUNUINO_IO_VVT_TARGET_OFFSET));assert(memcmp(d.active+TUNUINO_IO_VTEC_OFFSET,p,256)==0);
    // Reference PID fields and overrides act in the native controller, not only in the editor.
    defaults(p);p[64]=3;put(p,132,(uint16_t)(int16_t)-500);put(p,134,500);put(p,136,(uint16_t)(int16_t)-1500);put(p,138,1500);put(p,140,50);
    assert(tunuino_vvt_decode(p,&config)&&config.advanced);config.delay_ms=0;
    reset();tick(0,9000,true);assert(state.active&&state.duty==5500); // +15% correction cap.
    tick(100000,7050,true);assert(state.duty==4000); // 0.5 degree deadband.
    config.kp=0;config.ki=20000;config.deadband=0;reset();tick(0,7100,true);for(unsigned i=0;i<10;++i)tick(100000,7100,true);assert(state.integral==500000000&&state.duty==4500);
    config.override_dc=1;config.overridden_dc=3500;config.invert=1;tick(100000,7100,true);assert(state.duty==3500&&state.integral==0&&tunuino_vvt_pwm_duty(&config,&state)==6500);
    tick(1000,7100,false);assert(state.latched&&!state.active&&tunuino_vvt_pwm_duty(&config,&state)==0);
    config.override_target=1;config.overridden_target=1000;config.table_mode=2;tunuino_vvt_resolve_target(&config,NULL,false,false,0,0);assert(!config.target_invalid&&config.target==1000);
    memcpy(bad,p,256);bad[125]=1;put(bad,126,8500);assert(!tunuino_vvt_decode(bad,&config));memcpy(bad,p,256);put(bad,132,1);assert(!tunuino_vvt_decode(bad,&config));memcpy(bad,p,256);put(bad,136,1);assert(!tunuino_vvt_decode(bad,&config));memcpy(bad,p,256);bad[128]=1;put(bad,130,5000);assert(!tunuino_vvt_decode(bad,&config));
    p[65]=0;p[123]=2;memcpy(all,d.active,sizeof(all));memcpy(all+TUNUINO_IO_VTEC_OFFSET,p,256);all[6]=1;put(all,16,25000);
    assert(!tunuino_device_validate(all,sizeof(all)));memcpy(all+TUNUINO_IO_VVT_TARGET2_OFFSET,table,576);assert(tunuino_device_validate(all,sizeof(all)));
    assert(!tunuino_device_validate(all,TUNUINO_IO_V8_SIZE)); // angle 2 cannot reference a missing page.
    puts("VVT RPM/MAP signed interpolation, travel validation, target source failure and legacy migration; VVT PID units, inverse direction, duplicate feedback, limits, stale/source faults, latches, timer wrap, PWM/watchdog and disabled legacy configuration passed (host simulation)");return 0;
}
