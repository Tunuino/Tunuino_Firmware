#include "tunuino/vtec.h"
#include "tunuino/device.h"
#include "tunuino/outputs.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void put(uint8_t *p,unsigned at,unsigned n){p[at]=(uint8_t)(n>>8);p[at+1]=(uint8_t)n;}
static void defaults(uint8_t p[256]){memset(p,0,256);p[0]=p[1]=1;p[2]=10;const unsigned at[]={4,6,8,10,12,14,16,18,20,22,24,26},v[]={5000,4800,8000,4000,3500,6000,5500,7000,11000,100,2000,1500};for(unsigned i=0;i<12;++i)put(p,at[i],v[i]);}
static uint32_t now;
static tunuino_vtec_config config;
static tunuino_vtec_state state;
static void tick(uint32_t dt,bool armed,bool run,uint32_t rpm,uint16_t tps,uint32_t map,bool sensors,bool oil_valid,uint32_t oil){now+=dt;tunuino_vtec_update(&config,&state,now,armed,run,rpm,sensors,tps,sensors,map,sensors,80000,oil_valid,oil);}
static void reset(void){memset(&state,0,sizeof(state));now=0;}
static void engage(void){tick(0,true,true,5000000,5000,100000,true,true,250000);tick(100000,true,true,5000000,5000,100000,true,true,250000);assert(state.active);}
static uint32_t levels;
static void level(void *u,unsigned ch,bool on){(void)u;if(on)levels|=UINT32_C(1)<<ch;else levels&=~(UINT32_C(1)<<ch);}
int main(void){
    uint8_t p[256];defaults(p);assert(tunuino_vtec_decode(p,&config));uint8_t bad[256];
    const unsigned offsets[]={0,1,2,3,4,6,8,10,12,14,16,18,20,22,24,26,28,255};
    for(unsigned i=0;i<sizeof(offsets)/sizeof(offsets[0]);++i){memcpy(bad,p,256);bad[offsets[i]]=255;assert(!tunuino_vtec_decode(bad,&config));}
    assert(tunuino_vtec_decode(p,&config));reset();tick(0,false,true,6000000,9000,150000,true,true,250000);assert(!state.active&&state.reason==TUNUINO_VTEC_DISARMED);
    tick(0,true,false,6000000,9000,150000,true,true,250000);assert(!state.active&&state.reason==TUNUINO_VTEC_STOPPED);
    engage();tick(20000,true,true,4900000,3700,57000,true,true,250000);assert(state.active);
    tick(20000,true,true,4799000,3700,57000,true,true,250000);assert(!state.active&&state.reason==TUNUINO_VTEC_RPM);
    tick(20000,true,true,4900000,9000,100000,true,true,250000);assert(!state.active);tick(20000,true,true,5000000,9000,100000,true,true,250000);assert(state.active);
    tick(20000,true,true,6000000,3400,100000,true,true,250000);assert(!state.active&&state.reason==TUNUINO_VTEC_TPS);
    tick(20000,true,true,6000000,5000,100000,true,true,250000);assert(state.active);tick(20000,true,true,6000000,5000,54000,true,true,250000);assert(!state.active&&state.reason==TUNUINO_VTEC_MAP);
    tick(20000,true,true,8000000,9000,100000,true,true,250000);assert(!state.active&&state.reason==TUNUINO_VTEC_RPM);
    reset();engage();tick(20000,true,true,6000000,5000,100000,false,true,250000);assert(!state.active&&state.latched);tick(20000,true,true,6000000,5000,100000,true,true,250000);assert(!state.active&&state.reason==TUNUINO_VTEC_LATCHED);
    tick(0,false,false,0,0,0,false,false,0);assert(!state.latched);engage();
    reset();config.oil_required=1;engage();tick(20000,true,true,6000000,5000,100000,true,true,175000);assert(state.active);tick(20000,true,true,6000000,5000,100000,true,true,149000);assert(!state.active&&state.latched);
    reset();tick(0,true,true,6000000,5000,100000,true,false,0);assert(!state.active&&!state.latched);
    reset();engage();tick(100001,true,true,6000000,5000,100000,true,true,250000);assert(!state.active&&state.latched);
    reset();now=UINT32_MAX-50000;engage();assert(state.active); // Wrapping timer.
    reset();config.enabled=0;tick(0,true,true,6000000,5000,100000,true,true,250000);assert(!state.active);config.enabled=1;
    tunuino_outputs out;tunuino_outputs_init(&out,level,NULL);tunuino_outputs_arm(&out,0,1000);assert(tunuino_outputs_level(&out,9,true)&&levels==(1u<<9));tunuino_outputs_poll(&out,1000);assert(!levels&&!out.armed&&out.fault==TUNUINO_OUTPUT_WATCHDOG);
    tunuino_device d;tunuino_device_init(&d,NULL,NULL,NULL);uint8_t all[TUNUINO_IO_SETTINGS_SIZE];memcpy(all,d.active,sizeof(all));memcpy(all+TUNUINO_IO_VTEC_OFFSET,p,256);assert(!tunuino_device_validate(all,sizeof(all)));all[TUNUINO_IO_VTEC_OFFSET+1]=0;assert(tunuino_device_validate(all,sizeof(all)));assert(tunuino_device_restore(&d,all,TUNUINO_IO_V5_SIZE));assert(!d.active[TUNUINO_IO_VTEC_OFFSET]);
    puts("VTEC decoding, hysteresis, runtime, pressure/sensor latches, wrap, watchdog and legacy defaults passed (host simulation)");return 0;
}
