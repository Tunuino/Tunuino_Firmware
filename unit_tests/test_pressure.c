#include "tunuino/device.h"
#include "tunuino/pressure.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static tunuino_device device;
static void put(uint8_t *p,unsigned v){p[0]=(uint8_t)(v>>8);p[1]=(uint8_t)v;}
static uint32_t u32(const uint8_t *p){return ((uint32_t)tunuino_device_u16(p)<<16)|tunuino_device_u16(p+2);}
static void preset(uint8_t *p,unsigned channel){p[0]=(uint8_t)channel;put(p+1,300);put(p+3,3500);put(p+5,0);put(p+7,10000);}
static void query(uint8_t out[15]){
    tunuino_identity id={TUNUINO_PROTEUS,TUNUINO_STM32_F4,7,0,2,0,463,{42}};
    tunuino_dispatch_context context={&id,&device.live,&device.ve,&device.ignition,&device};
    uint8_t request[32],response[64];size_t n,w;const uint8_t *body;
    assert(tunuino_frame_encode((const uint8_t *)"TUNUINO:K",9,request,sizeof(request),&n)==TUNUINO_FRAME_OK);
    assert(tunuino_dispatch_reply(&context,request,n,response,sizeof(response),&w)==TUNUINO_DISPATCH_OK);
    assert(tunuino_frame_decode(response,w,&body,&n)==TUNUINO_FRAME_OK&&n==15);memcpy(out,body,15);
}
int main(void){
    tunuino_device_init(&device,NULL,NULL,NULL);assert(tunuino_device_validate(device.active,sizeof(device.active)));
    uint8_t original[TUNUINO_IO_SETTINGS_SIZE],config[TUNUINO_IO_SETTINGS_SIZE];memcpy(original,device.active,sizeof(original));memcpy(config,original,sizeof(config));
    config[38]=1;preset(config+39,7);preset(config+48,8);assert(tunuino_device_validate(config,sizeof(config)));assert(tunuino_device_validate(config,TUNUINO_IO_LEGACY_SIZE));
    for(unsigned at=0;at<sizeof(config);++at)if(at<38||at>=64)assert(config[at]==original[at]);
    uint8_t bad[TUNUINO_IO_SETTINGS_SIZE];memcpy(bad,config,sizeof(bad));bad[38]=2;assert(tunuino_device_validation_errors(bad,sizeof(bad))&TUNUINO_CONFIG_PRESSURE);
    for(unsigned at=57;at<64;++at){memcpy(bad,config,sizeof(bad));bad[at]=1;assert(!tunuino_device_validate(bad,sizeof(bad)));}
    for(unsigned sensor=0;sensor<2;++sensor){unsigned at=39+9*sensor;memcpy(bad,config,sizeof(bad));bad[at]=12;assert(!tunuino_device_validate(bad,sizeof(bad)));memcpy(bad,config,sizeof(bad));put(bad+at+3,300);assert(!tunuino_device_validate(bad,sizeof(bad)));memcpy(bad,config,sizeof(bad));put(bad+at+7,20001);assert(!tunuino_device_validate(bad,sizeof(bad)));}
    memcpy(bad,config,sizeof(bad));bad[48]=7;assert(!tunuino_device_validate(bad,sizeof(bad)));
    memcpy(bad,config,sizeof(bad));bad[6]=7;put(bad+16,50000);assert(!tunuino_device_validate(bad,sizeof(bad)));
    assert(tunuino_device_restore(&device,config,sizeof(config)));uint16_t raw[16]={0};raw[6]=1900;raw[7]=3500;tunuino_device_sample(&device,100,raw,true,0,false,0,0);
    uint8_t answer[15];query(answer);assert(answer[0]==0&&answer[1]==1&&answer[2]==3&&u32(answer+3)==500000&&u32(answer+7)==1000000&&u32(answer+11)==device.generation);
    raw[6]=299;tunuino_device_sample(&device,101,raw,true,0,false,0,0);query(answer);assert(answer[2]==2&&u32(answer+3)==0);raw[6]=3501;tunuino_device_sample(&device,102,raw,true,0,false,0,0);query(answer);assert(answer[2]==2);
    raw[6]=300;tunuino_device_sample(&device,103,raw,true,0,false,0,0);query(answer);assert(answer[2]==3&&u32(answer+3)==0);
    tunuino_device_sample(&device,104,raw,false,0,false,0,0);query(answer);assert(answer[2]==0);
    tunuino_device_sample(&device,105,raw,true,0,false,0,0);device.dirty=true;query(answer);assert(answer[2]==0);device.dirty=false;
    assert(tunuino_device_restore(&device,config,sizeof(config)));query(answer);assert(answer[2]==0); /* old sample cannot acquire a new generation */
    uint32_t untouched=123;assert(!tunuino_pressure_read(config+38,2,raw,true,&untouched)&&untouched==123);
    config[39]=0;assert(tunuino_device_restore(&device,config,sizeof(config)));tunuino_device_sample(&device,106,raw,true,0,false,0,0);query(answer);assert(answer[2]==2);
    assert(tunuino_device_restore(&device,original,sizeof(original)));query(answer);assert(answer[2]==0);
    puts("Pressure v0/v1, bounds, channel reservations, calibrated native readings, ADC faults, disabled channels and generation isolation passed (host simulation)");
}
