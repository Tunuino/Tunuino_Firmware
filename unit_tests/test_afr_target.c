#include "tunuino/device.h"
#include "tunuino/engine_features.h"
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
static void grid(uint8_t *p){
 for(unsigned i=0;i<16;++i){put(p,2*i,500+1000*i);put(p,32+2*i,2000+1000*i);}
 for(unsigned r=0;r<16;++r)for(unsigned c=0;c<16;++c)put(p,64+2*(r*16+c),800+10*r+5*c);
}
static bool target_reply(tunuino_device *d,const tunuino_identity *id,uint8_t *body,size_t *n){uint8_t packet[300];size_t size;const uint8_t *payload;if(tunuino_device_reply(d,id,(const uint8_t*)"TUNUINO:L",9,packet,sizeof(packet),&size)!=TUNUINO_DISPATCH_OK||tunuino_frame_decode(packet,size,&payload,n)!=TUNUINO_FRAME_OK)return false;memcpy(body,payload,*n);return true;}
int main(void){
 uint8_t page[576]={0};uint16_t target;
 assert(tunuino_afr_validate(page)&&!tunuino_afr_lookup(page,0,0,&target));grid(page);
 assert(tunuino_afr_validate(page));
 for(unsigned r=0;r<16;++r)for(unsigned c=0;c<16;++c){assert(tunuino_afr_lookup(page,(500+1000*c)*1000,(20+10*r)*1000,&target));assert(target==800+10*r+5*c);}
 assert(tunuino_afr_lookup(page,1000000,25000,&target)&&target==808);
 assert(tunuino_afr_lookup(page,0,0,&target)&&target==800);
 assert(tunuino_afr_lookup(page,UINT32_MAX,UINT32_MAX,&target)&&target==1025);
 put(page,2,500);assert(!tunuino_afr_validate(page));grid(page);put(page,32,50001);assert(!tunuino_afr_validate(page));grid(page);
 put(page,64,649);assert(!tunuino_afr_lookup(page,500000,20000,&target));put(page,64,1301);assert(!tunuino_afr_validate(page));grid(page);
 tunuino_device d;tunuino_device_init(&d,NULL,NULL,NULL);uint8_t config[TUNUINO_IO_SETTINGS_SIZE];memcpy(config,d.active,sizeof(config));
    config[6]=1;config[7]=2;config[8]=1;put(config,14,1000);put(config,16,25000);put(config,22,500);put(config,24,2000);put(config,64,63536);put(config,66,4000);put(config,68,10000);
    uint8_t *profile=config+128;profile[0]=1;profile[1]=1;put(profile,2,5000);put(profile,4,2000);put(profile,6,1000);put(profile,8,15000);put(profile,12,9000);put(profile,14,8500);
    uint8_t *aux=config+TUNUINO_IO_AUX_OFFSET;extension(aux);aux[0]=1;aux[5]=3;put(aux,8,4095);put(aux,10,100);put(aux,12,30);put(aux,16,10000);put(aux,20,500);put(aux,22,2000);
    const unsigned temps[]={63536,4000,10000},rpms[]={1000,900,850};for(unsigned i=0;i<3;++i){put(aux,28+2*i,temps[i]);put(aux,34+2*i,rpms[i]);}
    put(aux,46,400);put(aux,50,9500);put(aux,54,10000);put(aux,56,20000);put(aux,58,1000);put(aux,60,1000);put(aux,66,200);

 put(aux,70,TUNUINO_FEATURE_TARGET);assert(tunuino_device_validate(config,sizeof(config)));
 assert(tunuino_device_restore(&d,config,TUNUINO_IO_V4_SIZE));assert(tunuino_afr_empty(d.active+TUNUINO_IO_AFR_OFFSET));assert(tunuino_device_u16(d.active+TUNUINO_IO_AUX_OFFSET+70)==TUNUINO_FEATURE_TARGET);
 put(aux,70,TUNUINO_FEATURE_TARGET|TUNUINO_FEATURE_TARGET_TABLE);
 assert(!tunuino_device_validate(config,sizeof(config)));assert(!tunuino_device_validate(config,TUNUINO_IO_V4_SIZE));
 memcpy(config+TUNUINO_IO_AFR_OFFSET,page,576);assert(tunuino_device_validate(config,sizeof(config)));
 tunuino_feature_config features;assert(tunuino_features_decode(aux,&features));features.target_table=config+TUNUINO_IO_AFR_OFFSET;
 tunuino_feature_state state={0};tunuino_feature_inputs in={0};
 assert(tunuino_features_update(&features,&state,&in,0,1000000,25000,true,80000,400,1000,2000));assert(state.target==808&&state.multiplier==10000000/808);
 features.target_table=NULL;assert(!tunuino_features_update(&features,&state,&in,0,1000000,25000,true,80000,400,1000,2000));
 features.flags=TUNUINO_FEATURE_TARGET;assert(tunuino_features_update(&features,&state,&in,0,1000000,25000,true,80000,400,1000,2000)&&state.target==1000&&state.multiplier==10000);
 assert(tunuino_device_restore(&d,config,sizeof(config)));d.live.valid_mask=3;d.live.rpm_milli=1000000;d.live.map_milli_kpa=25000;
 tunuino_identity id={TUNUINO_PROTEUS,TUNUINO_STM32_F4,7,0,2,0,TUNUINO_IO_CAPABILITIES,{42}};uint8_t out[300];size_t n;
 tunuino_identity minimal=id;minimal.capabilities&=~(TUNUINO_DISPATCH_SCOPE|TUNUINO_DISPATCH_SERVICE);tunuino_dispatch_context context={&minimal,&d.live,&d.ve,&d.ignition,&d};assert(tunuino_dispatch_validate(&context)==TUNUINO_DISPATCH_OK);
 assert(target_reply(&d,&id,out,&n)&&n==9&&out[2]==3&&tunuino_device_u16(out+3)==808);
 assert(memcmp(d.active,config,sizeof(config))==0&&!d.dirty);d.live.valid_mask=2;
 assert(target_reply(&d,&id,out,&n)&&out[2]==2);
 d.live.valid_mask=3;d.dirty=true;assert(target_reply(&d,&id,out,&n)&&out[2]==2);
 puts("AFR grid bounds, nodes, bilinear lookup, clamping, legacy mode, fuel multiplier and native target provenance passed (host simulation)");return 0;
}
