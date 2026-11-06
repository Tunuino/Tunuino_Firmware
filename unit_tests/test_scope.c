#include "tunuino/trigger_scope.h"
#include "tunuino/device.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t u32(const uint8_t *p){return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
static bool hook(void *user,const uint8_t *q,size_t n,uint8_t *out,size_t cap,size_t *written){return tunuino_scope_reply(user,true,q,n,out,cap,written);}
int main(void){
    tunuino_trigger_scope scope={0};uint8_t reply[257],query[8]={0};size_t written;
    tunuino_scope_reset(&scope,7);
    assert(tunuino_scope_reply(&scope,true,query,1,reply,sizeof(reply),&written)&&written==21&&!reply[2]);
    for(unsigned i=0;i<300;++i)tunuino_scope_record(&scope,(tunuino_capture_event){100000+i*1000,i%6?TUNUINO_CAPTURE_CRANK:TUNUINO_CAPTURE_CAM});
    assert(tunuino_scope_reply(&scope,true,query,1,reply,sizeof(reply),&written)&&reply[2]&&reply[3]==1&&reply[4]==0);
    uint8_t header[21];memcpy(header,reply,21);query[0]=1;memcpy(query+1,reply+9,4);query[7]=32;
    tunuino_scope_record(&scope,(tunuino_capture_event){999999,TUNUINO_CAPTURE_CRANK});
    for(unsigned offset=0;offset<256;offset+=32){query[5]=(uint8_t)(offset>>8);query[6]=(uint8_t)offset;
        assert(tunuino_scope_reply(&scope,true,query,8,reply,sizeof(reply),&written)&&written==181);
        assert(memcmp(reply,header,21)==0&&u32(reply+21)==offset*1000&&u32(reply+21+31*5)==(offset+31)*1000);
    }
    assert(!tunuino_scope_reply(&scope,false,query,8,reply,sizeof(reply),&written));
    query[1]^=1;assert(!tunuino_scope_reply(&scope,true,query,8,reply,sizeof(reply),&written));query[1]^=1;
    tunuino_scope_reset(&scope,8);assert(!tunuino_scope_reply(&scope,true,query,8,reply,sizeof(reply),&written));
    query[0]=0;assert(tunuino_scope_reply(&scope,false,query,1,reply,sizeof(reply),&written)&&!reply[2]);
    tunuino_device device;tunuino_device_init(&device,NULL,NULL,NULL);device.scope_command=hook;device.scope_user=&scope;
    tunuino_identity identity={TUNUINO_PROTEUS,TUNUINO_STM32_F4,7,0,2,0,31,{42}};
    tunuino_scope_reset(&scope,device.generation);
    tunuino_scope_record(&scope,(tunuino_capture_event){1000,TUNUINO_CAPTURE_CRANK});
    uint8_t packet[512];const uint8_t *body;size_t body_size;
    assert(tunuino_device_reply(&device,&identity,(const uint8_t*)"TUNUINO:S\0",10,packet,sizeof(packet),&written)==TUNUINO_DISPATCH_OK);
    assert(tunuino_frame_decode(packet,written,&body,&body_size)==TUNUINO_FRAME_OK&&body_size==21&&body[2]==1);
    assert(!device.dirty&&!device.persisted&&device.generation==1);
    tunuino_dispatch_context context={&identity,&device.live,&device.ve,&device.ignition,&device};
    uint8_t request[32];size_t request_size;
    assert(tunuino_frame_encode((const uint8_t*)"TUNUINO:S\0",10,request,sizeof(request),&request_size)==TUNUINO_FRAME_OK);
    assert(tunuino_dispatch_reply(&context,request,request_size,packet,sizeof(packet),&written)==TUNUINO_DISPATCH_OK);
    device.scope_command=NULL;assert(tunuino_dispatch_validate(&context)==TUNUINO_DISPATCH_UNCONFIGURED);
    identity.capabilities=15;assert(tunuino_dispatch_validate(&context)==TUNUINO_DISPATCH_OK);
    puts("scope ordering, ring wrap, frozen chunks, invalidation and read bounds passed");return 0;
}
