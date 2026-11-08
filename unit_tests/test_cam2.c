#include "tunuino/device.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static tunuino_device d;
int main(void){
 tunuino_device_init(&d,NULL,NULL,NULL);uint8_t p[TUNUINO_IO_SETTINGS_SIZE];memcpy(p,d.active,sizeof(p));uint8_t *c=p+57;
 assert(tunuino_device_validate(p,sizeof(p)));c[0]=1;c[1]=1;c[2]=1;c[3]=3;c[4]=3;c[5]=232;assert(tunuino_device_validate(p,sizeof(p)));assert(!p[38]);
 for(unsigned i=0;i<7;++i){if(i==5)continue;uint8_t save=c[i];c[i]=255;assert(!tunuino_device_validate(p,sizeof(p)));c[i]=save;}
 c[4]=0;c[5]=99;assert(!tunuino_device_validate(p,sizeof(p)));c[4]=19;c[5]=137;assert(!tunuino_device_validate(p,sizeof(p)));c[4]=3;c[5]=232;
 tunuino_vss_capture capture,other;tunuino_vss_tracker tracker,other_tracker;tunuino_vss_reset(&capture,&tracker,3);tunuino_vss_reset(&other,&other_tracker,0);
 tunuino_vss_edge(&capture,1000);tunuino_vss_reading r=tunuino_cam2_sample(&tracker,&capture,c,1000);assert(!r.valid&&r.reason==2);
 tunuino_vss_edge(&capture,1005);assert(capture.count==1);tunuino_vss_edge(&other,6000);assert(other.count==1&&capture.count==1);
 tunuino_vss_edge(&capture,11000);r=tunuino_cam2_sample(&tracker,&capture,c,11000);assert(r.valid==1&&r.reason==0&&r.frequency_milli==100000&&r.period_us==10000&&r.speed_milli==0);
 r=tunuino_cam2_sample(&tracker,&capture,c,1011001);assert(!r.valid&&r.reason==3);r=tunuino_cam2_sample(&tracker,&capture,c,UINT64_C(4294978296));assert(!r.valid);
 tunuino_vss_edge(&capture,(uint32_t)UINT64_C(4294988296));r=tunuino_cam2_sample(&tracker,&capture,c,UINT64_C(4294988296));assert(!r.valid);
 tunuino_vss_edge(&capture,(uint32_t)UINT64_C(4294998296));r=tunuino_cam2_sample(&tracker,&capture,c,UINT64_C(4294998296));assert(r.valid==1);
 assert(tunuino_device_restore(&d,p,sizeof(p)));d.cam2=r;d.cam2_generation=d.generation;
 tunuino_identity id={TUNUINO_PROTEUS,TUNUINO_STM32_F4,7,0,2,0,8207,{42}};tunuino_dispatch_context ctx={&id,&d.live,&d.ve,&d.ignition,&d};uint8_t q[32],out[128];size_t qn,n,bn;const uint8_t *b;
 assert(tunuino_frame_encode((const uint8_t *)"TUNUINO:J",9,q,sizeof(q),&qn)==TUNUINO_FRAME_OK);assert(tunuino_dispatch_reply(&ctx,q,qn,out,sizeof(out),&n)==TUNUINO_DISPATCH_OK);assert(tunuino_frame_decode(out,n,&b,&bn)==TUNUINO_FRAME_OK&&bn==28&&b[2]==1&&b[3]==0&&tunuino_vss_u32(b+4)==100000);
 d.dirty=true;assert(tunuino_dispatch_reply(&ctx,q,qn,out,sizeof(out),&n)==TUNUINO_DISPATCH_OK);assert(tunuino_frame_decode(out,n,&b,&bn)==TUNUINO_FRAME_OK&&!b[2]&&b[3]==6);d.dirty=false;
 assert(tunuino_device_restore(&d,p,sizeof(p)));assert(tunuino_dispatch_reply(&ctx,q,qn,out,sizeof(out),&n)==TUNUINO_DISPATCH_OK);assert(tunuino_frame_decode(out,n,&b,&bn)==TUNUINO_FRAME_OK&&!b[2]);
 /* Initializing pressure while CAM2 is present preserves the independent extension. */
 uint8_t original_cam2[7];memcpy(original_cam2,c,7);p[38]=1;for(unsigned at=39;at<=48;at+=9){p[at+3]=15;p[at+4]=255;p[at+7]=39;p[at+8]=16;}assert(tunuino_device_validate(p,sizeof(p)));assert(!memcmp(p+57,original_cam2,7));
 puts("CAM2 configuration, independent capture, blanking, stale/wrap, read-only generation and pressure compatibility passed (host simulation)");
}
