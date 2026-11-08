#include "tunuino/device.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static tunuino_device d;
static void put16(uint8_t *p,unsigned n){p[0]=(uint8_t)(n>>8);p[1]=(uint8_t)n;}
static void put32(uint8_t *p,uint32_t n){put16(p,n>>16);put16(p+2,n);}
static tunuino_vss_reading feed(tunuino_vss_capture *c,tunuino_vss_tracker *s,const uint8_t *p,uint64_t now){tunuino_vss_edge(c,(uint32_t)now);return tunuino_vss_sample(s,c,p,now);}
int main(void){
 tunuino_device_init(&d,NULL,NULL,NULL);uint8_t p[TUNUINO_IO_SETTINGS_SIZE];memcpy(p,d.active,sizeof(p));uint8_t *v=p+82;
 assert(tunuino_device_validate(p,sizeof(p)));v[0]=1;v[1]=1;put32(v+4,1000);put16(v+8,1000);assert(tunuino_device_validate(p,sizeof(p)));assert(p[70]==0);
 for(unsigned i=10;i<14;++i){v[i]=1;assert(!tunuino_device_validate(p,sizeof(p)));v[i]=0;}v[1]=2;assert(!tunuino_device_validate(p,sizeof(p)));v[1]=1;v[2]=2;assert(!tunuino_device_validate(p,sizeof(p)));v[2]=0;v[3]=4;assert(!tunuino_device_validate(p,sizeof(p)));v[3]=0;
 tunuino_vss_capture c;tunuino_vss_tracker s;tunuino_vss_reset(&c,&s,0);assert(!feed(&c,&s,v,1000).valid);assert(!tunuino_vss_sample(&s,&c,v,1001).valid);
 tunuino_vss_reading r=feed(&c,&s,v,11000);assert(r.valid&&r.frequency_milli==100000&&r.speed_milli==100000&&r.period_us==10000);
 r=tunuino_vss_sample(&s,&c,v,1011001);assert(!r.valid&&r.reason==3);
 r=feed(&c,&s,v,UINT64_C(4294978296));assert(!r.valid);r=tunuino_vss_sample(&s,&c,v,UINT64_C(4294978297));assert(!r.valid);
 r=feed(&c,&s,v,UINT64_C(4294988296));assert(r.valid&&r.speed_milli==100000);
 r=tunuino_vss_sample(&s,&c,v,5);assert(!r.valid&&r.reason==5);
 tunuino_vss_reset(&c,&s,3);v[3]=3;assert(!feed(&c,&s,v,1000).valid);tunuino_vss_edge(&c,1001);assert(c.count==1);r=feed(&c,&s,v,11000);assert(r.valid&&r.count==2);
 put32(v+4,250);r=feed(&c,&s,v,21000);assert(r.valid&&r.speed_milli==25000);
 r=feed(&c,&s,v,21020);assert(!r.valid&&r.reason==4);r=feed(&c,&s,v,31020);assert(r.valid);
 put32(v+4,0);r=feed(&c,&s,v,41020);assert(r.valid==1&&r.reason==7&&r.frequency_milli==100000&&!r.speed_milli);put32(v+4,250);
 v[1]=0;r=tunuino_vss_sample(&s,&c,v,31021);assert(!r.valid&&r.reason==1);v[1]=1;
 assert(tunuino_device_restore(&d,p,sizeof(p)));d.vss=r;d.vss.valid=3;d.vss.reason=0;d.vss.speed_milli=25000;d.vss.frequency_milli=100000;d.vss_generation=d.generation;
 tunuino_identity id={TUNUINO_PROTEUS,TUNUINO_STM32_F4,7,0,2,0,4111,{42}};tunuino_dispatch_context ctx={&id,&d.live,&d.ve,&d.ignition,&d};uint8_t q[32],out[128];size_t qn,n,bn;const uint8_t *b;
 assert(tunuino_frame_encode((const uint8_t *)"TUNUINO:U",9,q,sizeof(q),&qn)==TUNUINO_FRAME_OK);assert(tunuino_dispatch_reply(&ctx,q,qn,out,sizeof(out),&n)==TUNUINO_DISPATCH_OK);assert(tunuino_frame_decode(out,n,&b,&bn)==TUNUINO_FRAME_OK&&bn==28&&b[2]==3&&tunuino_vss_u32(b+8)==25000);
 d.dirty=true;assert(tunuino_dispatch_reply(&ctx,q,qn,out,sizeof(out),&n)==TUNUINO_DISPATCH_OK);assert(tunuino_frame_decode(out,n,&b,&bn)==TUNUINO_FRAME_OK&&!b[2]&&b[3]==6);d.dirty=false;
 assert(tunuino_device_restore(&d,p,sizeof(p)));assert(tunuino_dispatch_reply(&ctx,q,qn,out,sizeof(out),&n)==TUNUINO_DISPATCH_OK);assert(tunuino_frame_decode(out,n,&b,&bn)==TUNUINO_FRAME_OK&&!b[2]);
 puts("VSS format, physical input bounds, period/ratio, blanking, stale/clock/wrap isolation, disabled and native reply generation checks passed (host simulation)");
}
