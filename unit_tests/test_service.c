#include "tunuino/config_journal.h"
#include "tunuino/service_data.h"
#include "tunuino/link.h"
#include "tunuino/trigger_scope.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
static uint8_t banks[2][TUNUINO_JOURNAL_BANK_BYTES],baseline[2][TUNUINO_JOURNAL_BANK_BYTES];
static int remaining=-1;static unsigned programmed;static bool fail_read,fail_verify;
static tunuino_config_journal journal;static tunuino_device device;static tunuino_service_data service;static uint64_t now;
static uint32_t u32(const uint8_t *p){return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
static void p32(uint8_t *p,uint32_t v){p[0]=(uint8_t)(v>>24);p[1]=(uint8_t)(v>>16);p[2]=(uint8_t)(v>>8);p[3]=(uint8_t)v;}
static void p16(uint8_t *p,unsigned v){p[0]=(uint8_t)(v>>8);p[1]=(uint8_t)v;}
static bool read_flash(void *u,unsigned bank,size_t at,uint8_t *b,size_t n){(void)u;if(fail_read){fail_read=false;return false;}if(bank>1||at+n>TUNUINO_JOURNAL_BANK_BYTES)return false;memcpy(b,banks[bank]+at,n);return true;}
static bool program_flash(void *u,unsigned bank,size_t at,const uint8_t *b,size_t n){(void)u;if(bank>1||at%4||n%4||at+n>TUNUINO_JOURNAL_BANK_BYTES)return false;
    for(size_t i=0;i<n;++i){if(remaining==0)return false;if(remaining>0)--remaining;if((banks[bank][at+i]&b[i])!=b[i])return false;banks[bank][at+i]&=b[i];++programmed;}
    if(n==4&&memcmp(b,"DONE",4)==0&&fail_verify){fail_read=true;fail_verify=false;}return true;}
static bool erase_flash(void *u,unsigned bank){(void)u;if(bank>1)return false;memset(banks[bank],255,TUNUINO_JOURNAL_BANK_BYTES);return true;}
static const tunuino_journal_io io={read_flash,program_flash,erase_flash,NULL};
static bool save_service(void *u,const uint8_t b[TUNUINO_SERVICE_SIZE]){(void)u;now+=5000;return tunuino_journal_save_service(&journal,device.active,b);}
static uint64_t clock_ms(void *u){(void)u;return now;}
static bool service_hook(void *u,const uint8_t *q,size_t n,uint8_t *out,size_t cap,size_t *written){(void)u;return tunuino_service_reply(&service,device.generation,now,true,save_service,NULL,q,n,out,cap,written);}
static bool dummy_scope(void *u,const uint8_t *q,size_t n,uint8_t *out,size_t cap,size_t *written){(void)u;(void)q;(void)n;(void)out;(void)cap;(void)written;return false;}
static void legacy(unsigned format){uint8_t r[TUNUINO_JOURNAL_RECORD_BYTES];size_t settings=format==2?TUNUINO_IO_LEGACY_SIZE:format>=6?TUNUINO_IO_SETTINGS_SIZE:format==5?TUNUINO_IO_V5_SIZE:TUNUINO_IO_V4_SIZE;size_t n=settings+32u+(format>=4?516u:0u);memset(r,255,n);memcpy(r,"TNUIO00",7);r[7]=(uint8_t)('0'+format);p32(r+8,12);p32(r+12,(uint32_t)n-32);if(format>=4)p32(r+16,1);memcpy(r+(format>=4?20:16),device.active,settings);if(format>=4){tunuino_service_data old;tunuino_service_init(&old,NULL);memcpy(r+20+settings,old.current,512);}p32(r+n-16,tunuino_crc32(r,n-16));memcpy(r+n-4,"DONE",4);memcpy(banks[0],r,n);}

int main(void){
    tunuino_device_init(&device,NULL,clock_ms,NULL);uint8_t restored[TUNUINO_IO_SETTINGS_SIZE],metadata[512]={0};
    memset(banks,255,sizeof(banks));assert(!tunuino_journal_init(&journal,&io,restored)&&journal.healthy);
    tunuino_service_init(&service,NULL);memcpy(metadata,service.current,512);
    assert(tunuino_journal_save_service(&journal,device.active,metadata));assert(!journal.calibration_persisted);
    tunuino_config_journal reboot;assert(tunuino_journal_init(&reboot,&io,restored)&&!reboot.calibration_persisted); // Metadata never makes calibration permanent.
    assert(tunuino_journal_save(&journal,device.active)&&journal.calibration_persisted);assert(tunuino_journal_init(&reboot,&io,restored)&&reboot.calibration_persisted);
    for(unsigned format=2;format<=6;++format){memset(banks,255,sizeof(banks));legacy(format);assert(tunuino_journal_init(&journal,&io,restored)&&journal.legacy&&journal.calibration_persisted);for(unsigned i=TUNUINO_IO_AFR_OFFSET;i<sizeof(restored);++i)assert(restored[i]==0);
        if(format>=4){uint8_t original[512];memcpy(original,journal.service,512);memcpy(baseline,banks,sizeof(banks));for(int cut=0;cut<(int)TUNUINO_JOURNAL_RECORD_BYTES;cut+=4){memcpy(banks,baseline,sizeof(banks));remaining=-1;assert(tunuino_journal_init(&journal,&io,restored));remaining=cut;assert(!tunuino_journal_save_service(&journal,restored,metadata));remaining=-1;assert(tunuino_journal_init(&reboot,&io,restored)&&reboot.format==format&&memcmp(reboot.service,original,512)==0);}memcpy(banks,baseline,sizeof(banks));assert(tunuino_journal_init(&journal,&io,restored));}
        metadata[1]=1;memcpy(metadata+2,"1HGBH41JXMN109186",17);
        assert(tunuino_journal_save_service(&journal,restored,metadata)&&journal.format==7&&journal.bank==1);assert(tunuino_journal_init(&reboot,&io,restored)&&memcmp(reboot.service,metadata,512)==0);}
    memcpy(baseline,banks,sizeof(banks));uint8_t next[512];memcpy(next,metadata,512);next[18]='7';
    for(int cut=0;cut<(int)TUNUINO_JOURNAL_RECORD_BYTES;cut+=4){memcpy(banks,baseline,sizeof(banks));remaining=-1;assert(tunuino_journal_init(&journal,&io,restored));remaining=cut;assert(!tunuino_journal_save_service(&journal,device.active,next));remaining=-1;assert(tunuino_journal_init(&reboot,&io,restored)&&memcmp(reboot.service,metadata,512)==0&&memcmp(restored,device.active,sizeof(restored))==0);}
    remaining=-1;memcpy(banks,baseline,sizeof(banks));assert(tunuino_journal_init(&journal,&io,restored));tunuino_service_init(&service,journal.service);
    uint8_t q[300]={2},out[257];size_t written;p32(q+1,u32(service.current+278));q[5]=17;memcpy(q+6,"WVWZZZ1JZXW000001",17);
    assert(tunuino_service_reply(&service,1,100,false,save_service,NULL,q,23,out,sizeof(out),&written)&&out[0]==0x86);
    assert(tunuino_service_reply(&service,1,100,true,save_service,NULL,q,23,out,sizeof(out),&written)&&out[0]==0&&written==5);
    assert(memcmp(service.current+2,q+6,17)==0&&tunuino_journal_init(&reboot,&io,restored)&&memcmp(reboot.service+2,q+6,17)==0);
    assert(tunuino_service_reply(&service,1,100,true,save_service,NULL,q,23,out,sizeof(out),&written)&&out[0]==0x85); // Old revision cannot overwrite data.
    memset(q,0,sizeof(q));q[0]=3;p32(q+1,u32(service.current+278));p16(q+5,128);for(unsigned i=0;i<64;++i){q[7+i*2]=0xc3;q[8+i*2]=0xa9;}
    assert(tunuino_service_reply(&service,1,200,true,save_service,NULL,q,135,out,sizeof(out),&written)&&out[0]==0);
    uint8_t kept[512];memcpy(kept,service.current,512);p32(q+1,u32(service.current+278));p16(q+5,130);q[135]=0xc3;q[136]=0xa9;
    assert(!tunuino_service_reply(&service,1,200,true,save_service,NULL,q,137,out,sizeof(out),&written)&&memcmp(kept,service.current,512)==0);
    device.live.valid_mask=1|2|8;device.live.rpm_milli=5000000;device.live.map_milli_kpa=120000;device.live.coolant_milli_c=-5000;device.trigger_errors=3;
    tunuino_service_sample(&service,&device,1000,4);device.live.coolant_milli_c=95000;tunuino_service_sample(&service,&device,1100,0);
    assert(u32(service.current+282)==5000000&&u32(service.current+290)==95000&&u32(service.current+308)==3&&u32(service.current+312)==4&&tunuino_service_pending(&service));
    memset(q,0,sizeof(q));q[0]=5;p32(q+1,u32(service.current+278));assert(tunuino_service_reply(&service,1,1200,true,save_service,NULL,q,5,out,sizeof(out),&written)&&out[0]==0&&!tunuino_service_pending(&service));
    // Snapshot remains stable while live statistics continue changing.
    q[0]=0;assert(tunuino_service_reply(&service,1,1300,true,save_service,NULL,q,1,out,sizeof(out),&written));uint8_t header[21];memcpy(header,out,21);device.live.rpm_milli=6000000;tunuino_service_sample(&service,&device,1400,0);
    q[0]=1;memcpy(q+1,header+12,4);p16(q+5,256);q[7]=128;assert(tunuino_service_reply(&service,1,1400,true,save_service,NULL,q,8,out,sizeof(out),&written)&&u32(out+21+282-256)==5000000);
    // A committed record with a lost verification response is uncertain; no retry is allowed.
    q[0]=2;p32(q+1,u32(service.current+278));q[5]=17;memcpy(q+6,"1HGBH41JXMN109186",17);fail_verify=true;
    assert(tunuino_service_reply(&service,1,1500,true,save_service,NULL,q,23,out,sizeof(out),&written)&&out[0]==0x80&&service.uncertain);unsigned before=programmed;
    assert(tunuino_service_reply(&service,1,1500,true,save_service,NULL,q,23,out,sizeof(out),&written)&&out[0]==0x80&&programmed==before);
    assert(tunuino_journal_init(&reboot,&io,restored)&&memcmp(reboot.service+2,"1HGBH41JXMN109186",17)==0);tunuino_service_init(&service,reboot.service);journal=reboot;
    // Flash delay starts USB response timing after persistence, preserving the link.
    tunuino_identity id={TUNUINO_PROTEUS,TUNUINO_STM32_F4,7,0,2,0,63,{42}};device.scope_command=dummy_scope;device.service_command=service_hook;
    tunuino_dispatch_context context={&id,&device.live,&device.ve,&device.ignition,&device};tunuino_link link;assert(tunuino_link_init(&link,&context,1000,1000,now)==TUNUINO_LINK_RECEIVING);
    uint8_t body[32]="TUNUINO:N";body[9]=2;p32(body+10,u32(service.current+278));body[14]=17;memcpy(body+15,"WVWZZZ1JZXW000001",17);uint8_t packet[64];size_t packet_size,consumed;
    assert(tunuino_frame_encode(body,32,packet,sizeof(packet),&packet_size)==TUNUINO_FRAME_OK);assert(tunuino_link_feed(&link,packet,packet_size,now,&consumed)==TUNUINO_LINK_RESPONDING);assert(tunuino_link_poll(&link,now)==TUNUINO_LINK_RESPONDING);
    puts("service persistence, legacy migration, torn records, CAS, UTF-8, retained maxima, uncertain writes and slow-flash USB timing passed");return 0;
}
