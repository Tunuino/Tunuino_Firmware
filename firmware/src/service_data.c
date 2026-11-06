/* Copyright (c) 2026 Tunuino. VIN is separate from projects/calibration pages. */
#include "tunuino/service_data.h"
#include "tunuino/aux_control.h"
#include <string.h>
static uint16_t u16(const uint8_t *p){return (uint16_t)((uint16_t)p[0]<<8|p[1]);}
static uint32_t u32(const uint8_t *p){return (uint32_t)u16(p)<<16|u16(p+2);}
static uint64_t u64(const uint8_t *p){return (uint64_t)u32(p)<<32|u32(p+4);}
static void put16(uint8_t *p,uint16_t n){p[0]=(uint8_t)(n>>8);p[1]=(uint8_t)n;}
static void put32(uint8_t *p,uint32_t n){put16(p,(uint16_t)(n>>16));put16(p+2,(uint16_t)n);}
static void put64(uint8_t *p,uint64_t n){put32(p,(uint32_t)(n>>32));put32(p+4,(uint32_t)n);}
static bool zero(const uint8_t *p,size_t n){for(size_t i=0;i<n;++i)if(p[i])return false;return true;}
static bool text(const uint8_t *p,size_t length){
    unsigned characters=0;for(size_t i=0;i<length;){uint32_t cp;unsigned n;
        if(p[i]<128){cp=p[i];n=1;}else if(p[i]>=0xc2&&p[i]<=0xdf){cp=p[i]&31u;n=2;}
        else if(p[i]>=0xe0&&p[i]<=0xef){cp=p[i]&15u;n=3;}else if(p[i]>=0xf0&&p[i]<=0xf4){cp=p[i]&7u;n=4;}else return false;
        if(i+n>length)return false;
        for(unsigned k=1;k<n;++k){if((p[i+k]&0xc0u)!=0x80u)return false;cp=(cp<<6)|(p[i+k]&63u);}
        if(cp<32||(cp>=127&&cp<160)||(n==2&&cp<128)||(n==3&&cp<2048)||(n==4&&cp<65536)
            ||cp>0x10ffffu||(cp>=0xd800&&cp<=0xdfff)||++characters>64)return false;
        i+=n;
    }return true;
}
bool tunuino_service_validate(const uint8_t b[TUNUINO_SERVICE_SIZE]){
    if(!b)return false;
    if(zero(b,TUNUINO_SERVICE_SIZE))return true;
    if(b[0]!=1||b[1]>1||b[19]||b[295]||(b[294]&~15u)||!zero(b+316,8)||!zero(b+332,180))return false;
    for(unsigned i=2;i<19;++i){if(!b[1]){if(b[i])return false;}else if(!((b[i]>='0'&&b[i]<='9')||(b[i]>='A'&&b[i]<='Z'&&b[i]!='I'&&b[i]!='O'&&b[i]!='Q')))return false;}
    unsigned length=u16(b+20);if(length>256||!text(b+22,length)||!zero(b+22+length,256-length))return false;
    if((b[294]&1)&&u32(b+282)>20000000u)return false;
    if((b[294]&2)&&u32(b+286)>500000u)return false;
    if((b[294]&4)&&((int32_t)u32(b+290)<-40000||(int32_t)u32(b+290)>150000))return false;
    if((b[294]&8)&&((int32_t)u32(b+296)<-4000||(int32_t)u32(b+296)>15000))return false;
    return true;
}
void tunuino_service_init(tunuino_service_data *s,const uint8_t b[TUNUINO_SERVICE_SIZE]){
    memset(s,0,sizeof(*s));if(b&&tunuino_service_validate(b))memcpy(s->current,b,TUNUINO_SERVICE_SIZE);
    if(s->current[0]==0)s->current[0]=1;
    memcpy(s->persisted,s->current,TUNUINO_SERVICE_SIZE);
}
bool tunuino_service_pending(const tunuino_service_data *s){return memcmp(s->current,s->persisted,TUNUINO_SERVICE_SIZE)!=0;}
void tunuino_service_prepare(const tunuino_service_data *s,uint64_t now,uint8_t next[TUNUINO_SERVICE_SIZE]){
    memcpy(next,s->current,TUNUINO_SERVICE_SIZE);if(tunuino_service_pending(s)){uint32_t revision=u32(next+278)+1u;if(!revision)revision=1;put32(next+278,revision);put64(next+324,now);}
}
void tunuino_service_committed(tunuino_service_data *s,const uint8_t b[TUNUINO_SERVICE_SIZE]){memcpy(s->current,b,TUNUINO_SERVICE_SIZE);memcpy(s->persisted,b,TUNUINO_SERVICE_SIZE);s->uncertain=false;}
void tunuino_service_sample(tunuino_service_data *s,const tunuino_device *d,uint64_t now,uint8_t fault){
    if(s->uncertain)return;
    uint8_t *b=s->current,mask=b[294];
    if(d->live.valid_mask&TUNUINO_LIVE_RPM_VALID){if(!(mask&1)||d->live.rpm_milli>u32(b+282))put32(b+282,d->live.rpm_milli);mask|=1;}
    if(d->live.valid_mask&TUNUINO_LIVE_MAP_VALID){if(!(mask&2)||d->live.map_milli_kpa>u32(b+286))put32(b+286,d->live.map_milli_kpa);mask|=2;}
    if(d->live.valid_mask&TUNUINO_LIVE_COOLANT_VALID){if(!(mask&4)||d->live.coolant_milli_c>(int32_t)u32(b+290))put32(b+290,(uint32_t)d->live.coolant_milli_c);mask|=4;}
    if(s->cached_generation!=d->generation){memset(&s->features,0,sizeof(s->features));(void)tunuino_features_decode(d->active+TUNUINO_IO_AUX_OFFSET,&s->features);s->cached_generation=d->generation;}
    const uint8_t *a=d->active+TUNUINO_IO_AUX_OFFSET;tunuino_aux_config aux={0};aux.tps_channel=a[0]==1?a[5]:0;aux.tps_closed=u16(a+6);aux.tps_open=u16(a+8);aux.tps_tolerance=u16(a+66);
    uint16_t tps=0;bool tv=tunuino_aux_tps(&aux,d->adc_valid,d->raw,&tps);tunuino_feature_inputs inputs;
    tunuino_features_sample(&s->features,d->adc_valid,d->raw,tv,tps,false,0,&inputs);
    if(inputs.iat_valid){if(!(mask&8)||inputs.iat>(int32_t)u32(b+296))put32(b+296,(uint32_t)inputs.iat);mask|=8;}
    if(s->have_sample&&s->previous_full&&now>=s->last_sample_ms&&now-s->last_sample_ms<=2000&&tv&&tps>=9500
        &&(d->live.valid_mask&TUNUINO_LIVE_RPM_VALID)&&d->live.rpm_milli>0){
        const uint64_t old=u64(b+300),delta=now-s->last_sample_ms;put64(b+300,UINT64_MAX-old<delta?UINT64_MAX:old+delta);
    }
    if(d->trigger_errors>u32(b+308))put32(b+308,d->trigger_errors);
    put32(b+312,u32(b+312)|fault);b[294]=mask;s->last_sample_ms=now;s->have_sample=true;s->previous_full=tv&&tps>=9500&&(d->live.valid_mask&TUNUINO_LIVE_RPM_VALID)&&d->live.rpm_milli>0;
}
bool tunuino_service_reply(tunuino_service_data *s,uint32_t generation,uint64_t now,bool writable,
    tunuino_service_save save,void *user,const uint8_t *q,size_t length,uint8_t *out,size_t cap,size_t *written){
    if(written)*written=0;
    if(!s||!q||!length||!out||!written||cap<21||!generation)return false;
    unsigned offset=0,count=0;
    if(q[0]==0&&length==1){if(++s->token==0)s->token=1;memcpy(s->frozen,s->current,TUNUINO_SERVICE_SIZE);s->frozen_generation=generation;s->frozen_dirty=tunuino_service_pending(s);}
    else if(q[0]==1&&length==8){offset=u16(q+5);count=q[7];if(u32(q+1)!=s->token||generation!=s->frozen_generation||!s->token||!count||count>128||offset>512||count>512-offset||cap<21+count)return false;}
    else if(q[0]>=2&&q[0]<=5&&length>=5){
        if(!writable){out[0]=0x86;*written=1;return true;}
        if(!save||s->uncertain){out[0]=s->uncertain?0x80:0x84;*written=1;return true;}
        if(u32(q+1)!=u32(s->current+278)){out[0]=0x85;*written=1;return true;}
        uint8_t previous[TUNUINO_SERVICE_SIZE];memcpy(previous,s->current,sizeof(previous));
        if(q[0]==2){if(length<6||(q[5]!=0&&q[5]!=17)||length!=6u+q[5])return false;s->current[1]=q[5]?1:0;memset(s->current+2,0,17);if(q[5])memcpy(s->current+2,q+6,17);}
        else if(q[0]==3){if(length<7||u16(q+5)>256||length!=7u+u16(q+5))return false;put16(s->current+20,u16(q+5));memset(s->current+22,0,256);memcpy(s->current+22,q+7,u16(q+5));}
        else if(q[0]==4){if(length!=10||memcmp(q+5,"RESET",5)!=0)return false;memset(s->current+282,0,230);s->have_sample=false;s->previous_full=false;}
        else if(length!=5)return false;
        if(!tunuino_service_validate(s->current)){memcpy(s->current,previous,sizeof(previous));return false;}
        uint8_t next[TUNUINO_SERVICE_SIZE];tunuino_service_prepare(s,now,next);
        if(!save(user,next)){memcpy(s->current,previous,sizeof(previous));s->uncertain=true;out[0]=0x80;*written=1;return true;}
        tunuino_service_committed(s,next);out[0]=0;put32(out+1,u32(next+278));*written=5;return true;
    }else return false;
    memset(out,0,21+count);out[1]=1;out[2]=(uint8_t)((s->frozen_dirty?1:0)|(s->uncertain?2:0));put32(out+4,u32(s->frozen+278));put32(out+8,s->frozen_generation);put32(out+12,s->token);put16(out+16,512);put16(out+18,(uint16_t)offset);out[20]=(uint8_t)count;memcpy(out+21,s->frozen+offset,count);*written=21+count;return true;
}
