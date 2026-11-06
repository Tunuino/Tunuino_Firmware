/* Copyright (c) 2026 Tunuino. Consumer-thread scope; no ISR or output changes. */
#include "tunuino/trigger_scope.h"
#include <string.h>
static uint32_t u32(const uint8_t *p){return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
static void put32(uint8_t *p,uint32_t v){p[0]=(uint8_t)(v>>24);p[1]=(uint8_t)(v>>16);p[2]=(uint8_t)(v>>8);p[3]=(uint8_t)v;}
void tunuino_scope_reset(tunuino_trigger_scope *s,uint32_t generation){uint32_t token=s->token+1u;memset(s,0,sizeof(*s));s->generation=generation;s->token=token?token:1u;}
void tunuino_scope_record(void *user,tunuino_capture_event event){
    tunuino_trigger_scope *s=user;if(!s||!s->generation||(event.kind!=TUNUINO_CAPTURE_CRANK&&event.kind!=TUNUINO_CAPTURE_CAM))return;
    if(s->count){unsigned previous=(s->next+TUNUINO_SCOPE_CAPACITY-1u)%TUNUINO_SCOPE_CAPACITY;if(event.time_us<s->ring[previous].time_us){s->count=0;s->next=0;s->valid=false;}}
    s->ring[s->next]=event;s->next=(s->next+1u)%TUNUINO_SCOPE_CAPACITY;if(s->count<TUNUINO_SCOPE_CAPACITY)++s->count;
}
bool tunuino_scope_reply(tunuino_trigger_scope *s,bool healthy,const uint8_t *q,size_t length,uint8_t *out,size_t capacity,size_t *written){
    if(!s||!q||!out||!written||capacity<21)return false;
    unsigned offset=0,count=0;
    if(length==1&&q[0]==0){
        if(++s->token==0)s->token=1;
        s->valid=healthy&&s->count>0;s->frozen_count=s->valid?s->count:0;
        for(unsigned i=0;i<s->frozen_count;++i)s->frozen[i]=s->ring[(s->next+TUNUINO_SCOPE_CAPACITY-s->count+i)%TUNUINO_SCOPE_CAPACITY];
    }else if(length==8&&q[0]==1){
        offset=(unsigned)q[5]<<8|q[6];count=q[7];
        if(!healthy||!s->valid||u32(q+1)!=s->token||!count||count>32||offset>s->frozen_count||count>s->frozen_count-offset||capacity<21u+count*5u)return false;
    }else return false;
    memset(out,0,21u+count*5u);out[1]=1;out[2]=s->valid;out[3]=(uint8_t)(s->frozen_count>>8);out[4]=(uint8_t)s->frozen_count;put32(out+5,s->generation);put32(out+9,s->token);
    const uint64_t origin=s->frozen_count?s->frozen[0].time_us:0;put32(out+13,(uint32_t)(origin>>32));put32(out+17,(uint32_t)origin);
    for(unsigned i=0;i<count;++i){const tunuino_capture_event event=s->frozen[offset+i];uint64_t delta=event.time_us-origin;if(delta>UINT32_MAX)return false;put32(out+21+5*i,(uint32_t)delta);out[25+5*i]=(uint8_t)event.kind;}
    *written=21u+count*5u;return true;
}
