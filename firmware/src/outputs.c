/* Copyright (c) 2026 Tunuino. Bounded pulse timing independent of USB. */
#include "tunuino/outputs.h"
#include <string.h>
static bool due(uint32_t now,uint32_t deadline){return (uint32_t)(now-deadline)<UINT32_C(0x80000000);}
static void level(tunuino_outputs *o,unsigned ch,bool high){
    uint32_t bit=UINT32_C(1)<<ch;
    if(((o->active_mask&bit)!=0)!=high){
        if(high)o->active_mask|=bit;else o->active_mask&=~bit;
        o->write(o->user,ch,high);
    }
}
void tunuino_outputs_init(tunuino_outputs *o,tunuino_output_write write,void *user){
    memset(o,0,sizeof(*o));o->write=write;o->user=user;
    for(unsigned i=0;i<TUNUINO_OUTPUT_COUNT;++i)write(user,i,false);
}
void tunuino_outputs_stop(tunuino_outputs *o,uint8_t fault){
    for(unsigned i=0;i<TUNUINO_OUTPUT_COUNT;++i){level(o,i,false);o->pulse[i].state=0;}
    memset(o->pwm,0,sizeof(o->pwm));
    o->armed=0;o->fault|=fault;
}
void tunuino_outputs_arm(tunuino_outputs *o,uint32_t now,uint32_t lease){
    tunuino_outputs_stop(o,0);o->fault=0;
    if(lease==0||lease>1000000)return;
    o->armed=1;o->lease=now+lease;
}
bool tunuino_outputs_keepalive(tunuino_outputs *o,uint32_t now,uint32_t lease){
    if(!o->armed||lease==0||lease>1000000)return false;
    if(due(now,o->lease)){tunuino_outputs_stop(o,TUNUINO_OUTPUT_WATCHDOG);return false;}
    o->lease=now+lease;return true;
}
bool tunuino_outputs_pulse(tunuino_outputs *o,unsigned ch,uint32_t now,uint32_t start,uint32_t duration){
    if(!o->armed||ch>=TUNUINO_OUTPUT_COUNT||o->pulse[ch].state||(ch<16&&o->pwm[ch].enabled)||duration==0
        ||duration>(ch<16?20000u:5000u)||(uint32_t)(start-now)>1000000u
        ||(o->active_mask&(UINT32_C(1)<<ch)))return false;
    if(due(now,o->lease)){tunuino_outputs_stop(o,TUNUINO_OUTPUT_WATCHDOG);return false;}
    o->pulse[ch]=(tunuino_output_pulse){start,start+duration,1};return true;
}
bool tunuino_outputs_level(tunuino_outputs *o,unsigned ch,bool high){
    if(!o->armed||ch>=16||o->pulse[ch].state||o->pwm[ch].enabled)return false;
    level(o,ch,high);return true;
}
void tunuino_outputs_cancel_injector(tunuino_outputs *o,unsigned ch){
    if(ch<16&&o->pulse[ch].state){o->pulse[ch].state=0;level(o,ch,false);}
}
bool tunuino_outputs_pwm(tunuino_outputs *o,unsigned ch,uint32_t now,uint16_t hz,uint16_t duty){
    if(!o->armed||ch>=16||o->pulse[ch].state||hz<10||hz>300||duty>10000)return false;
    if(due(now,o->lease)){tunuino_outputs_stop(o,TUNUINO_OUTPUT_WATCHDOG);return false;}
    tunuino_output_pwm *p=&o->pwm[ch];uint32_t period=(1000000u+hz/2u)/hz;
    if(p->enabled&&p->period!=period)return false;
    uint32_t high=(uint32_t)(((uint64_t)period*duty+5000u)/10000u);
    if(duty&&high<10u)high=10u;
    if(duty<10000&&period-high<10u)high=period-10u;
    if(!p->enabled){
        if(o->active_mask&(UINT32_C(1)<<ch))return false;
        *p=(tunuino_output_pwm){period,now+1000u,0,0,high,1,0};
    }else if(!high&&p->requested){level(o,ch,false);p->phase=0;p->next=now+period;}
    p->requested=high;return true;
}
bool tunuino_outputs_aux_test(tunuino_outputs *o,unsigned ch,uint32_t now,uint32_t duration){
    if(o->armed||ch>=16||duration<1000||duration>800000)return false;
    tunuino_outputs_arm(o,now,900000);
    o->pulse[ch]=(tunuino_output_pulse){now+1000,now+1000+duration,1};
    return true;
}
void tunuino_outputs_poll(tunuino_outputs *o,uint32_t now){
    if(!o->armed)return;
    if(due(now,o->lease)){tunuino_outputs_stop(o,TUNUINO_OUTPUT_WATCHDOG);return;}
    /* Inspect every due edge BEFORE energizing any channel. Never replay a
     * missed dwell or injector pulse after an IRQ/flash/CPU stall. */
    for(unsigned i=0;i<TUNUINO_OUTPUT_COUNT;++i){
        tunuino_output_pulse *p=&o->pulse[i];
        uint32_t edge=p->state==1?p->start:p->end;
        if(p->state&&due(now,edge)&&(uint32_t)(now-edge)>100u){
            tunuino_outputs_stop(o,TUNUINO_OUTPUT_LATE);return;
        }
    }
    for(unsigned i=0;i<16;++i){
        tunuino_output_pwm *p=&o->pwm[i];
        if(p->enabled&&due(now,p->next)&&(uint32_t)(now-p->next)>100u){
            tunuino_outputs_stop(o,TUNUINO_OUTPUT_LATE);return;
        }
    }
    for(unsigned i=0;i<TUNUINO_OUTPUT_COUNT;++i){
        tunuino_output_pulse *p=&o->pulse[i];
        if(p->state==2&&due(now,p->end)){level(o,i,false);p->state=0;}
    }
    for(unsigned i=0;i<16;++i){
        tunuino_output_pwm *p=&o->pwm[i];
        if(!p->enabled||!due(now,p->next))continue;
        if(p->phase){level(o,i,false);p->phase=0;p->next=p->cycle+p->period;}
        else{
            p->cycle=p->next;p->high=p->requested;
            level(o,i,p->high!=0);
            if(p->high&&p->high<p->period){p->phase=1;p->next=p->cycle+p->high;}
            else p->next=p->cycle+p->period;
        }
    }
    for(unsigned i=0;i<TUNUINO_OUTPUT_COUNT;++i){
        tunuino_output_pulse *p=&o->pulse[i];
        if(p->state==1&&due(now,p->start)){
            if(due(now,p->end)){tunuino_outputs_stop(o,TUNUINO_OUTPUT_LATE);return;}
            level(o,i,true);p->state=2;
        }
    }
}
uint32_t tunuino_outputs_next(const tunuino_outputs *o,uint32_t now){
    if(!o->armed)return 0;
    uint32_t delay=due(now,o->lease)?1u:o->lease-now;
    for(unsigned i=0;i<TUNUINO_OUTPUT_COUNT;++i){
        const tunuino_output_pulse *p=&o->pulse[i];
        uint32_t edge=p->state==1?p->start:p->end;
        if(p->state){uint32_t d=due(now,edge)?1u:edge-now;if(d<delay)delay=d;}
    }
    for(unsigned i=0;i<16;++i)if(o->pwm[i].enabled){
        uint32_t d=due(now,o->pwm[i].next)?1u:o->pwm[i].next-now;if(d<delay)delay=d;
    }
    return delay?delay:1;
}
uint32_t tunuino_outputs_pending(const tunuino_outputs *o){
    uint32_t result=0;
    for(unsigned i=0;i<TUNUINO_OUTPUT_COUNT;++i)if(o->pulse[i].state||(i<16&&o->pwm[i].enabled))result|=UINT32_C(1)<<i;
    return result;
}
