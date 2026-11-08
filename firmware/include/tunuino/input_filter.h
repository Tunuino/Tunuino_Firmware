/* Copyright (c) 2026 Tunuino. Per-AV first-order low-pass acquisition. */
#ifndef TUNUINO_INPUT_FILTER_H
#define TUNUINO_INPUT_FILTER_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#define TUNUINO_INPUT_FILTER_OFFSET 70u
#define TUNUINO_INPUT_FILTER_SIZE 26u
/* Zero block: legacy acquisition. V1: version, eleven AV filter codes,
 * fourteen reserved zeros. 0 bypass, 1..6 = 50/25/10/5/2/1 Hz. */
typedef struct {float value[11];uint64_t last_ms;bool ready;} tunuino_input_filter;
static inline bool tunuino_input_filter_validate(const uint8_t *p){
    if(!p)return false;
    bool empty=true;for(unsigned i=0;i<26;++i)if(p[i])empty=false;
    if(empty)return true;
    if(p[0]!=1)return false;
    for(unsigned i=1;i<=11;++i)if(p[i]>6)return false;
    for(unsigned i=12;i<26;++i)if(p[i])return false;
    return true;
}
static inline void tunuino_input_filter_apply(tunuino_input_filter *s,const uint8_t *p,uint64_t ms,uint16_t raw[16],bool valid){
    if(!valid||!tunuino_input_filter_validate(p)||p[0]!=1){s->ready=false;return;}
    if(!s->ready||ms<s->last_ms||ms-s->last_ms>100){for(unsigned i=0;i<11;++i)s->value[i]=raw[i];s->ready=true;s->last_ms=ms;return;}
    const uint64_t dt=ms-s->last_ms;
    static const unsigned hz[]={0,50,25,10,5,2,1};
    for(unsigned i=0;i<11;++i){const unsigned code=p[i+1];if(!code)s->value[i]=raw[i];else if(dt){const float alpha=1.0f-expf(-6.28318530718f*hz[code]*(float)dt/1000.0f);s->value[i]+=alpha*((float)raw[i]-s->value[i]);}raw[i]=(uint16_t)(s->value[i]+0.5f);}
    s->last_ms=ms;
}
#endif
