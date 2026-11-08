/* Copyright (c) 2026 Tunuino. Conditioned trigger-edge blanking. */
#ifndef TUNUINO_PRIMARY_INPUT_H
#define TUNUINO_PRIMARY_INPUT_H
#include <stdbool.h>
#include <stdint.h>
/* Primary: header byte 5 version, byte 9 level; CAM: bytes 98/99. Legacy 0/0 is no filter.
 * V1 levels 0/1/2/3 = no blanking / 5 / 10 / 20 us.
 * This does not switch sensor circuitry or measure pulse width. */
typedef struct {uint32_t last;uint8_t level;bool ready;} tunuino_primary_input;
static inline bool tunuino_primary_input_validate(uint8_t version,uint8_t level){return (version==0&&level==0)||(version==1&&level<=3);}
static inline void tunuino_primary_input_reset(tunuino_primary_input *s,uint8_t level){s->last=0;s->level=level;s->ready=false;}
static inline bool tunuino_primary_input_accept(tunuino_primary_input *s,uint32_t counter,bool primary){
    static const uint32_t interval[]={0,5,10,20};
    if(!primary)return true;
    if(s->level>3)return false;
    if(s->ready&&(uint32_t)(counter-s->last)<interval[s->level])return false;
    s->ready=true;s->last=counter;return true;
}
#endif
