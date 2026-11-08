/* Copyright (c) 2026 Tunuino. Fuel/oil pressure acquisition, no actuation. */
#ifndef TUNUINO_PRESSURE_H
#define TUNUINO_PRESSURE_H
#include <stdbool.h>
#include <stdint.h>
#define TUNUINO_PRESSURE_OFFSET 38u
#define TUNUINO_PRESSURE_SIZE 26u
static inline uint16_t tunuino_pressure_u16(const uint8_t *p){return (uint16_t)((uint16_t)p[0]<<8|p[1]);}
/* All-zero v0 remains the original disabled configuration. V1 has two
 * nine-byte records: channel, ADC low/high, pressure low/high in deci-kPa.
 * Last seven bytes are reserved. AV 1..11 only; internal supply is excluded. */
static inline bool tunuino_pressure_validate(const uint8_t *p){
    if(!p)return false;
    bool empty=true;for(unsigned i=0;i<TUNUINO_PRESSURE_SIZE;++i)if(p[i])empty=false;
    if(empty)return true;
    if(p[0]!=1)return false;
    for(unsigned i=19;i<26;++i)if(p[i])return false;
    for(unsigned i=0;i<2;++i){const uint8_t *r=p+1+9*i;
        if(r[0]>11||tunuino_pressure_u16(r+1)>=tunuino_pressure_u16(r+3)||tunuino_pressure_u16(r+3)>4095
            ||tunuino_pressure_u16(r+5)>=tunuino_pressure_u16(r+7)||tunuino_pressure_u16(r+7)>20000)return false;
    }return true;
}
static inline bool tunuino_pressure_read(const uint8_t *p,unsigned sensor,const uint16_t raw[16],bool adc_valid,uint32_t *milli_kpa){
    if(!p||!raw||!milli_kpa||sensor>1||!adc_valid||p[0]!=1||!tunuino_pressure_validate(p))return false;
    const uint8_t *r=p+1+9*sensor;if(!r[0])return false;
    const uint16_t value=raw[r[0]-1],low=tunuino_pressure_u16(r+1),high=tunuino_pressure_u16(r+3);
    if(value>4095||value<low||value>high)return false;
    const uint32_t a=(uint32_t)tunuino_pressure_u16(r+5)*100u,b=(uint32_t)tunuino_pressure_u16(r+7)*100u;
    *milli_kpa=a+(uint32_t)((uint64_t)(b-a)*(value-low)/(high-low));return true;
}
#endif
