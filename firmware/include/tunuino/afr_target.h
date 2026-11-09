/* Copyright (c) 2026 Tunuino. Native RPM/MAP lambda target, unsigned milli-lambda. */
#ifndef TUNUINO_AFR_TARGET_H
#define TUNUINO_AFR_TARGET_H
#include <stdint.h>
#include <stdbool.h>
#define TUNUINO_AFR_SIZE 576u
static inline uint16_t tunuino_afr_u16(const uint8_t *p){return (uint16_t)((uint16_t)p[0]<<8|p[1]);}
static inline bool tunuino_afr_empty(const uint8_t *p){if(!p)return true;for(unsigned i=0;i<TUNUINO_AFR_SIZE;++i)if(p[i])return false;return true;}
static inline bool tunuino_afr_validate(const uint8_t *p){
 if(!p)return false;
 if(tunuino_afr_empty(p))return true;
 for(unsigned i=0;i<16;++i){uint16_t rpm=tunuino_afr_u16(p+2*i),map=tunuino_afr_u16(p+32+2*i);if(rpm>20000||map>50000||(i&&(rpm<=tunuino_afr_u16(p+2*i-2)||map<=tunuino_afr_u16(p+30+2*i))))return false;}
 for(unsigned i=0;i<256;++i){uint16_t v=tunuino_afr_u16(p+64+2*i);if(v<650||v>1300)return false;}return true;
}
static inline bool tunuino_afr_lookup(const uint8_t *p,uint32_t rpm_milli,uint32_t map_milli,uint16_t *target){
 if(!p||!target||tunuino_afr_empty(p)||!tunuino_afr_validate(p))return false;
 uint32_t x=rpm_milli,y=map_milli;unsigned c=0,r=0;
 uint32_t x0=(uint32_t)tunuino_afr_u16(p)*1000u,xN=(uint32_t)tunuino_afr_u16(p+30)*1000u,y0=(uint32_t)tunuino_afr_u16(p+32)*10u,yN=(uint32_t)tunuino_afr_u16(p+62)*10u;
 if(x<x0)x=x0;
 if(x>xN)x=xN;
 if(y<y0)y=y0;
 if(y>yN)y=yN;
 while(c<14&&x>(uint32_t)tunuino_afr_u16(p+2*(c+1))*1000u)++c;
 while(r<14&&y>(uint32_t)tunuino_afr_u16(p+32+2*(r+1))*10u)++r;
 x0=(uint32_t)tunuino_afr_u16(p+2*c)*1000u;y0=(uint32_t)tunuino_afr_u16(p+32+2*r)*10u;
 uint32_t dx=(uint32_t)tunuino_afr_u16(p+2*c+2)*1000u-x0,dy=(uint32_t)tunuino_afr_u16(p+34+2*r)*10u-y0;
 uint64_t a=(uint64_t)tunuino_afr_u16(p+64+2*(r*16+c))*(dx-(x-x0))+(uint64_t)tunuino_afr_u16(p+64+2*(r*16+c+1))*(x-x0);
 uint64_t b=(uint64_t)tunuino_afr_u16(p+64+2*((r+1)*16+c))*(dx-(x-x0))+(uint64_t)tunuino_afr_u16(p+64+2*((r+1)*16+c+1))*(x-x0);
 *target=(uint16_t)((a*(dy-(y-y0))+b*(y-y0)+(uint64_t)dx*dy/2)/((uint64_t)dx*dy));return true;
}
#endif
