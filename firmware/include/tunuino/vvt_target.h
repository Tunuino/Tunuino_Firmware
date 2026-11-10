/* Copyright (c) 2026 Tunuino. Signed crank-degree targets, 16 RPM x 16 MAP. */
#ifndef TUNUINO_VVT_TARGET_H
#define TUNUINO_VVT_TARGET_H
#include <stdbool.h>
#include <stdint.h>
static inline uint16_t tunuino_vvt_target_u16(const uint8_t *p){return (uint16_t)((uint16_t)p[0]<<8|p[1]);}
static inline int16_t tunuino_vvt_target_s16(const uint8_t *p){uint16_t v=tunuino_vvt_target_u16(p);return (int16_t)(v<32768?(int32_t)v:(int32_t)v-65536);}
static inline bool tunuino_vvt_target_empty(const uint8_t *p){if(!p)return true;for(unsigned i=0;i<576;++i)if(p[i])return false;return true;}
static inline bool tunuino_vvt_target_validate(const uint8_t *p){
 if(!p)return false;
 if(tunuino_vvt_target_empty(p))return true;
 for(unsigned i=0;i<16;++i){uint16_t x=tunuino_vvt_target_u16(p+2*i),y=tunuino_vvt_target_u16(p+32+2*i);if(x>20000||y>50000||(i&&(x<=tunuino_vvt_target_u16(p+2*i-2)||y<=tunuino_vvt_target_u16(p+30+2*i))))return false;}
 for(unsigned i=0;i<256;++i){int v=tunuino_vvt_target_s16(p+64+2*i);if(v< -9000||v>9000)return false;}return true;
}
static inline bool tunuino_vvt_target_limits(const uint8_t *p,int16_t low,int16_t high){
 if(!tunuino_vvt_target_validate(p)||tunuino_vvt_target_empty(p))return false;
 for(unsigned i=0;i<256;++i){int v=tunuino_vvt_target_s16(p+64+2*i);if(v<low||v>high)return false;}return true;
}
/* The committed page is validated once when loaded; no table scan in the control loop. */
static inline bool tunuino_vvt_target_lookup(const uint8_t *p,uint32_t rpm,uint32_t map,int16_t *target){
 if(!p||!target)return false;
 uint32_t x=rpm,y=map;unsigned c=0,r=0;
 uint32_t x0=(uint32_t)tunuino_vvt_target_u16(p)*1000u,xN=(uint32_t)tunuino_vvt_target_u16(p+30)*1000u,y0=(uint32_t)tunuino_vvt_target_u16(p+32)*10u,yN=(uint32_t)tunuino_vvt_target_u16(p+62)*10u;
 if(x<x0)x=x0;
 if(x>xN)x=xN;
 if(y<y0)y=y0;
 if(y>yN)y=yN;
 while(c<14&&x>(uint32_t)tunuino_vvt_target_u16(p+2*(c+1))*1000u)++c;
 while(r<14&&y>(uint32_t)tunuino_vvt_target_u16(p+32+2*(r+1))*10u)++r;
 x0=(uint32_t)tunuino_vvt_target_u16(p+2*c)*1000u;y0=(uint32_t)tunuino_vvt_target_u16(p+32+2*r)*10u;
 int64_t dx=(int64_t)tunuino_vvt_target_u16(p+2*c+2)*1000-x0,dy=(int64_t)tunuino_vvt_target_u16(p+34+2*r)*10-y0;
 if(dx<=0||dy<=0)return false;
 int64_t a=(int64_t)tunuino_vvt_target_s16(p+64+2*(r*16+c))*(dx-(x-x0))+(int64_t)tunuino_vvt_target_s16(p+64+2*(r*16+c+1))*(x-x0);
 int64_t b=(int64_t)tunuino_vvt_target_s16(p+64+2*((r+1)*16+c))*(dx-(x-x0))+(int64_t)tunuino_vvt_target_s16(p+64+2*((r+1)*16+c+1))*(x-x0);
 int64_t n=a*(dy-(y-y0))+b*(y-y0),d=dx*dy;
 *target=(int16_t)((n+(n<0?-d/2:d/2))/d);return true;
}
#endif
