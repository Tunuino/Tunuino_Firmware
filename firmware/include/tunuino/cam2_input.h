/* Copyright (c) 2026 Tunuino. Independent digital CAM 2 acquisition. */
#ifndef TUNUINO_CAM2_INPUT_H
#define TUNUINO_CAM2_INPUT_H
#include "tunuino/vss.h"
#define TUNUINO_CAM2_OFFSET 57u
#define TUNUINO_CAM2_SIZE 7u
/* V0 all-zero disabled; V1 version/input/edge/blanking/timeout U16/reserved.
 * Input 1 is DIGITAL2 J101/1, fixed Hall/optical Schmitt input, not VR. */
static inline bool tunuino_cam2_validate(const uint8_t *p){
 if(!p)return false;
 bool empty=true;for(unsigned i=0;i<7;++i)if(p[i])empty=false;
 return empty||(p[0]==1&&p[1]<=1&&p[2]<=1&&p[3]<=3&&tunuino_vss_u16(p+4)>=100&&tunuino_vss_u16(p+4)<=5000&&!p[6]);
}
static inline tunuino_vss_reading tunuino_cam2_sample(tunuino_vss_tracker *s,const tunuino_vss_capture *c,const uint8_t *p,uint64_t now){
 uint8_t config[14]={0};
 if(tunuino_cam2_validate(p)&&p[0]==1){memcpy(config,p,4);memcpy(config+8,p+4,2);}
 tunuino_vss_reading r=tunuino_vss_sample(s,c,config,now);
 if(r.valid==1&&r.reason==7)r.reason=0;
 return r;
}
#endif
