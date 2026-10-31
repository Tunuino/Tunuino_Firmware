/* Copyright (c) 2026 Tunuino. Shared persisted/volatile profile validation. */
#include "tunuino/engine_control.h"
static uint16_t u16(const uint8_t *p){return (uint16_t)((uint16_t)p[0]<<8|p[1]);}
static uint32_t u32(const uint8_t *p){return (uint32_t)u16(p)<<16|u16(p+2);}
static int32_t signed16(const uint8_t *p){uint16_t v=u16(p);return v<32768?v:(int32_t)v-65536;}
bool tunuino_engine_profile_decode(const uint8_t p[128],uint16_t slots,uint8_t missing,tunuino_engine_profile *out){
    if(!p||!out||(p[0]<1||p[0]>3)||p[1]<1||p[1]>12||slots<4||slots>120||missing==0||missing>=slots/2)return false;
    tunuino_engine_profile c={0};c.cylinders=p[1];
    c.required_fuel_us=u16(p+2);c.dwell_us=u16(p+4);c.dead_us=u16(p+6);
    c.maximum_injection_us=u16(p+8);c.prime_ms=u16(p+10);
    c.fan_on_centi=(int16_t)signed16(p+12);c.fan_off_centi=(int16_t)signed16(p+14);
    c.injection_end_centi=(int16_t)signed16(p+16);c.tooth_zero_centi=u32(p+18);
    c.pump=p[22];c.fan=p[23];c.relay=p[24];c.cam_first=p[25];c.cam_last=p[26];c.cam_phase=p[27];
    if(c.required_fuel_us<100||c.required_fuel_us>30000||c.dwell_us<100||c.dwell_us>5000
        ||c.dead_us>5000||c.maximum_injection_us<100||c.maximum_injection_us>20000
        ||c.dead_us>=c.maximum_injection_us||c.prime_ms>3000||c.tooth_zero_centi>=72000
        ||c.injection_end_centi< -30000||c.injection_end_centi>30000
        ||c.fan_off_centi<0||c.fan_off_centi>=c.fan_on_centi||c.fan_on_centi>12000
        ||c.cam_first>c.cam_last||c.cam_last>=slots-missing||c.cam_phase>1)return false;
    c.operation=p[28];
    if((p[0]==1&&c.operation)||(p[0]==2&&(c.operation<1||c.operation>2))
        ||(p[0]==3&&(c.operation<3||c.operation>4||c.cylinders%2)))return false;
    bool paired=p[0]==3;
    for(unsigned i=29;i<32;++i)if(p[i])return false;
    uint32_t used=0;
    for(unsigned i=0;i<12;++i){const uint8_t *row=p+32+8*i;
        if(i>=c.cylinders){for(unsigned j=0;j<8;++j)if(row[j])return false;continue;}
        if(row[0]>=16||row[1]>=12||u32(row+2)>=72000||row[6]||row[7])return false;
        uint32_t mask=(UINT32_C(1)<<row[0])|(UINT32_C(1)<<(16+row[1]));
        if((used&(UINT32_C(1)<<row[0]))||(!paired&&(used&mask)))return false;
        used|=mask;c.cylinder[i]=(tunuino_cylinder){row[0],row[1],u32(row+2)};
        for(unsigned j=0;j<i;++j)if(c.cylinder[j].tdc_centi==c.cylinder[i].tdc_centi)return false;
    }
    if(paired)for(unsigned i=0;i<c.cylinders;++i){
        unsigned partners=0;
        for(unsigned j=0;j<c.cylinders;++j)if(i!=j&&c.cylinder[i].coil==c.cylinder[j].coil){
            uint32_t a=c.cylinder[i].tdc_centi,b=c.cylinder[j].tdc_centi;
            if((a>b?a-b:b-a)!=36000u)return false;
            ++partners;
        }
        if(partners!=1)return false;
    }
    const uint8_t aux[3]={c.pump,c.fan,c.relay};
    for(unsigned i=0;i<3;++i)if(aux[i]){
        if(aux[i]>16||(used&(UINT32_C(1)<<(aux[i]-1))))return false;
        used|=UINT32_C(1)<<(aux[i]-1);
    }
    *out=c;return true;
}
