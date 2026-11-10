/* Copyright (c) 2026 Tunuino. CAM 1 VVT controller with independently resolved target. */
#include "tunuino/vvt.h"
#include "tunuino/vvt_target.h"
#include <string.h>
static uint16_t u(const uint8_t *p){return (uint16_t)((uint16_t)p[0]<<8|p[1]);}
static int16_t s16(const uint8_t *p){uint16_t v=u(p);return (int16_t)(v<32768?(int32_t)v:(int32_t)v-65536);}
bool tunuino_vvt_decode(const uint8_t p[256],tunuino_vvt_config *out){
    if(!p||!out||p[0]>2)return false;
    tunuino_vvt_config c={0};
    if(p[0]<2||!p[64]){for(unsigned i=64;i<256;++i)if(p[i])return false;*out=c;return true;}
    if(p[0]!=2||(p[64]<1||p[64]>3)||p[65]>1||p[66]>16||p[67]>1||p[110]>1||p[111])return false;
    if((p[64]==1&&p[123])||p[123]>(p[64]==3?2:1))return false;
    c.table_mode=p[123];
    c.advanced=p[64]==3;c.i_min=-10000;c.i_max=10000;c.output_min=-10000;c.output_max=10000;
    if(c.advanced){
        if(p[124]>1||p[125]>1||p[128]>1||p[129])return false;
        c.invert=p[124];c.override_dc=p[125];c.overridden_dc=u(p+126);c.override_target=p[128];c.overridden_target=s16(p+130);
        c.i_min=s16(p+132);c.i_max=s16(p+134);c.output_min=s16(p+136);c.output_max=s16(p+138);c.deadband=u(p+140);
        if(c.i_min< -10000||c.i_min>0||c.i_max<0||c.i_max>10000||c.output_min< -10000||c.output_min>0||c.output_max<0||c.output_max>10000||c.deadband>2000||c.overridden_dc>9500)return false;
    }
    for(unsigned i=c.advanced?142:124;i<256;++i)if(p[i])return false;
    memcpy(c.source,p+116,7);
    if(c.source[0]){if(c.source[0]<4||c.source[0]>120||!c.source[1]||c.source[1]>=c.source[0]/2||c.source[2]>c.source[3]||c.source[3]>=c.source[0]-c.source[1]||c.source[4]>1||c.source[5]>3||c.source[6]>3)return false;}
    else for(unsigned i=1;i<7;++i)if(c.source[i])return false;
    c.enabled=p[65];c.channel=p[66];c.direction=p[67];c.hz=u(p+68);c.min_duty=u(p+70);c.max_duty=u(p+72);c.steady=u(p+74);
    c.kp=u(p+76);c.ki=u(p+78);c.kd=u(p+80);c.reference=(uint32_t)u(p+82)<<16|u(p+84);
    c.minimum=s16(p+86);c.maximum=s16(p+88);c.target=s16(p+90);c.rpm_min=u(p+94);c.rpm_hysteresis=u(p+96);
    c.clt_min=s16(p+98);c.clt_hysteresis=u(p+100);c.delay_ms=u(p+102);c.max_age_ms=u(p+104);
    c.tracking_ms=u(p+106);c.tolerance=u(p+108);c.oil_required=p[110];c.oil_on=u(p+112);c.oil_off=u(p+114);
    if(p[92]||p[93]||c.hz<10||c.hz>300||c.min_duty>=c.max_duty||c.max_duty>9500||c.steady<c.min_duty||c.steady>c.max_duty
        ||c.kp>20000||c.ki>20000||c.kd>5000||(!c.kp&&!c.ki)||c.reference>=72000
        ||c.minimum< -9000||c.maximum>9000||c.minimum>=c.maximum||c.maximum-c.minimum>9000||c.target<c.minimum||c.target>c.maximum
        ||c.rpm_min<500||c.rpm_min>12000||c.rpm_hysteresis>c.rpm_min-100||c.clt_min<0||c.clt_min>12000||c.clt_hysteresis>2000
        ||c.delay_ms>60000||c.max_age_ms<20||c.max_age_ms>1000||c.tracking_ms<100||c.tracking_ms>5000||!c.tolerance||c.tolerance>2000
        ||c.oil_off>c.oil_on||c.oil_on>20000||(c.oil_required&&!c.oil_off)||(c.enabled&&!c.channel))return false;
    if(c.advanced&&((c.override_dc&&(c.overridden_dc<c.min_duty||c.overridden_dc>c.max_duty))
        ||(c.override_target&&(c.overridden_target<c.minimum||c.overridden_target>c.maximum))))return false;
    *out=c;return true;
}
bool tunuino_vvt_window(const tunuino_vvt_config *c,uint16_t slots,uint16_t missing,uint16_t first,uint16_t last,uint8_t phase){
    if(!c||slots<=missing||first>last||last>=slots-missing||phase>1)return false;
    int64_t low=(int64_t)phase*36000+(int64_t)first*36000/slots;
    int64_t high=(int64_t)phase*36000+(int64_t)(last+1u+(last==slots-missing-1u?missing:0u))*36000/slots-1;
    return (int64_t)c->reference-c->maximum>=low&&(int64_t)c->reference-c->minimum<=high;
}
void tunuino_vvt_resolve_target(tunuino_vvt_config *c,const uint8_t table[576],bool table_valid,bool map_valid,uint32_t rpm,uint32_t map){
    if(!c)return;
    if(c->override_target){c->target=c->overridden_target;c->target_invalid=0;return;}
    c->target_invalid=c->table_mode&&(!table_valid||!map_valid||!tunuino_vvt_target_lookup(table,rpm,map,&c->target));
}
uint16_t tunuino_vvt_pwm_duty(const tunuino_vvt_config *c,const tunuino_vvt_state *s){return s->active?(c->invert?(uint16_t)(10000-s->duty):s->duty):0;}
static void inhibit(tunuino_vvt_state *s,uint8_t reason,bool fault){
    s->active=0;s->duty=0;s->integral=0;s->edge=0;s->tracking=0;s->reason=reason;
    if(fault)s->latched=1;
}
void tunuino_vvt_update(const tunuino_vvt_config *c,tunuino_vvt_state *s,uint32_t now,
    bool armed,bool running,uint32_t rpm,bool cv,int32_t clt,bool ov,uint32_t oil,
    bool cam_valid,uint64_t snapshot,uint64_t edge,uint32_t raw){
    if(!c->enabled||!armed){memset(s,0,sizeof(*s));s->reason=c->enabled?TUNUINO_VVT_DISARMED:TUNUINO_VVT_DISABLED;return;}
    bool active=s->active!=0;uint32_t dt=s->sampled?now-s->last:0;s->last=now;s->sampled=1;s->feedback=0;
    if(!running){s->elapsed=0;s->sampled=0;inhibit(s,s->latched?TUNUINO_VVT_LATCHED:TUNUINO_VVT_STOPPED,false);return;}
    if(s->latched){inhibit(s,TUNUINO_VVT_LATCHED,false);return;}
    if(dt>100000u){s->elapsed=0;inhibit(s,TUNUINO_VVT_FEEDBACK,active);return;}
    uint32_t delay=(uint32_t)c->delay_ms*1000u;s->elapsed=s->elapsed>=delay||dt>=delay-s->elapsed?delay:s->elapsed+dt;
    if(!cv){inhibit(s,TUNUINO_VVT_TEMPERATURE,active);return;}
    if(c->oil_required&&(!ov||oil<(uint32_t)(active?c->oil_off:c->oil_on)*100u)){inhibit(s,TUNUINO_VVT_OIL,active);return;}
    if(rpm<(uint32_t)(active?c->rpm_min-c->rpm_hysteresis:c->rpm_min)*1000u){inhibit(s,TUNUINO_VVT_RPM,false);return;}
    if(clt<(int32_t)(active?c->clt_min-c->clt_hysteresis:c->clt_min)*10){inhibit(s,TUNUINO_VVT_TEMPERATURE,false);return;}
    if(s->elapsed<delay){inhibit(s,TUNUINO_VVT_DELAY,false);return;}
    if(c->target_invalid){inhibit(s,TUNUINO_VVT_TARGET,active);return;}
    if(!cam_valid||raw>=72000||snapshot<edge||snapshot-edge>(uint64_t)c->max_age_ms*1000u){inhibit(s,TUNUINO_VVT_FEEDBACK,active);return;}
    int32_t angle=(int32_t)c->reference-(int32_t)raw;
    if(angle>36000)angle-=72000;
    if(angle< -36000)angle+=72000;
    if(angle<c->minimum||angle>c->maximum){inhibit(s,TUNUINO_VVT_RANGE,true);return;}
    s->angle=(int16_t)angle;s->feedback=1;
    if(active&&edge==s->edge)return;
    if(active&&edge<s->edge){inhibit(s,TUNUINO_VVT_FEEDBACK,true);return;}
    uint64_t step=active?edge-s->edge:0;
    if(step>(uint64_t)c->max_age_ms*1000u){inhibit(s,TUNUINO_VVT_FEEDBACK,true);return;}
    int32_t error=(int32_t)c->target-angle,sign=c->direction?-1:1;
    const int32_t actual_error=error;const bool deadband=error<=c->deadband&&error>=-(int32_t)c->deadband;if(deadband)error=0;
    int64_t proportional=(int64_t)c->kp*error/1000;
    int64_t delta=(int64_t)c->ki*error*(int64_t)step/1000;
    int64_t derivative=step&&!deadband?-(int64_t)c->kd*(angle-s->previous)*1000000/(1000*(int64_t)step):0;
    int64_t candidate=s->integral+delta;
    if(candidate>(int64_t)c->i_max*1000000)candidate=(int64_t)c->i_max*1000000;
    if(candidate<(int64_t)c->i_min*1000000)candidate=(int64_t)c->i_min*1000000;
    int64_t correction=proportional+candidate/1000000+derivative;
    int64_t result=c->steady+sign*correction;
    if(!((result>c->max_duty&&sign*delta>0)||(result<c->min_duty&&sign*delta<0)||(correction>c->output_max&&delta>0)||(correction<c->output_min&&delta<0)))s->integral=candidate;
    correction=proportional+s->integral/1000000+derivative;
    correction=correction<c->output_min?c->output_min:correction>c->output_max?c->output_max:correction;
    result=c->steady+sign*correction;
    s->duty=(uint16_t)(result<c->min_duty?c->min_duty:result>c->max_duty?c->max_duty:result);
    if(c->override_dc){s->duty=c->overridden_dc;s->integral=0;}
    if(!c->override_dc&&(actual_error>c->tolerance||actual_error< -(int32_t)c->tolerance)){uint32_t limit=(uint32_t)c->tracking_ms*1000u;s->tracking=step>=limit-s->tracking?limit:s->tracking+(uint32_t)step;
        if(s->tracking>=limit){inhibit(s,TUNUINO_VVT_TRACKING,true);return;}}
    else s->tracking=0;
    s->previous=(int16_t)angle;s->edge=edge;s->active=1;s->reason=TUNUINO_VVT_ACTIVE;
}
