/* Copyright (c) 2026 Tunuino. IO/calibration protocol and explicit board output hook. */
#include "tunuino/device.h"
#include "tunuino/engine_control.h"
#include "tunuino/aux_control.h"
#include <string.h>

uint16_t tunuino_device_u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] << 8 | p[1]); }
static void put16(uint8_t *p, uint16_t value) { p[0]=(uint8_t)(value>>8);p[1]=(uint8_t)value; }
static void put32(uint8_t *p, uint32_t value) { put16(p,(uint16_t)(value>>16));put16(p+2,(uint16_t)value); }
static int32_t signed16(const uint8_t *p) {
    uint16_t value=tunuino_device_u16(p);
    return value < 32768 ? (int32_t)value : (int32_t)value-65536;
}
static size_t page_offset(unsigned page) {
    return page==1 ? 0u : page==2 ? TUNUINO_IO_CONFIG_SIZE
        : page==3 ? TUNUINO_IO_CONFIG_SIZE+TUNUINO_IO_MAP_SIZE : TUNUINO_IO_AUX_OFFSET;
}
static size_t page_size(unsigned page) {
    return page==1||page==4 ? TUNUINO_IO_CONFIG_SIZE : TUNUINO_IO_MAP_SIZE;
}
/* Only active format: 16x16, axes U16, cells signed S16. */
static bool map_decode(tunuino_map *map, const uint8_t *bytes, tunuino_map_kind kind) {
    memset(map,0,sizeof(*map));
    map->kind=kind;map->rpm_count=map->load_count=TUNUINO_IO_MAP_DIM;
    for(unsigned i=0;i<TUNUINO_IO_MAP_DIM;++i) {
        map->rpm[i]=tunuino_device_u16(bytes+2*i);
        map->load_milli_kpa[i]=(int32_t)tunuino_device_u16(bytes+32+2*i)*10;
    }
    for(unsigned r=0;r<TUNUINO_IO_MAP_DIM;++r)
        for(unsigned c=0;c<TUNUINO_IO_MAP_DIM;++c)
            map->cells[r][c]=signed16(bytes+64+2*(r*TUNUINO_IO_MAP_DIM+c))*10;
    return tunuino_map_validate(map)==TUNUINO_MAP_OK;
}

uint32_t tunuino_device_validation_errors(const uint8_t *p, size_t length) {
    if (p==NULL || (length!=TUNUINO_IO_SETTINGS_SIZE&&length!=TUNUINO_IO_LEGACY_SIZE)) return TUNUINO_CONFIG_SCHEMA;
    uint32_t errors=0;
    if(tunuino_device_u16(p)!=TUNUINO_IO_SCHEMA || p[5]!=0 || p[9]!=0) errors|=TUNUINO_CONFIG_SCHEMA;
    for(unsigned i=27;i<32;++i) if(p[i]!=0) errors|=TUNUINO_CONFIG_SCHEMA;
    for(unsigned i=98;i<100;++i) if(p[i]!=0) errors|=TUNUINO_CONFIG_SCHEMA;
    tunuino_fuel_corrections fueling;
    if(!tunuino_fuel_corrections_decode(p+103,&fueling))errors|=TUNUINO_CONFIG_FUELING;
    bool has_profile=false;
    for(unsigned i=128;i<256;++i)if(p[i])has_profile=true;
    tunuino_engine_profile profile;
    if(has_profile&&!tunuino_engine_profile_decode(p+128,p[3],p[4],&profile))errors|=TUNUINO_CONFIG_ENGINE;
    if(length==TUNUINO_IO_SETTINGS_SIZE){
        uint32_t used=0;bool have=has_profile&&!(errors&TUNUINO_CONFIG_ENGINE);
        if(have){
            for(unsigned i=0;i<profile.cylinders;++i)used|=UINT32_C(1)<<profile.cylinder[i].injector;
            const uint8_t aux[3]={profile.pump,profile.fan,profile.relay};
            for(unsigned i=0;i<3;++i)if(aux[i])used|=UINT32_C(1)<<(aux[i]-1);
        }
        tunuino_aux_config c={0};
        if(!tunuino_aux_decode(p+TUNUINO_IO_AUX_OFFSET,used,have,&c))errors|=TUNUINO_CONFIG_AUX;
        else if((c.idle_mode&&!p[8])||(c.boost_mode&&(!p[6]||(uint32_t)tunuino_device_u16(p+16)<c.boost_cut))
            ||(c.tps_channel&&(c.tps_channel==p[6]||c.tps_channel==p[7])))errors|=TUNUINO_CONFIG_AUX;
        tunuino_feature_config f;
        if(!tunuino_features_decode(p+TUNUINO_IO_AUX_OFFSET,&f))errors|=TUNUINO_CONFIG_AUX;
        else if(f.flags){
            if(!have||!p[6]||!p[8]||((f.flags&(TUNUINO_FEATURE_ACCEL|TUNUINO_FEATURE_DFCO|TUNUINO_FEATURE_LAMBDA))&&!c.tps_channel)
                ||((f.flags&TUNUINO_FEATURE_LAMBDA)&&!p[7])
                ||((f.flags&TUNUINO_FEATURE_LIMIT)&&f.limit_cut>=tunuino_device_u16(p+96))
                ||((f.flags&TUNUINO_FEATURE_IAT)&&f.iat_channel==p[8]))errors|=TUNUINO_CONFIG_AUX;
            if(have&&profile.operation>=3&&(f.flags&TUNUINO_FEATURE_TRIMS)){
                for(unsigned i=0;i<profile.cylinders;++i)for(unsigned j=0;j<i;++j)
                    if(profile.cylinder[i].coil==profile.cylinder[j].coil&&f.spark_trim[i]!=f.spark_trim[j])errors|=TUNUINO_CONFIG_AUX;
            }
            uint32_t analog=0;const uint8_t existing[3]={p[6],p[7],c.tps_channel};
            for(unsigned i=0;i<3;++i)if(existing[i])analog|=UINT32_C(1)<<(existing[i]-1);
            const uint8_t channels[3]={f.voltage_channel,f.launch_channel,f.flex_channel};
            const uint16_t flags[3]={TUNUINO_FEATURE_VOLTAGE|TUNUINO_FEATURE_PROTECTION,TUNUINO_FEATURE_LAUNCH,TUNUINO_FEATURE_FLEX};
            for(unsigned i=0;i<3;++i)if(f.flags&flags[i]){uint32_t bit=UINT32_C(1)<<(channels[i]-1);if(analog&bit)errors|=TUNUINO_CONFIG_AUX;analog|=bit;}
        }
    }
    if(p[102]>1 || (p[102] && (!has_profile||!p[2]||!p[6]||!p[8])))errors|=TUNUINO_CONFIG_AUTONOMOUS;
    if(p[2]>1 || p[3]<4 || p[3]>120 || p[4]<1 || p[4]>=p[3]/2
        || tunuino_device_u16(p+96)<1000 || tunuino_device_u16(p+96)>20000) errors|=TUNUINO_CONFIG_TRIGGER;
    if(p[6]>11 || tunuino_device_u16(p+10)>=tunuino_device_u16(p+12)
        || tunuino_device_u16(p+12)>4095 || (p[6] && tunuino_device_u16(p+14)>=tunuino_device_u16(p+16))
        || tunuino_device_u16(p+14)>50000 || tunuino_device_u16(p+16)>50000) errors|=TUNUINO_CONFIG_MAP;
    if(p[7]>11 || tunuino_device_u16(p+18)>=tunuino_device_u16(p+20)
        || tunuino_device_u16(p+20)>4095 || (p[7] && (tunuino_device_u16(p+22)==0
        || tunuino_device_u16(p+22)>=tunuino_device_u16(p+24)))
        || tunuino_device_u16(p+22)>5000 || tunuino_device_u16(p+24)>5000) errors|=TUNUINO_CONFIG_LAMBDA;
    if(p[8]>4 || p[26]!=3) errors|=TUNUINO_CONFIG_COOLANT;
    for(unsigned i=0;i<16;++i) {
        unsigned raw=tunuino_device_u16(p+32+2*i);int32_t value=signed16(p+64+2*i);
        if(i<3) {
            if(raw>4095 || value< -4000 || value>15000 || (i && raw<=tunuino_device_u16(p+30+2*i)))
                errors|=TUNUINO_CONFIG_COOLANT;
        } else if(raw!=0 || value!=0) errors|=TUNUINO_CONFIG_COOLANT;
    }
    if(p[8] && signed16(p+64)==signed16(p+66) && signed16(p+66)==signed16(p+68)) errors|=TUNUINO_CONFIG_COOLANT;
    if(tunuino_device_u16(p+100)<2800 || tunuino_device_u16(p+100)>3600) errors|=TUNUINO_CONFIG_ADC;
    for(unsigned page=2;page<=3;++page) {
        const uint8_t *m=p+page_offset(page);
        uint32_t issue=page==2?TUNUINO_CONFIG_VE:TUNUINO_CONFIG_IGNITION;
        for(unsigned i=0;i<TUNUINO_IO_MAP_DIM;++i) {
            if(tunuino_device_u16(m+2*i)>20000 || tunuino_device_u16(m+32+2*i)>50000
                || (i && (tunuino_device_u16(m+2*i)<=tunuino_device_u16(m+2*i-2)
                    || tunuino_device_u16(m+32+2*i)<=tunuino_device_u16(m+30+2*i)))) errors|=issue;
        }
        for(unsigned i=0;i<TUNUINO_IO_MAP_DIM*TUNUINO_IO_MAP_DIM;++i) {
            int32_t value=signed16(m+64+2*i);
            if((page==2 && (value<0 || value>30000)) || (page==3 && (value< -9000 || value>9000))) errors|=issue;
            /* Autonomous operation requires an explicitly populated fuel map. */
            if(p[102]&&page==2&&value==0)errors|=TUNUINO_CONFIG_AUTONOMOUS;
        }
    }
    return errors;
}

bool tunuino_device_validate(const uint8_t *p, size_t length) {
    return tunuino_device_validation_errors(p,length)==0;
}

static bool publish(tunuino_device *d) {
    memset(&d->cam,0,sizeof(d->cam));d->position_flags=0;d->position_time_us=0;
    if (++d->generation==0) d->generation=1;
    if (!map_decode(&d->scratch,d->active+TUNUINO_IO_CONFIG_SIZE,TUNUINO_MAP_VE)
        || tunuino_map_snapshot_prepare(&d->ve,&d->scratch,d->generation)!=TUNUINO_MAP_OK
        || !map_decode(&d->scratch,d->active+TUNUINO_IO_CONFIG_SIZE+TUNUINO_IO_MAP_SIZE,TUNUINO_MAP_IGNITION)
        || tunuino_map_snapshot_prepare(&d->ignition,&d->scratch,d->generation)!=TUNUINO_MAP_OK) return false;
    return true;
}

void tunuino_device_init(tunuino_device *d, tunuino_device_save save, tunuino_device_clock clock, void *user) {
    memset(d,0,sizeof(*d));d->save=save;d->clock=clock;d->user=user;
    uint8_t *p=d->active;
    put16(p,TUNUINO_IO_SCHEMA);p[3]=36;p[4]=1; /* Acquisition remains disabled until configured. */
    put16(p+10,0);put16(p+12,4095);
    put16(p+18,0);put16(p+20,4095);
    /* An explicit user calibration is required before selecting an input. */
    p[26]=3;put16(p+32,0);put16(p+34,2048);put16(p+36,4095);
    put16(p+96,12000);put16(p+100,3300);
    const uint16_t rpm[16]={500,750,1000,1250,1500,1750,2000,2500,
        3000,3500,4000,4500,5000,6000,7000,8000};
    const uint16_t load[16]={2000,3000,4000,5000,6000,7000,8000,9000,
        10000,11000,12000,13000,14000,16000,18000,20000};
    for(unsigned page=2;page<=3;++page) for(unsigned i=0;i<16;++i) {
        put16(p+page_offset(page)+2*i,rpm[i]);
        put16(p+page_offset(page)+32+2*i,load[i]);
    }
    memcpy(d->staged,d->active,sizeof(d->active));(void)publish(d);
}

bool tunuino_device_restore(tunuino_device *d, const uint8_t *bytes, size_t length) {
    if(d==NULL || !tunuino_device_validate(bytes,length))return false;
    memmove(d->active,bytes,length);memcpy(d->staged,d->active,length);d->dirty=false;d->persisted=true;
    if(length==TUNUINO_IO_LEGACY_SIZE){memset(d->active+length,0,TUNUINO_IO_SETTINGS_SIZE-length);memcpy(d->staged,d->active,sizeof(d->active));}
    return publish(d);
}

void tunuino_device_sample(tunuino_device *d, uint64_t time_ms, const uint16_t raw[16],
    bool adc_valid, uint32_t rpm_milli, bool rpm_valid, uint16_t tooth, uint32_t errors) {
    tunuino_telemetry_sensors sensors={0};tunuino_analog_point points[16];
    const uint8_t *p=d->active;
    memset(&d->cam,0,sizeof(d->cam));d->position_flags=0;d->position_time_us=0;
    d->adc_valid=adc_valid&&raw!=NULL;
    d->trigger_valid=rpm_valid&&p[2]&&rpm_milli<=(uint32_t)tunuino_device_u16(p+96)*1000u;
    d->tooth=tooth;d->trigger_errors=errors;
    memset(&d->live,0,sizeof(d->live));d->live.time_ms=time_ms;
    if (adc_valid && raw!=NULL) {
        for(unsigned i=0;i<16;++i) if(raw[i]>4095) d->adc_valid=false;
        if(d->adc_valid)memcpy(d->raw,raw,sizeof(d->raw));
    }
    if(!d->adc_valid)memset(d->raw,0,sizeof(d->raw));
    if(rpm_valid && p[2] && rpm_milli <= (uint32_t)tunuino_device_u16(p+96)*1000u) {
        d->live.rpm_milli=rpm_milli;d->live.valid_mask|=TUNUINO_LIVE_RPM_VALID;
    }
    if(d->adc_valid && p[6]) {
        points[0]=(tunuino_analog_point){tunuino_device_u16(p+10),(int32_t)tunuino_device_u16(p+14)*10};
        points[1]=(tunuino_analog_point){tunuino_device_u16(p+12),(int32_t)tunuino_device_u16(p+16)*10};
        const tunuino_analog_curve curve={points,2,0,500000};
        (void)tunuino_analog_update(&sensors,TUNUINO_ANALOG_MAP,&curve,raw[p[6]-1],true,0);
    }
    if(d->adc_valid && p[7]) {
        points[0]=(tunuino_analog_point){tunuino_device_u16(p+18),(int32_t)tunuino_device_u16(p+22)*1000};
        points[1]=(tunuino_analog_point){tunuino_device_u16(p+20),(int32_t)tunuino_device_u16(p+24)*1000};
        const tunuino_analog_curve curve={points,2,1,5000000};
        (void)tunuino_analog_update(&sensors,TUNUINO_ANALOG_LAMBDA,&curve,raw[p[7]-1],true,0);
    }
    if(d->adc_valid && p[8]) {
        for(unsigned i=0;i<p[26];++i)points[i]=(tunuino_analog_point){tunuino_device_u16(p+32+2*i),signed16(p+64+2*i)*10};
        const tunuino_analog_curve curve={points,p[26],-40000,150000};
        (void)tunuino_analog_update(&sensors,TUNUINO_ANALOG_COOLANT,&curve,raw[11+p[8]],true,0);
    }
    d->live.valid_mask|=sensors.valid_mask;d->live.map_milli_kpa=sensors.map_milli_kpa;
    d->live.lambda_million=sensors.lambda_million;d->live.coolant_milli_c=sensors.coolant_milli_c;
}

void tunuino_device_position(tunuino_device *d,uint64_t now,
    const tunuino_position_reading *position,const tunuino_cam_reading *cam) {
    memset(&d->cam,0,sizeof(d->cam));d->position_flags=0;d->position_time_us=now;
    if (!d->trigger_valid || !position || !position->crank_valid) return;
    d->position_flags=1;
    if (!position->cycle_valid) return;
    d->position_flags|=2;
    if (cam && cam->valid && cam->angle_centi<72000 && cam->time_us<=now
        && now-cam->time_us<UINT64_C(120000000)/30u) {
        d->cam=*cam;d->position_flags|=4;
    }
}

static void runtime(tunuino_device *d, const tunuino_identity *id, uint8_t out[100]) {
    memset(out,0,100);uint8_t packet[38],query[14];size_t size;const uint8_t *body;
    (void)tunuino_frame_encode((const uint8_t *)"TUNUINO?",8,query,sizeof(query),&size);
    (void)tunuino_identity_reply(id,query,size,packet,sizeof(packet),&size);
    (void)tunuino_frame_decode(packet,size,&body,&size);memcpy(out,body+1,31);
    if(d->output_command){
        const uint8_t q=0;uint8_t state[24];size_t length=0;
        if(d->output_command(d->output_user,&q,1,state,sizeof(state),&length)&&length==24)
            out[31]=(uint8_t)(state[2]|((state[15]&1u)?16u:0u)|((state[15]&2u)?32u:0u)|(state[3]?128u:0u));
    }
    put32(out+32,(uint32_t)(d->live.time_ms/1000));put16(out+36,(uint16_t)(d->live.time_ms%1000));
    out[38]=d->live.valid_mask;out[39]=(uint8_t)((d->adc_valid?1:0)|(d->dirty?2:0)|(d->persisted?4:0)|(d->trigger_valid?8:0));
    put32(out+40,d->live.rpm_milli);put32(out+44,d->live.map_milli_kpa);put32(out+48,d->live.lambda_million);
    put32(out+52,(uint32_t)d->live.coolant_milli_c);put32(out+56,d->generation);
    for(unsigned i=0;i<16;++i)put16(out+60+2*i,d->raw[i]);
    put16(out+92,tunuino_device_u16(d->active+100));put16(out+94,d->tooth);put32(out+96,d->trigger_errors);
}

/* Read-only 16x16 fuel/spark TABLE PREVIEW; sequential scratch lookup. No actuation. */
static void engine_preview(tunuino_device *d, uint8_t out[TUNUINO_ENGINE_PREVIEW_SIZE]) {
    memset(out,0,TUNUINO_ENGINE_PREVIEW_SIZE);
    out[1]=1; /* Wire format v1. 0 means no trustworthy calculation. */
    const tunuino_live_sample *sample=&d->live;
    uint8_t reasons=0;
    if(!(sample->valid_mask&TUNUINO_LIVE_RPM_VALID)||sample->rpm_milli==0)
        reasons|=TUNUINO_ENGINE_NO_RPM;
    if(!(sample->valid_mask&TUNUINO_LIVE_MAP_VALID)||sample->map_milli_kpa==0)
        reasons|=TUNUINO_ENGINE_NO_MAP;
    if(!d->active[2])reasons|=TUNUINO_ENGINE_TRIGGER_DISABLED;
    if(!d->trigger_valid)reasons|=TUNUINO_ENGINE_NO_SYNC;
    if(!d->adc_valid)reasons|=TUNUINO_ENGINE_BAD_ADC;
    if(sample->rpm_milli>=(uint32_t)tunuino_device_u16(d->active+96)*1000u)
        reasons|=TUNUINO_ENGINE_RPM_LIMIT;
    if(sample->valid_mask&TUNUINO_LIVE_RPM_VALID)put32(out+8,sample->rpm_milli);
    if(sample->valid_mask&TUNUINO_LIVE_MAP_VALID)put32(out+12,sample->map_milli_kpa);
    put32(out+16,d->generation);
    if(!reasons) {
        double ve=0.0, advance=0.0;
        tunuino_map_status v=TUNUINO_MAP_FORMAT, a=TUNUINO_MAP_FORMAT;
        const double rpm=sample->rpm_milli/1000.0;
        const double kpa=sample->map_milli_kpa/1000.0;
        if(map_decode(&d->scratch,d->active+TUNUINO_IO_CONFIG_SIZE,TUNUINO_MAP_VE))
            v=tunuino_map_lookup(&d->scratch,rpm,kpa,&ve);
        if(map_decode(&d->scratch,d->active+TUNUINO_IO_CONFIG_SIZE+TUNUINO_IO_MAP_SIZE,TUNUINO_MAP_IGNITION))
            a=tunuino_map_lookup(&d->scratch,rpm,kpa,&advance);
        if(v==TUNUINO_MAP_CLAMPED||a==TUNUINO_MAP_CLAMPED)
            reasons|=TUNUINO_ENGINE_OUT_OF_TABLE;
        else if(v!=TUNUINO_MAP_OK||a!=TUNUINO_MAP_OK||ve<=0.0
                ||ve>300.0||advance< -90.0||advance>90.0)
            reasons|=TUNUINO_ENGINE_UNCALIBRATED;
        else {
            const uint16_t ve_centi=(uint16_t)(ve*100.0+0.5);
            const int32_t spark_centi=(int32_t)(advance*100.0+(advance>=0?0.5:-0.5));
            put16(out+4,ve_centi);
            put16(out+6,(uint16_t)(int16_t)spark_centi);
            out[2]=1; /* Computation valid; actuators remain disabled. */
        }
    }
    out[3]=reasons;
}

static void engine_timing_preview(tunuino_device *d,
    uint8_t out[TUNUINO_ENGINE_TIMING_SIZE]) {
    memset(out,0,TUNUINO_ENGINE_TIMING_SIZE);
    out[1]=1;
    put32(out+4,d->live.rpm_milli);
    put32(out+28,d->generation);
    uint8_t preview[TUNUINO_ENGINE_PREVIEW_SIZE];
    engine_preview(d,preview);
    out[3]=preview[3];
    if(preview[2]!=1)return;
    tunuino_engine_math_result values;
    if(tunuino_engine_math_calculate(d->live.rpm_milli,d->active[3],d->active[4],
        tunuino_device_u16(preview+4),(int16_t)tunuino_device_u16(preview+6),
        d->live.map_milli_kpa,&values)!=TUNUINO_ENGINE_MATH_OK) {
        out[3]=TUNUINO_ENGINE_UNCALIBRATED;
        return;
    }
    out[2]=1;
    put32(out+8,values.revolution_us);
    put32(out+12,values.cycle_us);
    put32(out+16,values.wheel_slot_us);
    put32(out+20,values.load_centi_percent);
    put32(out+24,(uint32_t)values.spark_relative_us);
}

bool tunuino_device_engine_values(tunuino_device *d,uint16_t *ve,int16_t *advance){
    uint8_t preview[TUNUINO_ENGINE_PREVIEW_SIZE];engine_preview(d,preview);
    if(preview[2]){*ve=tunuino_device_u16(preview+4);*advance=(int16_t)tunuino_device_u16(preview+6);return true;}
    /* At starter speed use the explicitly calibrated first RPM column.
     * Never extrapolate above the RPM axis or outside either MAP axis. */
    if(preview[3]!=TUNUINO_ENGINE_OUT_OF_TABLE)return false;
    double values[2];
    for(unsigned i=0;i<2;++i){
        const uint8_t *p=d->active+TUNUINO_IO_CONFIG_SIZE+i*TUNUINO_IO_MAP_SIZE;
        if(d->live.rpm_milli>(uint32_t)tunuino_device_u16(p+30)*1000u
            ||d->live.map_milli_kpa<(uint32_t)tunuino_device_u16(p+32)*10u
            ||d->live.map_milli_kpa>(uint32_t)tunuino_device_u16(p+62)*10u
            ||!map_decode(&d->scratch,p,i==0?TUNUINO_MAP_VE:TUNUINO_MAP_IGNITION))return false;
        tunuino_map_status s=tunuino_map_lookup(&d->scratch,d->live.rpm_milli/1000.0,d->live.map_milli_kpa/1000.0,&values[i]);
        if(s!=TUNUINO_MAP_OK&&s!=TUNUINO_MAP_CLAMPED)return false;
    }
    if(values[0]<=0||values[0]>300||values[1]< -90||values[1]>90)return false;
    *ve=(uint16_t)(values[0]*100+0.5);
    *advance=(int16_t)(values[1]*100+(values[1]>=0?0.5:-0.5));return true;
}

tunuino_dispatch_status tunuino_device_reply(tunuino_device *d,const tunuino_identity *id,
    const uint8_t *q,size_t length,uint8_t *response,size_t capacity,size_t *written) {
    uint8_t body[257]={0};size_t size=1;
    if(length>=10 && memcmp(q,"TUNUINO:X",9)==0 && d->output_command){
        if(capacity<(q[9]==0?30u:q[9]==7?38u:q[9]==8?46u:7u))return TUNUINO_DISPATCH_CAPACITY;
        if(!d->output_command(d->output_user,q+9,length-9,body,sizeof(body),&size)){body[0]=0x84;size=1;}
    }
    else if(length==1 && q[0]=='D') {memcpy(d->staged,d->active,sizeof(d->staged));d->dirty=false;}
    else if(length==1 && q[0]=='A') {runtime(d,id,body+1);size=101;}
    else if(length==5 && q[0]=='O') {
        /* TunerStudio ranged output: big-endian U16 offset/count. Keep A for
         * existing clients, but allow a future INI to request exact slices. */
        unsigned offset=tunuino_device_u16(q+1),count=tunuino_device_u16(q+3);
        if(!count||count>100||offset>=100||count>100-offset)body[0]=0x83;
        else {
            uint8_t channels[100];
            runtime(d,id,channels);
            memcpy(body+1,channels+offset,count);
            size=1+count;
        }
    }
    else if(length==9 && memcmp(q,"TUNUINO:V",9)==0) {
        /* Read-only staged validation v1; no flash, activation or outputs. */
        body[1]=1;body[2]=d->dirty?1:0;body[3]=d->persisted?1:0;
        put32(body+4,tunuino_device_validation_errors(d->staged,sizeof(d->staged)));
        put32(body+8,d->generation);put32(body+12,tunuino_crc32(d->staged,256));size=16;
    }
    else if(length==9 && memcmp(q,"TUNUINO:E",9)==0) {
        engine_preview(d,body);
        size=TUNUINO_ENGINE_PREVIEW_SIZE;
    }
    else if(length==9 && memcmp(q,"TUNUINO:T",9)==0) {
        engine_timing_preview(d,body);
        size=TUNUINO_ENGINE_TIMING_SIZE;
    }
    else if(length==9 && memcmp(q,"TUNUINO:H",9)==0) {
        /* Cam acquisition v1: flags, reserved, phase centidegrees, captured
         * edge us, snapshot us, generation, wheel slots/missing, reserved. */
        body[1]=1;body[2]=d->position_flags;
        put32(body+4,d->cam.angle_centi);
        put32(body+8,(uint32_t)(d->cam.time_us>>32));put32(body+12,(uint32_t)d->cam.time_us);
        put32(body+16,(uint32_t)(d->position_time_us>>32));put32(body+20,(uint32_t)d->position_time_us);
        put32(body+24,d->generation);put16(body+28,d->active[3]);body[30]=d->active[4];size=32;
    }
    else if(length==1 && (q[0]=='S'||q[0]=='Q'||q[0]=='V'||q[0]=='F')) {
        const char *text=q[0]=='V'?TUNUINO_IO_VERSION:q[0]=='F'?TUNUINO_IO_TS_SERIAL_VERSION:TUNUINO_IO_SIGNATURE;
        /* TunerStudio 3.3.01 on the physical F427 reports Unsupported Controller
         * Firmware when CRC-framed S includes a trailing 0x00: it compares
         * the full received string against the exact INI signature. Do not
         * include NUL in framed S/Q; keep the V string terminator and the
         * separate plain Q/S discovery behavior unchanged. */
        size_t count=strlen(text);
        if(q[0]=='V') ++count;
        memcpy(body+1,text,count);size=1+count;
    } else if(length==1 && (q[0]=='C'||q[0]=='c')) {
        /* TunerStudio communication checks, with the envelope status first. */
        if (q[0]=='C') { body[1]=0xFF;size=2; }
        else { put16(body+1,(uint16_t)(d->live.time_ms/1000u));size=3; }
    } else if(length==1 && q[0]=='I') {
        body[1]=0;size=2; /* The local ECU has CAN ID zero. */
    } else if(length==2 && q[0]=='f' && q[1]==0) {
        body[1]=2;put16(body+2,100);put16(body+4,64);size=6;
    } else if(length>=3 && (q[0]=='r'||q[0]=='w'||q[0]=='b'||q[0]=='k')) {
        unsigned page=tunuino_device_u16(q+1);
        if(page<1||page>4)body[0]=0x83;
        else if((q[0]=='w'||q[0]=='b')&&d->configuration_writable&&!d->configuration_writable())body[0]=0x84;
        else if(q[0]=='b' && length==3) {
            if(!tunuino_device_validate(d->staged,sizeof(d->staged)))body[0]=0x84;
            else if(d->save==NULL || !d->save(d->user,d->staged,sizeof(d->staged)))body[0]=0x80;
            else if(!tunuino_device_restore(d,d->staged,sizeof(d->staged)))body[0]=0x80;
        } else if(q[0]=='k' && length==3) {
            put32(body+1,tunuino_crc32(d->staged+page_offset(page),page_size(page)));size=5;
        } else if(length>=7 && (q[0]=='r'||q[0]=='w')) {
            unsigned offset=tunuino_device_u16(q+3),count=tunuino_device_u16(q+5);
            if(count==0||count>256||offset>page_size(page)||count>page_size(page)-offset
                ||length!=(q[0]=='r'?7u:7u+count))body[0]=0x83;
            else if(q[0]=='r'){memcpy(body+1,d->staged+page_offset(page)+offset,count);size=1+count;}
            else {memcpy(d->staged+page_offset(page)+offset,q+7,count);d->dirty=memcmp(d->active,d->staged,sizeof(d->active))!=0;}
        } else body[0]=0x83;
    } else body[0]=0x83;
    return tunuino_frame_encode(body,size,response,capacity,written)==TUNUINO_FRAME_OK?TUNUINO_DISPATCH_OK:TUNUINO_DISPATCH_CAPACITY;
}
