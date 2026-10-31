/* Copyright (c) 2026 Tunuino. Persistent throttle, idle and boost control. */
#ifndef TUNUINO_AUX_CONTROL_H
#define TUNUINO_AUX_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#define TUNUINO_AUX_CONFIG_SIZE 256u
typedef struct {
    uint8_t idle_mode,idle_channel,boost_mode,boost_channel,tps_channel;
    uint16_t tps_closed,tps_open,idle_hz,boost_hz,idle_min,idle_max,cranking_duty,idle_tps,idle_max_rpm;
    uint16_t idle_p,idle_i,target_rpm[3],idle_base[3],cranking_rpm;
    int16_t temperature[3];
    uint16_t boost_min,boost_max,boost_base,boost_target,boost_cut,boost_min_rpm,boost_tps,boost_p,boost_i;
    uint16_t tps_tolerance;
} tunuino_aux_config;
typedef struct {int64_t integral;uint32_t updated;uint8_t active;} tunuino_pi_state;
typedef struct {
    tunuino_aux_config config;
    tunuino_pi_state idle,boost;
    uint16_t idle_duty,boost_duty,idle_target,tps;
    uint8_t tps_valid,idle_active,boost_active;
} tunuino_aux_control;
bool tunuino_aux_decode(const uint8_t bytes[256],uint32_t used_ls,bool have_profile,tunuino_aux_config *out);
bool tunuino_aux_tps(const tunuino_aux_config *config,bool adc_valid,const uint16_t raw[16],uint16_t *percent);
void tunuino_aux_reset(tunuino_aux_control *control);
/* False means measured overboost: caller must cut all engine outputs. */
bool tunuino_aux_update(tunuino_aux_control *control,uint32_t now,bool armed,bool running,
    uint32_t rpm_milli,bool tps_valid,uint16_t tps,bool coolant_valid,int32_t coolant_milli,
    bool map_valid,uint32_t map_milli);
#endif
