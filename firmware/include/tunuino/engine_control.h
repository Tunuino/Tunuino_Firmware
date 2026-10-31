/* Copyright (c) 2026 Tunuino. Configurable four-stroke sequential control. */
#ifndef TUNUINO_ENGINE_CONTROL_H
#define TUNUINO_ENGINE_CONTROL_H
#include "tunuino/outputs.h"
#include "tunuino/position.h"
#include "tunuino/fuel_corrections.h"
#include "tunuino/engine_features.h"
#define TUNUINO_ENGINE_PROFILE_SIZE 128u
typedef struct { uint8_t injector,coil; uint32_t tdc_centi; } tunuino_cylinder;
typedef struct {
    uint8_t cylinders,pump,fan,relay,cam_first,cam_last,cam_phase,operation;
    uint16_t required_fuel_us,dwell_us,dead_us,maximum_injection_us,prime_ms;
    int16_t fan_on_centi,fan_off_centi,injection_end_centi;
    uint32_t tooth_zero_centi;
    tunuino_cylinder cylinder[12];
} tunuino_engine_profile;
typedef struct {
    tunuino_outputs outputs;
    tunuino_engine_profile profile;
    tunuino_fuel_corrections fueling;
    tunuino_fuel_correction_state fuel_state;
    tunuino_feature_config features;
    tunuino_feature_inputs feature_inputs;
    tunuino_feature_state feature_state;
    uint32_t generation,armed_at,last_now,last_angle,cycle;
    uint32_t last_injection_us;
    uint32_t scheduled_cycle[28];
    uint8_t configured,mode,have_phase,fan_on,missing;
} tunuino_engine_control;
bool tunuino_engine_profile_decode(const uint8_t bytes[128],uint16_t wheel_slots,
    uint8_t missing,tunuino_engine_profile *profile);
void tunuino_engine_control_init(tunuino_engine_control *e,tunuino_output_write write,void *user);
void tunuino_engine_control_stop(tunuino_engine_control *e,uint8_t fault);
bool tunuino_engine_control_configure(tunuino_engine_control *e,const uint8_t bytes[128],
    uint16_t slots,uint8_t missing,uint32_t generation);
bool tunuino_engine_control_arm(tunuino_engine_control *e,uint32_t now,uint32_t generation);
/* Producer: coherent crank/cam position and the timestamp of its last crank
 * edge. All angles are explicit relative to cylinder TDC; no preset engine. */
void tunuino_engine_control_update(tunuino_engine_control *e,uint32_t now,uint32_t edge,
    const tunuino_position_reading *position,uint16_t slots,uint32_t generation,
    bool calculation_valid,uint16_t ve_centi,int16_t advance_centi,uint32_t map_milli_kpa,
    bool coolant_valid,int32_t coolant_milli);
#endif
