/* Copyright (c) 2026 Tunuino. Optional engine strategies, page 4 bytes 68..255. */
#ifndef TUNUINO_ENGINE_FEATURES_H
#define TUNUINO_ENGINE_FEATURES_H
#include <stdbool.h>
#include <stdint.h>
#define TUNUINO_FEATURE_IAT 1u
#define TUNUINO_FEATURE_VOLTAGE 2u
#define TUNUINO_FEATURE_TARGET 4u
#define TUNUINO_FEATURE_LAMBDA 8u
#define TUNUINO_FEATURE_ACCEL 16u
#define TUNUINO_FEATURE_DFCO 32u
#define TUNUINO_FEATURE_LIMIT 64u
#define TUNUINO_FEATURE_LAUNCH 128u
#define TUNUINO_FEATURE_FLEX 256u
#define TUNUINO_FEATURE_PROTECTION 512u
#define TUNUINO_FEATURE_TRIMS 1024u
typedef struct {
    uint16_t flags;
    uint8_t iat_channel,voltage_channel,launch_channel,flex_channel;
    uint16_t iat_raw[3],voltage_raw[2],voltage_value[2],voltage_axis[3],deadtime[3],dwell[3];
    int16_t iat_temperature[3];
    uint16_t target_map[3],target_lambda[3],lambda_p,lambda_i,lambda_limit;
    int16_t lambda_clt;
    uint16_t lambda_min_rpm,lambda_max_rpm,lambda_tps,lambda_delay;
    uint16_t accel_rate,accel_us,accel_ms,dfco_cut,dfco_resume,dfco_tps,dfco_delay;
    int16_t dfco_clt;
    uint16_t limit_cut,limit_resume,launch_cut,launch_resume,launch_raw;
    uint16_t flex_raw[2],flex_fuel;
    int16_t flex_advance,iat_reference,iat_retard_start,protection_clt,launch_advance;
    uint16_t iat_retard,protection_min_v,protection_max_v,transient_delay;
    int16_t fuel_trim[12],spark_trim[12];
} tunuino_feature_config;
typedef struct {
    bool tps_valid,iat_valid,voltage_valid,launch_valid,launch,flex_valid,lambda_valid;
    uint16_t tps,voltage,ethanol;
    int32_t iat;
    uint32_t lambda;
} tunuino_feature_inputs;
typedef struct {
    uint32_t last,run_elapsed,accel_elapsed,dfco_elapsed,quiet_elapsed;
    uint32_t multiplier;
    uint16_t previous_tps,target,extra_us,dead_us,dwell_us;
    int16_t advance_delta;
    int32_t lambda_trim;
    int64_t integral;
    uint8_t sampled,running,run_delay_done,accel_active,dfco_wait,dfco,limited,launch_limited;
    uint8_t fuel_cut,closed_loop,protection,inhibit;
} tunuino_feature_state;
bool tunuino_features_decode(const uint8_t page[256],tunuino_feature_config *out);
void tunuino_features_sample(const tunuino_feature_config *config,bool adc_valid,const uint16_t raw[16],
    bool tps_valid,uint16_t tps,bool lambda_valid,uint32_t lambda,tunuino_feature_inputs *out);
/* False means a required sensor failed or a configured latched protection fired. */
bool tunuino_features_update(const tunuino_feature_config *config,tunuino_feature_state *state,
    const tunuino_feature_inputs *input,uint32_t now,uint32_t rpm,uint32_t map,
    bool coolant_valid,int32_t coolant,uint16_t cranking_rpm,uint16_t dead_us,uint16_t dwell_us);
#endif
