/* Copyright (c) 2026 Tunuino. Dry-run calculations only; no actuators. */
#include "tunuino/engine_math.h"
#include <stddef.h>

tunuino_engine_math_status tunuino_engine_math_calculate(
    uint32_t rpm_milli, uint16_t slots, uint8_t missing,
    uint16_t ve, int16_t advance, uint32_t map,
    tunuino_engine_math_result *out) {
    if(out==NULL)return TUNUINO_ENGINE_MATH_ARGUMENT;
    if(rpm_milli<1000u||rpm_milli>20000000u||slots<4||slots>120
        ||missing==0||missing>=slots/2||ve==0||ve>30000
        ||advance< -9000||advance>9000||map==0||map>500000u)
        return TUNUINO_ENGINE_MATH_RANGE;
    /* Integer arithmetic prevents NaNs and unstable float rounding on MCU.
     * 60e9 / (RPM*1000) = microseconds per crank revolution. */
    const uint64_t rev=(UINT64_C(60000000000)+rpm_milli/2)/rpm_milli;
    const uint64_t cycle=rev*2u;
    const uint64_t slot=(rev+slots/2u)/slots;
    const uint64_t load=((uint64_t)ve*map+50000u)/100000u;
    const int64_t spark_num=(int64_t)advance*(int64_t)rev;
    const uint64_t abs_spark=(uint64_t)(spark_num<0?-spark_num:spark_num);
    const uint64_t spark_magnitude=(abs_spark+18000u)/36000u;
    if(rev>UINT32_MAX||cycle>UINT32_MAX||slot==0
        ||load>UINT32_MAX||spark_magnitude>INT32_MAX)
        return TUNUINO_ENGINE_MATH_RANGE;
    const tunuino_engine_math_result candidate={
        (uint32_t)rev,(uint32_t)cycle,(uint32_t)slot,(uint32_t)load,
        (int32_t)(spark_num>=0?-(int64_t)spark_magnitude:(int64_t)spark_magnitude)
    };
    *out=candidate;
    return TUNUINO_ENGINE_MATH_OK;
}
