/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_OUTPUTS_H
#define TUNUINO_OUTPUTS_H
#include <stdbool.h>
#include <stdint.h>
#define TUNUINO_OUTPUT_COUNT 28u
#define TUNUINO_OUTPUT_LATE 1u
#define TUNUINO_OUTPUT_WATCHDOG 2u
#define TUNUINO_OUTPUT_SYNC 4u
#define TUNUINO_OUTPUT_CALIBRATION 8u
#define TUNUINO_OUTPUT_OVERBOOST 16u
#define TUNUINO_OUTPUT_PROTECTION 32u
typedef void (*tunuino_output_write)(void *user,unsigned channel,bool high);
typedef struct { uint32_t start,end; uint8_t state; } tunuino_output_pulse;
typedef struct {uint32_t period,next,cycle,high,requested;uint8_t enabled,phase;} tunuino_output_pwm;
/* Serialize thread and ISR calls with the platform system lock. Times are a
 * wrapping 1 MHz counter; all deadlines are less than one second ahead. */
typedef struct {
    tunuino_output_pulse pulse[TUNUINO_OUTPUT_COUNT];
    tunuino_output_pwm pwm[16];
    tunuino_output_write write;
    void *user;
    uint32_t active_mask,lease;
    uint8_t armed,fault;
} tunuino_outputs;
void tunuino_outputs_init(tunuino_outputs *o,tunuino_output_write write,void *user);
void tunuino_outputs_stop(tunuino_outputs *o,uint8_t fault);
void tunuino_outputs_arm(tunuino_outputs *o,uint32_t now,uint32_t lease_us);
bool tunuino_outputs_keepalive(tunuino_outputs *o,uint32_t now,uint32_t lease_us);
bool tunuino_outputs_pulse(tunuino_outputs *o,unsigned channel,uint32_t now,uint32_t start,uint32_t duration);
/* Dedicated disarmed relay test; callers must validate the assigned auxiliary channel. */
bool tunuino_outputs_aux_test(tunuino_outputs *o,unsigned channel,uint32_t now,uint32_t duration_us);
bool tunuino_outputs_level(tunuino_outputs *o,unsigned channel,bool high);
/* Cancel only an injector pulse. Ignition charge is never truncated by fuel cuts. */
void tunuino_outputs_cancel_injector(tunuino_outputs *o,unsigned channel);
/* PWM shares the edge/lease IRQ, LS channels only. 10000 = 100% duty.
 * Frequency is fixed while enabled; duty changes latch at the next period,
 * except a zero request which de-energizes immediately. */
bool tunuino_outputs_pwm(tunuino_outputs *o,unsigned channel,uint32_t now,uint16_t hz,uint16_t duty);
void tunuino_outputs_poll(tunuino_outputs *o,uint32_t now);
uint32_t tunuino_outputs_next(const tunuino_outputs *o,uint32_t now);
uint32_t tunuino_outputs_pending(const tunuino_outputs *o);
#endif
