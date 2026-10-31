/* Copyright (c) 2026 Tunuino. Proteus v0.7 F427 LED status tasks.
 * PE3 critical error, PE4 communication, PE5 engine state, PE6 warning.
 * Active-high GPIO outputs, independent of calibration and actuator pins. */
#include "ch.h"
#include "hal.h"
#include "board.h"
#include "board_status.h"

static virtual_timer_t communication_timer, engine_timer;
static bool usb_ready, have_activity, writing_flash, configuration_pending, trigger_warning;
static bool rotating, cranking, communication_on, running_on;
static unsigned startup_step;
static systime_t last_activity;
static systime_t last_trigger_error;
static uint32_t previous_trigger_errors;

static void led(unsigned pad, bool on) {
    if (on) palSetPad(GPIOE, pad);
    else palClearPad(GPIOE, pad);
}

static bool critical(void) {
    /* Code 20 is a recoverable transport interruption, not a CPU failure. */
    return tunuino_bench_fault != 0 && tunuino_bench_fault != 20;
}

static void communication_tick(virtual_timer_t *timer, void *argument) {
    (void)argument;
    unsigned delay = 100;
    chSysLockFromISR();
    if (critical()) {
        startup_step = 2;
        led(3, true);
    } else if (startup_step < 2) {
        bool on = startup_step++ == 0;
        for (unsigned pad = 3; pad <= 6; ++pad) led(pad, on);
        chVTSetI(timer, TIME_MS2I(delay), communication_tick, NULL);
        chSysUnlockFromISR();
        return;
    }

    bool active = usb_ready && have_activity &&
        (systime_t)(chVTGetSystemTimeX() - last_activity) < TIME_MS2I(1000);
    unsigned on_ms, off_ms;
    if (critical()) { on_ms = 450; off_ms = 50; }
    else if (active) { on_ms = 33; off_ms = 100; }
    else if (writing_flash || configuration_pending) { on_ms = 500; off_ms = 500; }
    else { on_ms = usb_ready ? 99 : 33; off_ms = on_ms * 3 / 5; }
    communication_on = !communication_on;
    led(4, communication_on);
    led(3, critical());
    led(6, trigger_warning || tunuino_bench_fault == 20);
    delay = communication_on ? on_ms : off_ms;
    chVTSetI(timer, TIME_MS2I(delay), communication_tick, NULL);
    chSysUnlockFromISR();
}

static void engine_tick(virtual_timer_t *timer, void *argument) {
    (void)argument;
    chSysLockFromISR();
    if (startup_step >= 2) {
        running_on = !critical() && (rotating ? !running_on : cranking);
        led(5, running_on);
    }
    chVTSetI(timer, TIME_MS2I(50), engine_tick, NULL);
    chSysUnlockFromISR();
}

void board_status_start(void) {
    usb_ready = have_activity = writing_flash = trigger_warning = false;
    configuration_pending = false;
    rotating = cranking = communication_on = running_on = false;
    startup_step = 0;
    previous_trigger_errors = 0;
    /* Set inactive latches before enabling the four dedicated LED outputs. */
    for (unsigned pad = 3; pad <= 6; ++pad) {
        led(pad, false);
        palSetPadMode(GPIOE, pad, PAL_MODE_OUTPUT_PUSHPULL);
    }
    chVTObjectInit(&communication_timer);
    chVTObjectInit(&engine_timer);
    chVTSet(&communication_timer, TIME_MS2I(100), communication_tick, NULL);
    chVTSet(&engine_timer, TIME_MS2I(50), engine_tick, NULL);
}

void board_status_usb(bool ready) {
    chSysLock();
    board_status_usb_i(ready);
    chSysUnlock();
}

void board_status_usb_i(bool ready) {
    if (!ready) have_activity = false;
    usb_ready = ready;
}

void board_status_activity(void) {
    chSysLock();
    last_activity = chVTGetSystemTimeX();
    have_activity = true;
    chSysUnlock();
}

void board_status_inputs(const tunuino_device *device) {
    bool valid = (device->live.valid_mask & TUNUINO_LIVE_RPM_VALID) != 0;
    chSysLock();
    configuration_pending = device->dirty;
    rotating = valid && device->live.rpm_milli >= TUNUINO_LED_CRANKING_RPM * 1000u;
    cranking = valid && device->live.rpm_milli > 0 && !rotating;
    if (device->trigger_errors != previous_trigger_errors) {
        last_trigger_error = chVTGetSystemTimeX();
        trigger_warning = true;
    } else if ((systime_t)(chVTGetSystemTimeX() - last_trigger_error) >= TIME_MS2I(500)) {
        trigger_warning = device->active[2] && device->trigger_errors != 0 && !device->trigger_valid;
    }
    previous_trigger_errors = device->trigger_errors;
    chSysUnlock();
}

void board_status_flash(bool writing) {
    chSysLock();
    writing_flash = writing;
    chSysUnlock();
}

void board_status_halt(void) {
    /* No timers, allocation or locks on this terminal path. */
    palSetPad(GPIOE, 3);
    palSetPadMode(GPIOE, 3, PAL_MODE_OUTPUT_PUSHPULL);
}
