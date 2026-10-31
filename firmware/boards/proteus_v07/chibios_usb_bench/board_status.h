/* Copyright (c) 2026 Tunuino. Proteus v0.7 F427 status indicators. */
#ifndef TUNUINO_BOARD_STATUS_H
#define TUNUINO_BOARD_STATUS_H
#include <stdbool.h>
#include "tunuino/device.h"

/* Display threshold only; does not change injection/ignition decisions. */
#define TUNUINO_LED_CRANKING_RPM 400u
void board_status_start(void);
void board_status_usb(bool ready);
/* Caller holds the ChibiOS ISR/system lock. */
void board_status_usb_i(bool ready);
void board_status_activity(void);
void board_status_inputs(const tunuino_device *device);
void board_status_flash(bool writing);
/* No RTOS calls: also usable by the system-halt hook with IRQs disabled. */
void board_status_halt(void);
#endif
