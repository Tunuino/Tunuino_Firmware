/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_CHIBIOS_USBCFG_H
#define TUNUINO_CHIBIOS_USBCFG_H
#include "hal.h"
extern const USBConfig usbcfg;
extern const SerialUSBConfig serusbcfg;
extern SerialUSBDriver SDU1;
extern volatile uint32_t tunuino_usb_generation;
void tunuino_usb_serial_init(void);
#endif
