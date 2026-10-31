/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_BOARD_IO_H
#define TUNUINO_BOARD_IO_H
#include "tunuino/device.h"
void board_io_start(tunuino_device *device);
void board_io_update(tunuino_device *device);
void board_io_resync(tunuino_device *device);
void board_io_usb_recover(tunuino_device *device);
void board_io_phase(tunuino_device *device,uint8_t first,uint8_t last,uint8_t phase);
uint64_t board_io_time_us(void);
bool board_io_save(void *user,const uint8_t *bytes,size_t length);
uint64_t board_io_clock(void *user);
#endif
