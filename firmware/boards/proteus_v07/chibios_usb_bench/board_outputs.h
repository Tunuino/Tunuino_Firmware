/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_BOARD_OUTPUTS_H
#define TUNUINO_BOARD_OUTPUTS_H
#include "tunuino/device.h"
#include "tunuino/position.h"
void board_outputs_start(tunuino_device *device);
void board_outputs_restore(tunuino_device *device,bool boot);
bool board_outputs_autonomous(void);
void board_outputs_update(tunuino_device *device,const tunuino_position_reading *position,uint32_t edge,uint32_t now);
void board_outputs_stop(void);
bool board_outputs_idle(void);
#endif
