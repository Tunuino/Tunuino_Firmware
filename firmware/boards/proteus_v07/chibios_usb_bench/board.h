/* Copyright (c) 2026 Tunuino. USB-only Proteus v0.7 / STM32F427. */
#ifndef TUNUINO_CHIBIOS_BOARD_H
#define TUNUINO_CHIBIOS_BOARD_H
#define BOARD_NAME "Tunuino Proteus v0.7 F427 USB bench"
#define STM32_LSECLK 0U
#define STM32_HSECLK 8000000U
#define STM32_VDD 330U
#define BOARD_OTG_NOVBUSSENS TRUE
#if !defined(_FROM_ASM_)
#include <stdint.h>
void boardInit(void);
void board_uid(uint8_t uid[12]);
uint64_t board_millis(void);
extern volatile uint32_t tunuino_bench_fault;
#endif
#endif
