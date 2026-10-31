/* Copyright (c) 2026 Tunuino. Early clocks/USB pads; outputs initialize separately. */
#include "ch.h"
#include "hal.h"
#include "board.h"
#include "board_io.h"

_Static_assert(STM32_SYSCLK == 168000000, "Unexpected Proteus system clock");
_Static_assert(STM32_PLL48CLK == 48000000, "USB requires 48 MHz");

volatile uint32_t tunuino_bench_fault;

/* ChibiOS startup calls this before C runtime initialization. */
void __early_init(void) {
  stm32_clock_init();
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOEEN;
  (void)RCC->AHB1ENR;
}

void boardInit(void) {
  palSetPadMode(GPIOA, 11, PAL_MODE_ALTERNATE(10));
  palSetPadMode(GPIOA, 12, PAL_MODE_ALTERNATE(10));
}

void board_uid(uint8_t uid[12]) {
  const volatile uint8_t *source = (const volatile uint8_t *)UID_BASE;
  for (unsigned i = 0; i < 12; ++i) uid[i] = source[i];
}

uint64_t board_millis(void) {
  return board_io_time_us()/1000;
}
