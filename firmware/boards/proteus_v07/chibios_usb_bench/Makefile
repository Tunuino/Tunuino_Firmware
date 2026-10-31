# USB-only bench using the user's pinned ChibiOS startup, HAL and CDC driver.
PROJECT = tunuino-proteus-v07-f427-usb-bench
MCU = cortex-m4
CHIBIOS ?= ../../../../.deps/chibios
ROOT := $(abspath ../../../..)
CONFDIR := ./cfg
BUILDDIR ?= ./build
DEPDIR := $(BUILDDIR)/.dep
USE_OPT = -Os -g3 -ffunction-sections -fdata-sections -fno-common
USE_COPT = -std=c11
USE_CPPOPT =
USE_LINK_GC = yes
USE_LTO = no
USE_VERBOSE_COMPILE = no
USE_SMART_BUILD = yes
USE_PROCESS_STACKSIZE = 0x2000
USE_EXCEPTIONS_STACKSIZE = 0x800
USE_FPU = no
USE_FPU_OPT =
USE_LDOPT =
include $(CHIBIOS)/os/license/license.mk
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk
LDSCRIPT = ./stm32f427_bench.ld
CSRC = $(ALLCSRC) board.c board_io.c board_outputs.c board_status.c usbcfg.c main.c $(wildcard $(ROOT)/firmware/src/*.c)
CPPSRC = $(ALLCPPSRC)
ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)
INCDIR = . $(CONFDIR) $(ALLINC) $(ROOT)/firmware/include
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes -Werror=implicit-function-declaration
CPPWARN = -Wall -Wextra -Wundef
UDEFS = -DSTM32F427xx
UADEFS =
UINCDIR =
ULIBDIR =
ULIBS = -lm -lnosys
RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk
