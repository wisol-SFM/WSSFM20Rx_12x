PROJECT_NAME     := blinky_FreeRTOS_pca10028
TARGETS          := nrf51422_xxac
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../../../../../..
PROJ_DIR := ../../..

$(OUTPUT_DIRECTORY)/nrf51422_xxac.out: \
  LINKER_SCRIPT  := blinky_FreeRTOS_gcc_nrf51.ld
# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/components/libraries/button/app_button.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer_freertos.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/external/freertos/source/croutine.c \
  $(SDK_ROOT)/external/freertos/source/event_groups.c \
  $(SDK_ROOT)/external/freertos/source/portable/MemMang/heap_1.c \
  $(SDK_ROOT)/external/freertos/source/list.c \
  $(SDK_ROOT)/external/freertos/portable/GCC/nrf51/port.c \
  $(SDK_ROOT)/external/freertos/portable/CMSIS/nrf51/port_cmsis.c \
  $(SDK_ROOT)/external/freertos/portable/CMSIS/nrf51/port_cmsis_systick.c \
  $(SDK_ROOT)/external/freertos/source/queue.c \
  $(SDK_ROOT)/external/freertos/source/tasks.c \
  $(SDK_ROOT)/external/freertos/source/timers.c \
  $(SDK_ROOT)/components/drivers_nrf/clock/nrf_drv_clock.c \
  $(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
  $(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd/nrf_nvic.c \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd/nrf_soc.c \
  $(PROJ_DIR)/../../bsp/bsp.c \
  $(PROJ_DIR)/main.c \
  $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf51.S \
  $(SDK_ROOT)/components/toolchain/system_nrf51.c \

# Include folders common to all targets
INC_FOLDERS += \
  $(PROJ_DIR)/config/blinky_FreeRTOS_pca10028 \
  $(PROJ_DIR)/config \
  $(SDK_ROOT)/components \
  $(PROJ_DIR) \
  $(PROJ_DIR)/../../bsp \
  $(SDK_ROOT)/external/freertos/source/include \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/external/freertos/config \
  $(SDK_ROOT)/components/libraries/util \
  ../config \
  $(SDK_ROOT)/components/device \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/libraries/button \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/external/freertos/portable/CMSIS/nrf51 \
  $(SDK_ROOT)/components/drivers_nrf/gpiote \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd \
  $(SDK_ROOT)/components/drivers_nrf/hal \
  $(SDK_ROOT)/components/toolchain/gcc \
  $(SDK_ROOT)/components/toolchain \
  $(SDK_ROOT)/components/drivers_nrf/common \
  $(SDK_ROOT)/components/drivers_nrf/clock \
  $(SDK_ROOT)/external/freertos/portable/GCC/nrf51 \
  $(SDK_ROOT)/components/libraries/log/src \

# Libraries common to all targets
LIB_FILES += \

# C flags common to all targets
CFLAGS += -DNRF51
CFLAGS += -DNRF51422
CFLAGS += -DFREERTOS
CFLAGS += -DBOARD_PCA10028
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs
CFLAGS +=  -Wall -Werror -O3 -g3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 
# generate dependency output file
CFLAGS += -MP -MD

# C++ flags common to all targets
CXXFLAGS += \

# Assembler flags common to all targets
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF51
ASMFLAGS += -DNRF51422
ASMFLAGS += -DFREERTOS
ASMFLAGS += -DBOARD_PCA10028

# Linker flags
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys







.PHONY: $(TARGETS) default all clean help flash 
# Default target - first one defined
default: nrf51422_xxac

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo 	nrf51422_xxac

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

include $(TEMPLATE_PATH)/Makefile.common
$(foreach target, $(TARGETS), $(call define_target, $(target)))
-include $(foreach target, $(TARGETS), $($(target)_dependencies))

# Flash the program
flash: $(OUTPUT_DIRECTORY)/nrf51422_xxac.hex
	@echo Flashing: $<
	nrfjprog --program $< -f nrf51 --chiperase
	nrfjprog --reset -f nrf51

