PROJECT_NAME     := gps_example
TARGETS          := gps_example
THIS_DIR         := $(shell dirname $(abspath $(lastword $(MAKEFILE_LIST))))
OUTPUT_DIRECTORY := $(THIS_DIR)/_build_make

SFXTOOLS_BASE := $(abspath $(THIS_DIR)/../tools/sfxtools_embedded)
SDK_ROOT := $(abspath $(THIS_DIR)/../../..)

GIT_VERSION := $(shell git describe --abbrev=4 --dirty --always --tags &>/dev/null || echo 'unknown')

PROJ_DIR := $(THIS_DIR)

SRC_FILES += \
  $(PROJ_DIR)/GPS_example_main.c \
  $(PROJ_DIR)/cfg_board.c \
  $(PROJ_DIR)/cfg_tmp102_module.c \
  $(PROJ_DIR)/cfg_external_sense_gpio.c \
  $(PROJ_DIR)/cfg_dtm.c \
  $(PROJ_DIR)/cfg_examples.c \
  $(PROJ_DIR)/cfg_bma250_module.c \
  $(PROJ_DIR)/nrf_dfu_flash_buttonless.c \
  $(SDK_ROOT)/examples/bsp/bsp.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_state.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_dfu/ble_dfu.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatt_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatts_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/id_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_data.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_data_storage.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_database.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_id.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/pm_buffer.c \
  $(SDK_ROOT)/components/ble/peer_manager/pm_mutex.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_dispatcher.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_manager.c \
  $(SDK_ROOT)/components/ble/ble_dtm/ble_dtm.c \
  $(SDK_ROOT)/components/ble/ble_dtm/ble_dtm_hw_nrf52.c \
  $(SDK_ROOT)/components/drivers_nrf/clock/nrf_drv_clock.c \
  $(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
  $(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
  $(SDK_ROOT)/components/drivers_nrf/uart/nrf_drv_uart.c \
  $(SDK_ROOT)/components/drivers_nrf/spi_master/nrf_drv_spi.c \
  $(SDK_ROOT)/components/drivers_nrf/twis_slave/nrf_drv_twis.c \
  $(SDK_ROOT)/components/drivers_nrf/ble_flash/ble_flash.c \
  $(SDK_ROOT)/components/drivers_nrf/twi_master/nrf_drv_twi.c \
  $(SDK_ROOT)/components/libraries/crc16/crc16.c \
  $(SDK_ROOT)/components/libraries/crc32/crc32.c \
  $(SDK_ROOT)/components/drivers_nrf/hal/nrf_saadc.c \
  $(SDK_ROOT)/components/drivers_nrf/saadc/nrf_drv_saadc.c \
  $(SDK_ROOT)/components/libraries/button/app_button.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/fifo/app_fifo.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/uart/app_uart_fifo.c \
  $(SDK_ROOT)/components/libraries/fstorage/fstorage.c \
  $(SDK_ROOT)/components/libraries/fstorage/fstorage_nosd.c \
  $(SDK_ROOT)/components/libraries/fds/fds.c \
  $(SDK_ROOT)/components/libraries/util/sdk_mapped_flags.c \
  $(SDK_ROOT)/components/libraries/sensorsim/sensorsim.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_ROOT)/external/segger_rtt/RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
  $(SDK_ROOT)/components/nfc/t2t_lib/hal_t2t/hal_nfc_t2t.c \
  $(SDK_ROOT)/components/nfc/ndef/uri/nfc_uri_msg.c \
  $(SDK_ROOT)/components/nfc/ndef/uri/nfc_uri_rec.c \
  $(SDK_ROOT)/components/nfc/ndef/generic/message/nfc_ndef_msg.c \
  $(SDK_ROOT)/components/nfc/ndef/generic/record/nfc_ndef_record.c \
  $(SDK_ROOT)/components/nfc/ndef/launchapp/nfc_launchapp_rec.c \
  $(SDK_ROOT)/components/nfc/ndef/launchapp/nfc_launchapp_msg.c \
  $(SDK_ROOT)/components/nfc/ndef/text/nfc_text_rec.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_settings.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus/ble_nus.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas/ble_bas.c


# Include folders common to all targets
INC_FOLDERS += \
  ${SDK_ROOT}/components/device   $(SDK_ROOT)/components/toolchain/cmsis/include   $(SDK_ROOT)/components/nfc/ndef/generic/message \
  $(SDK_ROOT)/components/drivers_nrf/comp \
  $(SDK_ROOT)/components/nfc/t2t_lib \
  $(SDK_ROOT)/components/drivers_nrf/twi_master \
  $(SDK_ROOT)/components/ble/ble_services/ble_ancs_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias_c \
  $(SDK_ROOT)/components/libraries/pwm \
  $(SDK_ROOT)/components/softdevice/s132/headers/nrf52 \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/ble/ble_services/ble_gls \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/drivers_nrf/i2s \
  $(SDK_ROOT)/components/drivers_nrf/gpiote \
  $(SDK_ROOT)/components/libraries/fifo \
  $(SDK_ROOT)/components/nfc/ndef/generic/record \
  $(SDK_ROOT)/components/libraries/bootloader/dfu \
  $(SDK_ROOT)/components/drivers_nrf/common \
  $(SDK_ROOT)/components/ble/ble_advertising \
  $(SDK_ROOT)/components/drivers_nrf/adc \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs_c \
  $(SDK_ROOT)/components/ble/ble_dtm \
  $(SDK_ROOT)/components/libraries/mailbox \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs_c \
  $(SDK_ROOT)/components/drivers_nrf/uart \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/ble/ble_services/ble_lls \
  $(SDK_ROOT)/components/drivers_nrf/wdt \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas \
  $(SDK_ROOT)/components/drivers_nrf/ble_flash \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/softdevice/s132/headers \
  $(SDK_ROOT)/components/ble/ble_services/ble_ans_c \
  $(SDK_ROOT)/components/libraries/slip \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(SDK_ROOT)/components/drivers_nrf/saadc \
  $(SDK_ROOT)/components/drivers_nrf/hal \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus_c \
  $(SDK_ROOT)/components/drivers_nrf/rtc \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias \
  $(SDK_ROOT)/components/drivers_nrf/ppi \
  $(SDK_ROOT)/components/ble/ble_services/ble_dfu \
  $(SDK_ROOT)/components/drivers_nrf/twis_slave \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs \
  $(SDK_ROOT)/components/ble/ble_services/ble_hts \
  $(SDK_ROOT)/components/drivers_nrf/delay \
  $(SDK_ROOT)/components/libraries/crc16 \
  $(SDK_ROOT)/components/drivers_nrf/timer \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/drivers_nrf/pwm \
  $(SDK_ROOT)/components/libraries/csense_drv \
  $(SDK_ROOT)/components/libraries/csense \
  $(SDK_ROOT)/components/drivers_nrf/rng \
  $(SDK_ROOT)/components/libraries/low_power_pwm \
  $(SDK_ROOT)/components/libraries/hardfault \
  $(SDK_ROOT)/components/ble/ble_services/ble_cscs \
  $(SDK_ROOT)/components/libraries/uart \
  $(SDK_ROOT)/components/libraries/hci \
  $(SDK_ROOT)/components/drivers_nrf/spi_slave \
  $(SDK_ROOT)/components/drivers_nrf/lpcomp \
  $(SDK_ROOT)/components/ble/ble_racp \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/toolchain \
  $(PROJ_DIR)/ \
	$(PROJ_DIR)/pca10040/s132/config \
  $(SDK_ROOT)/components/nfc/ndef/launchapp \
  $(SDK_ROOT)/components/libraries/led_softblink \
  $(SDK_ROOT)/components/drivers_nrf/qdec \
  $(SDK_ROOT)/components/ble/ble_services/ble_cts_c \
  $(SDK_ROOT)/components/drivers_nrf/spi_master \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus \
  $(SDK_ROOT)/components/ble/ble_services/ble_hids \
  $(SDK_ROOT)/components/drivers_nrf/pdm \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/nfc/ndef/text \
  $(SDK_ROOT)/components/ble/peer_manager \
  $(SDK_ROOT)/components/drivers_nrf/swi \
  $(SDK_ROOT)/components/ble/ble_services/ble_tps \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis \
  $(SDK_ROOT)/components/nfc/ndef/uri \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr \
  $(SDK_ROOT)/components/libraries/button \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs_c \
  $(SDK_ROOT)/examples/bsp \
  $(SDK_ROOT)/components/libraries/fds \
  $(SDK_ROOT)/components/libraries/twi \
  $(SDK_ROOT)/components/drivers_nrf/clock \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs \
  $(SDK_ROOT)/components/nfc/t2t_lib/hal_t2t \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs \
  $(SDK_ROOT)/components/libraries/log/src \


# Libraries common to all targets
LIB_FILES += \

# C flags common to all targets
CFLAGS += -DFEATURE_WISOL_APP
CFLAGS += -DNRF52
CFLAGS += -DNRF_DFU_SETTINGS_VERSION=1
CFLAGS += -DBOARD_PCA10040
CFLAGS += -DNRF52832
CFLAGS += -DNRF52_PAN_12
CFLAGS += -DNRF52_PAN_15
CFLAGS += -DNRF52_PAN_58
CFLAGS += -DNRF52_PAN_55
CFLAGS += -DNRF52_PAN_54
CFLAGS += -DNRF52_PAN_31
CFLAGS += -DNRF52_PAN_30
CFLAGS += -DNRF52_PAN_51
CFLAGS += -DNRF52_PAN_36
CFLAGS += -DNRF52_PAN_53
CFLAGS += -DFEATURE_WISOL_DEVICE
CFLAGS += -DS132
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DNRF_LOG_USES_RTT=1
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DNRF_SD_BLE_API_VERSION=3
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DNRF52_PAN_20
CFLAGS += -DNRF52_PAN_64
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF52_PAN_62
CFLAGS += -DNRF52_PAN_63
CFLAGS += -DGIT_VERSION=$(GIT_VERSION)
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -O3 -g3
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin --short-enums  -MP -Wall
# generate dependency output fle
CFLAGS += -MP -MD

# Libraries common to all targets
LIB_FILES += -lnfc_t2t_lib_gcc -lSFM20R1_gcc_lib

# Linker flags
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH_NORDIC)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -Tpca10040/s132/armgcc/sigfox_cfg2_app.ld
LDFLAGS += -L $(SDK_ROOT)/components/nfc/t2t_lib -L$(THIS_DIR)/pca10040/s132/arm5_no_packs/lib -L$(OUTPUT_DIRECTORY)
LDFLAGS += -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16  -Wl,--gc-sections --specs=nano.specs -lc -lnosys

.PHONY: $(TARGETS) default all clean help flash  flash_softdevice
# Default target - first one defined
default: gps_example

# print all targets that can be built
help:
	@echo following targets are available:
	@echo 	$(TARGETS)
	@echo 	flash
	@echo 	flashall

TEMPLATE_PATH_NORDIC := $(SDK_ROOT)/components/toolchain/gcc
TEMPLATE_PATH := $(THIS_DIR)

include $(TEMPLATE_PATH_NORDIC)/Makefile.common
$(foreach target, $(TARGETS), $(call define_target, $(target)))
-include $(foreach target, $(TARGETS), $($(target)_dependencies))

CROSS_COMPILE := $(GNU_INSTALL_ROOT)bin/arm-none-eabi-

SD_HEX := $(SDK_ROOT)/development/sigfox_cfg2/binary/s132_nrf52_3.0.0_softdevice.hex
APP_HEX := $(OUTPUT_DIRECTORY)/gps_example.hex
APP_ELF := $(OUTPUT_DIRECTORY)/gps_example.out
SOFTDEV_HEX := $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_3.0.0_softdevice.hex
BL_HEX := $(SDK_ROOT)/development/sigfox_cfg2/source_bootloader_secure/hex/bootloader.hex
BL_KEY := $(SDK_ROOT)/development/sigfox_cfg2/source_bootloader_secure/keys/private.pem
BL_MERGED := $(OUTPUT_DIRECTORY)/bl_merged.hex
DFU_FILE := $(OUTPUT_DIRECTORY)/$(PROJECT_NAME)_dfu.zip


# Flash the program
flash: $(APP_HEX) $(BL_MERGED)
	@echo Flashing: $(APP_HEX)
	@$(NRFJPROG) --program $(APP_HEX) -f nrf52 --sectorerase --verify
	@$(NRFJPROG) --reset --program $(BL_MERGED) -f nrf52 --sectorerase --verify

# Flash the program
flashall: $(APP_HEX) $(BL_MERGED)
	@echo Flashing: $(SD_HEX) and $(APP_HEX) and $(BL_MERGED)
	@$(NRFJPROG) --program $(SD_HEX) -f nrf52 --chiperase --verify
	@$(NRFJPROG) --program $(APP_HEX) -f nrf52 --sectorerase --verify
	@$(NRFJPROG) --reset --program $(BL_MERGED) -f nrf52 --sectorerase --verify
	
dfu: $(DFU_FILE)

$(DFU_FILE): $(APP_HEX)
	@echo Making DFU file
	@$(NRFUTIL) pkg generate --hw-version 52 --sd-req 0x8C --application-version 204 --application $(APP_HEX) --key-file $(BL_KEY) $(DFU_FILE)

$(BL_MERGED): $(APP_HEX) $(BL_HEX)
	@$(NRFUTIL) settings generate --family NRF52 --application $(APP_HEX) --application-version 204 --bootloader-version 1 --bl-settings-version 1 $(OUTPUT_DIRECTORY)/bl_setting_1.hex
	@$(MERGEHEX) -m $(BL_HEX) $(OUTPUT_DIRECTORY)/bl_setting_1.hex -o $(BL_MERGED)

RELEASE_DIR := $(OUTPUT_DIRECTORY)/$(PROJECT_NAME)_$(GIT_VERSION)
release: $(APP_HEX) $(APP_ELF) $(DFU_FILE) $(BL_MERGED)
	@rm -rf $(RELEASE_DIR)
	@mkdir -p $(RELEASE_DIR)
	@cp -a $(APP_HEX) $(RELEASE_DIR)/$(PROJECT_NAME)_$(GIT_VERSION).hex
	@cp -a $(APP_ELF) $(RELEASE_DIR)/$(PROJECT_NAME)_$(GIT_VERSION).elf
	@cp -a $(BL_MERGED) $(RELEASE_DIR)/$(PROJECT_NAME)_bl_$(GIT_VERSION).hex
	@cp -a $(DFU_FILE) $(RELEASE_DIR)/$(PROJECT_NAME)_$(GIT_VERSION)_dfu.zip
	@echo $(GIT_VERSION) > $(RELEASE_DIR)/version.txt
	@echo '================================================================'
	@echo 'Release directory: $(RELEASE_DIR)'
	@echo '================================================================'
