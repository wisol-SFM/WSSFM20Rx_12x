@echo off
DEL /Q development\sigfox_cfg2\binary\*_app_*.hex
DEL /Q development\sigfox_cfg2\binary\*_app_dfu_package_*.zip
DEL /Q development\sigfox_cfg2\binary\*_bl_setting_*.hex
DEL /Q development\sigfox_cfg2\binary\*_bootloader_*.hex
DEL /Q development\sigfox_cfg2\binary\*_factory_*.hex
DEL /Q development\sigfox_cfg2\binary\*.dll
DEL /Q development\sigfox_cfg2\binary\*.exe
DEL /Q development\sigfox_cfg2\binary\*.cmd
DEL /Q development\sigfox_cfg2\binary\*.ini

RD /Q /S development\sigfox_cfg2\source\pca10040\s132\arm5_no_packs\_build
RD /Q /S development\sigfox_cfg2\source\pca10040\s132\arm5_no_packs\_build_gcc
RD /Q /S development\sigfox_cfg2\source\pca10040\s132\armgcc\_build
RD /Q /S development\sigfox_cfg2\source\_build_make
RD /Q /S development\sigfox_cfg2\source\pca10040\s132\arm5_no_packs\RTE\_nrf52832_xxaa
DEL /Q development\sigfox_cfg2\source\pca10040\s132\arm5_no_packs\JLinkLog.txt
DEL /Q development\sigfox_cfg2\source\pca10040\s132\arm5_no_packs\JLinkSettings.ini
DEL /Q development\sigfox_cfg2\source\pca10040\s132\arm5_no_packs\sigfox_cfg2_pca10040_s132.uvguix.*
DEL /Q development\sigfox_cfg2\source\pca10040\s132\arm5_no_packs\sigfox_cfg2_pca10040_s132_gcc.uvguix.*
DEL /Q development\sigfox_cfg2\source\pca10040\s132\arm5_no_packs\EventRecorderStub.scvd
RD /Q /S development\sigfox_cfg2\source_bootloader_secure\pca10040\arm5_no_packs\_build
RD /Q /S development\sigfox_cfg2\source_bootloader_secure\pca10040\arm5_no_packs\_build_gcc
RD /Q /S development\sigfox_cfg2\source_bootloader_secure\pca10040\armgcc\_build
RD /Q /S development\sigfox_cfg2\source_bootloader_secure\_build_make
RD /Q /S development\sigfox_cfg2\source_bootloader_secure\pca10040\arm5_no_packs\RTE
DEL /Q development\sigfox_cfg2\source_bootloader_secure\pca10040\arm5_no_packs\secure_dfu_secure_dfu_ble_s132_pca10040.uvguix.*
RD /Q /S development\sigfox_cfg2\source_bootloader_secure\pca10040_debug\arm5_no_packs\_build
RD /Q /S development\sigfox_cfg2\source_bootloader_secure\pca10040_debug\arm5_no_packs\_build_gcc
RD /Q /S development\sigfox_cfg2\source_bootloader_secure\pca10040_debug\armgcc\_build
RD /Q /S development\sigfox_cfg2\source_bootloader_secure\_build_make
RD /Q /S development\sigfox_cfg2\source_bootloader_secure\pca10040_debug\arm5_no_packs\RTE\_nrf52832_xxaa_s132
DEL /Q development\sigfox_cfg2\source_bootloader_secure\pca10040_debug\arm5_no_packs\secure_dfu_secure_dfu_ble_s132_pca10040_debug.uvguix.*
RD /Q development\sigfox_cfg2\source_direct_test_mode\pca10040\s132\arm5_no_packs\_build
RD /Q development\sigfox_cfg2\source_direct_test_mode\pca10040\s132\arm5_no_packs\_build_gcc
RD /Q development\sigfox_cfg2\source_direct_test_mode\pca10040\s132\armgcc\_build
RD /Q development\sigfox_cfg2\source_direct_test_mode\_build_make
RD /Q /S development\sigfox_cfg2\source_direct_test_mode\pca10040\s132\arm5_no_packs\RTE\_nrf52832_xxaa
DEL /Q development\source_direct_test_mode\pca10040\s132\arm5_no_packs\direct_test_mode_pca10040_s132.uvguix.*

RD /Q /S development\sigfox_cfg2\source_gps_example\pca10040\s132\arm5_no_packs\_build
RD /Q /S development\sigfox_cfg2\source_gps_example\pca10040\s132\arm5_no_packs\_build_gcc
RD /Q /S development\sigfox_cfg2\source_gps_example\pca10040\s132\armgcc\_build
RD /Q /S development\sigfox_cfg2\source_gps_example\_build_make
RD /Q /S development\sigfox_cfg2\source_gps_example\pca10040\s132\arm5_no_packs\RTE\_nrf52832_xxaa
DEL /Q development\sigfox_cfg2\source_gps_example\pca10040\s132\arm5_no_packs\JLinkLog.txt
DEL /Q development\sigfox_cfg2\source_gps_example\pca10040\s132\arm5_no_packs\JLinkSettings.ini
DEL /Q development\sigfox_cfg2\source_gps_example\pca10040\s132\arm5_no_packs\GPS_example_keil.uvguix.*
DEL /Q development\sigfox_cfg2\source_gps_example\pca10040\s132\arm5_no_packs\GPS_example_gcc.uvguix.*
DEL /Q development\sigfox_cfg2\source_gps_example\pca10040\s132\arm5_no_packs\EventRecorderStub.scvd

RD /Q /S development\sigfox_cfg2\source_sensors_example\pca10040\s132\arm5_no_packs\_build
RD /Q /S development\sigfox_cfg2\source_sensors_example\pca10040\s132\arm5_no_packs\_build_gcc
RD /Q /S development\sigfox_cfg2\source_sensors_example\pca10040\s132\armgcc\_build
RD /Q /S development\sigfox_cfg2\source_sensors_example\_build_make
RD /Q /S development\sigfox_cfg2\source_sensors_example\pca10040\s132\arm5_no_packs\RTE\_nrf52832_xxaa
DEL /Q development\sigfox_cfg2\source_sensors_example\pca10040\s132\arm5_no_packs\JLinkLog.txt
DEL /Q development\sigfox_cfg2\source_sensors_example\pca10040\s132\arm5_no_packs\JLinkSettings.ini
DEL /Q development\sigfox_cfg2\source_sensors_example\pca10040\s132\arm5_no_packs\sensors_example_keil.uvguix.*
DEL /Q development\sigfox_cfg2\source_sensors_example\pca10040\s132\arm5_no_packs\sensors_example_gcc.uvguix.*
DEL /Q development\sigfox_cfg2\source_sensors_example\pca10040\s132\arm5_no_packs\EventRecorderStub.scvd

RD /Q /S development\sigfox_cfg2\source_sigfox_example\pca10040\s132\arm5_no_packs\_build
RD /Q /S development\sigfox_cfg2\source_sigfox_example\pca10040\s132\arm5_no_packs\_build_gcc
RD /Q /S development\sigfox_cfg2\source_sigfox_example\pca10040\s132\armgcc\_build
RD /Q /S development\sigfox_cfg2\source_sigfox_example\_build_make
RD /Q /S development\sigfox_cfg2\source_sigfox_example\pca10040\s132\arm5_no_packs\RTE\_nrf52832_xxaa
DEL /Q development\sigfox_cfg2\source_sigfox_example\pca10040\s132\arm5_no_packs\JLinkLog.txt
DEL /Q development\sigfox_cfg2\source_sigfox_example\pca10040\s132\arm5_no_packs\JLinkSettings.ini
DEL /Q development\sigfox_cfg2\source_sigfox_example\pca10040\s132\arm5_no_packs\sigfox_example_keil.uvguix.*
DEL /Q development\sigfox_cfg2\source_sigfox_example\pca10040\s132\arm5_no_packs\sigfox_example_gcc.uvguix.*
DEL /Q development\sigfox_cfg2\source_sigfox_example\pca10040\s132\arm5_no_packs\EventRecorderStub.scvd

RD /Q /S development\sigfox_cfg2\source_wifi_example\pca10040\s132\arm5_no_packs\_build
RD /Q /S development\sigfox_cfg2\source_wifi_example\pca10040\s132\arm5_no_packs\_build_gcc
RD /Q /S development\sigfox_cfg2\source_wifi_example\pca10040\s132\armgcc\_build
RD /Q /S development\sigfox_cfg2\source_wifi_example\_build_make
RD /Q /S development\sigfox_cfg2\source_wifi_example\pca10040\s132\arm5_no_packs\RTE\_nrf52832_xxaa
DEL /Q development\sigfox_cfg2\source_wifi_example\pca10040\s132\arm5_no_packs\JLinkLog.txt
DEL /Q development\sigfox_cfg2\source_wifi_example\pca10040\s132\arm5_no_packs\JLinkSettings.ini
DEL /Q development\sigfox_cfg2\source_wifi_example\pca10040\s132\arm5_no_packs\wifi_example_keil.uvguix.*
DEL /Q development\sigfox_cfg2\source_wifi_example\pca10040\s132\arm5_no_packs\wifi_example_gcc.uvguix.*
DEL /Q development\sigfox_cfg2\source_wifi_example\pca10040\s132\arm5_no_packs\EventRecorderStub.scvd

RD /Q /S development\sigfox_cfg2\source_simplework_example\pca10040\s132\arm5_no_packs\_build
RD /Q /S development\sigfox_cfg2\source_simplework_example\pca10040\s132\arm5_no_packs\_build_gcc
RD /Q /S development\sigfox_cfg2\source_simplework_example\pca10040\s132\armgcc\_build
RD /Q /S development\sigfox_cfg2\source_simplework_example\_build_make
RD /Q /S development\sigfox_cfg2\source_simplework_example\pca10040\s132\arm5_no_packs\RTE\_nrf52832_xxaa
DEL /Q development\sigfox_cfg2\source_simplework_example\pca10040\s132\arm5_no_packs\JLinkLog.txt
DEL /Q development\sigfox_cfg2\source_simplework_example\pca10040\s132\arm5_no_packs\JLinkSettings.ini
DEL /Q development\sigfox_cfg2\source_simplework_example\pca10040\s132\arm5_no_packs\simplework_example_keil.uvguix.*
DEL /Q development\sigfox_cfg2\source_simplework_example\pca10040\s132\arm5_no_packs\simplework_example_gcc.uvguix.*
DEL /Q development\sigfox_cfg2\source_simplework_example\pca10040\s132\arm5_no_packs\EventRecorderStub.scvd

RD /Q /S development\sigfox_cfg2\tools\decompress
MKDIR development\sigfox_cfg2\tools\decompress
ECHO keep > development\sigfox_cfg2\tools\decompress\.keep

