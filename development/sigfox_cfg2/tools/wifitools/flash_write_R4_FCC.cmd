@echo off

set UART_PORT=COM32

esptool --port %UART_PORT% -b 460800 write_flash --flash_mode dout --flash_freq 80m --flash_size 1MB --verify 0xfc000 bin_spi_at/esp_init_data_default_V0601_FCC_R4.bin 0xfe000 bin_spi_at/blank_multiboot_0x81000.bin 0x0000 bin_spi_at/boot_v1.6.bin 0x1000 bin_spi_at/SPT_AT_ModemSLP_GPIO_VER_FLASH_FCC_301117.bin 0x81000 bin_spi_at/ESP8266_SDK_INIT_PARAM_26M_20170421.bin
if not "%ERRORLEVEL%" == "0" (
echo WIFI Write Error
pause
)
