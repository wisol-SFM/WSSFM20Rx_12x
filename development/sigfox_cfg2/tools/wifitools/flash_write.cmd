@echo off

set UART_PORT=COM4

esptool --port %UART_PORT% -b 460800 write_flash --flash_mode dout --flash_freq 80m --flash_size 1MB --verify 0xfc000 bin_spi_at/esp_init_data_default_V0101.bin 0xfe000 bin_spi_at/blank_multiboot_0x81000.bin 0x0000 bin_spi_at/boot_v1.6.bin 0x1000 bin_spi_at/SPI_AT_ModemSLP_GPIO_VER.bin 0x81000 bin_spi_at/ESP8266_RF_TEST_BIN_20170406.bin
if not "%ERRORLEVEL%" == "0" (
echo WIFI Write Error
pause
)
