@echo off

set UART_PORT=COM32

esptool --port %UART_PORT% -b 460800 write_flash --flash_mode dout --flash_freq 80m --flash_size 1MB --verify 0x0000 Certification_Test/ESP8285_CE_Adaptivity_20171018.bin
if not "%ERRORLEVEL%" == "0" (
echo WIFI Write Error
pause
)
