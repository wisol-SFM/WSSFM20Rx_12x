@echo off

IF * == %1* GOTO :EOF
set FW_BIN_DIR=..\..\..\..\binary
set FW_NRF_TOOLS_DIR=..\..\..\..\tools\nrftools
set FW_HEX_FILE_NAME=%1

%FW_NRF_TOOLS_DIR%\mergehex -m %FW_BIN_DIR%\s132_nrf52_3.0.0_softdevice.hex %FW_HEX_FILE_NAME% -o %FW_HEX_FILE_NAME%.hex
