@echo off

set FW_DEF_FILE_NAME=..\..\..\..\source\cfg_board_def.h
set FW_BIN_DIR=..\..\..\..\binary
set FW_NRF_TOOLS_DIR=..\..\..\..\tools\nrftools

IF * == %1* GOTO :EOF
set FW_HEX_FILE_NAME=%1

set cmd="findstr REPLACE_DEVICE_DEFINE_HERE %FW_DEF_FILE_NAME%"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
echo CFG_LINE_INPUT : %CFG_LINE_INPUT%
set CDEV_BOARD_TYPE_DEFINE=%CFG_LINE_INPUT:~47%
echo CDEV_BOARD_TYPE_DEFINE : %CDEV_BOARD_TYPE_DEFINE%
set DEVICE_TYPE=Unknown
if "%CDEV_BOARD_TYPE_DEFINE:~0,14%" == "CDEV_BOARD_EVB" set DEVICE_TYPE=EVB
if "%CDEV_BOARD_TYPE_DEFINE:~0,16%" == "CDEV_BOARD_IHERE" set DEVICE_TYPE=iHere
if "%CDEV_BOARD_TYPE_DEFINE:~0,18%" == "CDEV_BOARD_IHEREV2" set DEVICE_TYPE=iHereV2
if "%CDEV_BOARD_TYPE_DEFINE:~0,21%" == "CDEV_BOARD_IHERE_MINI" set DEVICE_TYPE=iHereMini
if "%CDEV_BOARD_TYPE_DEFINE:~0,13%" == "CDEV_BOARD_M3" set DEVICE_TYPE=M3
echo DEVICE_TYPE : %DEVICE_TYPE%

set cmd="findstr CDEV_MODEL_NAME %FW_DEF_FILE_NAME%"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
set FW_MODEL_NAME=%CFG_LINE_INPUT:~25,6%
echo MODEL NAME : %FW_MODEL_NAME%

set cmd="findstr CDEV_SW_VER_MAJOR %FW_DEF_FILE_NAME%"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
set SW_VER_MAJOR=%CFG_LINE_INPUT:~27,1%
set cmd="findstr CDEV_SW_VER_MINOR %FW_DEF_FILE_NAME%"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
set SW_VER_MINOR=%CFG_LINE_INPUT:~27,2%
set FW_APP_VER=%SW_VER_MAJOR%%SW_VER_MINOR%
echo APP VERSION : %FW_APP_VER%

set cmd="findstr CDEV_BL_VER %FW_DEF_FILE_NAME%"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
set FW_BL_VER=%CFG_LINE_INPUT:~21,1%
echo BOOTLOADER VER : %FW_BL_VER%

set cmd="findstr CDEV_BL_SETTING_VER %FW_DEF_FILE_NAME%"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
set FW_SETTING_VER=%CFG_LINE_INPUT:~29,1%
echo BL SETTING VER : %FW_SETTING_VER%

set cmd="findstr CDEV_HW_VER %FW_DEF_FILE_NAME%"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
set FW_HW_VER=%CFG_LINE_INPUT:~21,2%
echo DFU HW VER : %FW_HW_VER%

set FW_APP_BIN_NAME=%FW_MODEL_NAME%_app_%FW_APP_VER%_%DEVICE_TYPE%.hex
copy /Y %FW_HEX_FILE_NAME% %FW_BIN_DIR%\%FW_APP_BIN_NAME%

%FW_NRF_TOOLS_DIR%\mergehex -m %FW_BIN_DIR%\s132_nrf52_3.0.0_softdevice.hex %FW_HEX_FILE_NAME% -o %FW_HEX_FILE_NAME%.hex
