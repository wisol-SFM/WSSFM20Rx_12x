@echo off

set FW_DEF_FILE_NAME=..\..\..\..\source\cfg_board_def.h
set FW_BIN_DIR=..\..\..\..\binary

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

set FW_APP_BIN_NAME=%FW_MODEL_NAME%_app_%FW_APP_VER%.hex
copy /Y %1 %FW_BIN_DIR%\%FW_APP_BIN_NAME%