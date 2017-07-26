@echo off

set FW_DEF_FILE_NAME=..\..\..\source\cfg_board_def.h
set FW_BIN_DIR=..\..\..\binary
set FW_PRV_KEY_NAME=..\..\keys\private.pem

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
set FW_BL_BIN_NAME=%FW_MODEL_NAME%_bootloader_%FW_BL_VER%.hex
set FW_BL_MERGE_BIN_NAME=%FW_MODEL_NAME%_bootloader_merged_AV%FW_APP_VER%_BV%FW_BL_VER%.hex
set FW_SETTING_BIN_NAME=%FW_MODEL_NAME%_bl_setting_%FW_BL_VER%.hex
set FW_PACKAGE_NAME=%FW_MODEL_NAME%_app_dfu_package_%FW_APP_VER%.zip

copy /Y %1 %FW_BIN_DIR%\%FW_BL_BIN_NAME%

del %FW_BIN_DIR%\%FW_SETTING_BIN_NAME%
del %FW_BIN_DIR%\%FW_BL_MERGE_BIN_NAME%
del %FW_BIN_DIR%\%FW_PACKAGE_NAME%

..\..\..\tools\nrftools\nrfutil.exe settings generate --family NRF52 --application %FW_BIN_DIR%\%FW_APP_BIN_NAME% --application-version %FW_APP_VER% --bootloader-version %FW_BL_VER% --bl-settings-version %FW_SETTING_VER% %FW_BIN_DIR%\%FW_SETTING_BIN_NAME%
..\..\..\tools\nrftools\mergehex -m %FW_BIN_DIR%\%FW_BL_BIN_NAME% %FW_BIN_DIR%\%FW_SETTING_BIN_NAME% -o %FW_BIN_DIR%\%FW_BL_MERGE_BIN_NAME%
..\..\..\tools\nrftools\nrfutil.exe pkg generate --hw-version %FW_HW_VER% --sd-req 0x8C --application-version %FW_APP_VER% --application %FW_BIN_DIR%\%FW_APP_BIN_NAME% --key-file %FW_PRV_KEY_NAME% %FW_BIN_DIR%\%FW_PACKAGE_NAME%