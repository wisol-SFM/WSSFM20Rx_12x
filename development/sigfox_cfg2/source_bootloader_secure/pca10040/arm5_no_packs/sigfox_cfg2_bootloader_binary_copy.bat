@echo off

set FW_DEF_FILE_NAME=..\..\..\source\cfg_board_def.h
set FW_BIN_DIR=..\..\..\binary
set FW_NRF_TOOLS_DIR=..\..\..\tools\nrftools
set FW_PRV_KEY_NAME=..\..\keys\private.pem

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
set FW_BL_BIN_NAME=%FW_MODEL_NAME%_bootloader_%FW_BL_VER%.hex
set FW_BL_MERGE_BIN_NAME=%FW_MODEL_NAME%_bootloader_merged_AV%FW_APP_VER%_BV%FW_BL_VER%.hex
set FW_FACTORY_BIN_NAME=%FW_MODEL_NAME%_factory_AV%FW_APP_VER%_BV%FW_BL_VER%_%DEVICE_TYPE%.hex
set FW_SETTING_BIN_NAME=%FW_MODEL_NAME%_bl_setting_%FW_BL_VER%.hex
set FW_PACKAGE_NAME=%FW_MODEL_NAME%_app_dfu_package_%FW_APP_VER%_%DEVICE_TYPE%.zip
set FW_APP_TEMP_NAME=TMP_APP.hex
set FW_FACTORY_WRITE_NAME=%FW_MODEL_NAME%_factory_write.cmd

copy /Y %1 %FW_BIN_DIR%\%FW_BL_BIN_NAME%

del %FW_BIN_DIR%\%FW_SETTING_BIN_NAME%
del %FW_BIN_DIR%\%FW_BL_MERGE_BIN_NAME%
del %FW_BIN_DIR%\%FW_PACKAGE_NAME%
del %FW_BIN_DIR%\%FW_APP_TEMP_NAME%

%FW_NRF_TOOLS_DIR%\nrfutil.exe settings generate --family NRF52 --application %FW_BIN_DIR%\%FW_APP_BIN_NAME% --application-version %FW_APP_VER% --bootloader-version %FW_BL_VER% --bl-settings-version %FW_SETTING_VER% %FW_BIN_DIR%\%FW_SETTING_BIN_NAME%
%FW_NRF_TOOLS_DIR%\mergehex -m %FW_BIN_DIR%\%FW_BL_BIN_NAME% %FW_BIN_DIR%\%FW_SETTING_BIN_NAME% -o %FW_BIN_DIR%\%FW_BL_MERGE_BIN_NAME%
%FW_NRF_TOOLS_DIR%\nrfutil.exe pkg generate --hw-version %FW_HW_VER% --sd-req 0x8C --application-version %FW_APP_VER% --application %FW_BIN_DIR%\%FW_APP_BIN_NAME% --key-file %FW_PRV_KEY_NAME% %FW_BIN_DIR%\%FW_PACKAGE_NAME%
findstr /N :020000041000EA %FW_BIN_DIR%\%FW_APP_BIN_NAME%
if "%ERRORLEVEL%" == "0" (
goto MAKE_APP_HEX
) else (
goto COPY_APP_HEX
)

:MAKE_APP_HEX
echo Remove write NRF_UICR_MBR_PARAMS_PAGE_ADDRESS[0x10001018]
setlocal enabledelayedexpansion
set cmd="findstr /N :020000041000EA %FW_BIN_DIR%\%FW_APP_BIN_NAME%"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
set FW_APP_LINE_MAX=%CFG_LINE_INPUT:~0,-16%
set FW_APP_LINE=1
for /f "usebackq tokens=* delims=" %%f in (%FW_BIN_DIR%\%FW_APP_BIN_NAME%) do (
    if !FW_APP_LINE! LSS %FW_APP_LINE_MAX% echo %%f>>%FW_BIN_DIR%\%FW_APP_TEMP_NAME%
    set /a FW_APP_LINE+=1
)
echo :00000001FF>>%FW_BIN_DIR%\%FW_APP_TEMP_NAME%
goto MERGE_ALL

:COPY_APP_HEX
echo not found NRF_UICR_MBR_PARAMS_PAGE_ADDRESS[0x10001018]
copy /Y %FW_BIN_DIR%\%FW_APP_BIN_NAME% %FW_BIN_DIR%\%FW_APP_TEMP_NAME%
goto MERGE_ALL

:MERGE_ALL
..\..\..\tools\nrftools\mergehex -m %FW_BIN_DIR%\s132_nrf52_3.0.0_softdevice.hex %FW_BIN_DIR%\%FW_APP_TEMP_NAME% %FW_BIN_DIR%\%FW_BL_MERGE_BIN_NAME% -o %FW_BIN_DIR%\%FW_FACTORY_BIN_NAME%
del %FW_BIN_DIR%\%FW_APP_TEMP_NAME%
copy %FW_NRF_TOOLS_DIR%\nrfjprog.* %FW_BIN_DIR%\
copy %FW_NRF_TOOLS_DIR%\jlinkarm_nrf52_nrfjprog.dll %FW_BIN_DIR%\
echo @echo off> %FW_BIN_DIR%\%FW_FACTORY_WRITE_NAME%
echo nrfjprog --reset --program %FW_FACTORY_BIN_NAME% -f nrf52 --chiperase --verify>> %FW_BIN_DIR%\%FW_FACTORY_WRITE_NAME%
echo if not "%%ERRORLEVEL%%" == "0" (>> %FW_BIN_DIR%\%FW_FACTORY_WRITE_NAME%
echo echo Factory Write Error>> %FW_BIN_DIR%\%FW_FACTORY_WRITE_NAME%
echo pause>> %FW_BIN_DIR%\%FW_FACTORY_WRITE_NAME%
echo )>> %FW_BIN_DIR%\%FW_FACTORY_WRITE_NAME%
