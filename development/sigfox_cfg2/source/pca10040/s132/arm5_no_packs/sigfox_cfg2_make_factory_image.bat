@echo off
set CURPATH=%cd%
cd ..\..\..\..\source_bootloader_secure\pca10040\arm5_no_packs
call sigfox_cfg2_bootloader_binary_copy.bat ..\..\hex\bootloader.hex
cd %CURPATH%
