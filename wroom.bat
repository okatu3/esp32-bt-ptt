@echo off
rem ESP32-WROOM version (internal ADC/DAC). Separate build folder and sdkconfig.
rem Usage:  wroom.bat build
rem         wroom.bat -p COM5 flash monitor
call "%~dp0idf_env.bat" -B build_wroom -D SDKCONFIG=sdkconfig.wroom -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.defaults.wroom" %*
