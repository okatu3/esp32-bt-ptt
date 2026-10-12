cmake_minimum_required(VERSION 3.16)

# ESP-ADF: only the codec driver (esp_codec_dev / ES8388) is taken from ADF.
if(DEFINED ENV{ADF_PATH})
    set(ADF_PATH $ENV{ADF_PATH})
else()
    set(ADF_PATH "C:/Users/user/esp-adf")
endif()
set(EXTRA_COMPONENT_DIRS "${ADF_PATH}/components/esp_codec_dev")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bt_ptt)
