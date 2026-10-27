set(srcs "main.c" "bt_ag.c" "audio_io.c" "pcm_ring.c" "tone.c" "ptt.c" "console.c" "settings.c")
if(CONFIG_BTPTT_BOARD_WROOM)
    list(APPEND srcs "audio_hw_internal.c")
else()
    list(APPEND srcs "audio_hw_es8388.c")
endif()

# Requirements cannot depend on sdkconfig, so both backends' are listed.
idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_timer esp_driver_gpio esp_driver_uart
             esp_driver_i2c esp_driver_i2s esp_codec_dev
             esp_adc esp_driver_dac esp_driver_gptimer
)
