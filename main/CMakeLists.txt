idf_component_register(
    SRCS "main.c" "bt_ag.c" "audio_io.c" "pcm_ring.c" "tone.c" "ptt.c" "console.c" "settings.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_timer esp_driver_gpio esp_driver_i2c esp_driver_i2s esp_driver_uart
             esp_codec_dev
)
