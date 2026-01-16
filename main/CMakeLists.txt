idf_component_register(
    SRCS "main.c" "cyd_hw.c" "ui.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer driver esp_lcd lvgl
)
