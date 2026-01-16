#pragma once

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"

void cyd_hw_init_backlight(void);
esp_lcd_panel_handle_t cyd_hw_init_lcd(esp_lcd_panel_io_handle_t *out_lcd_io);
esp_lcd_touch_handle_t cyd_hw_init_touch(void);
