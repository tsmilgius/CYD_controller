#pragma once

#include "esp_lcd_panel_vendor.h"

/* Display resolution */
#define LCD_H_RES 320
#define LCD_V_RES 240

/* LVGL buffer configuration */
#define LCD_BUFFER_LINES 40  /* Number of lines per LVGL buffer */

/* Panel color settings */
#define LCD_COLOR_SPACE ESP_LCD_COLOR_SPACE_RGB
#define LCD_INVERT_COLOR 0
#define LCD_MIRROR_X 1
#define LCD_MIRROR_Y 0

/* Touch orientation flags */
#define TOUCH_SWAP_XY 1
#define TOUCH_MIRROR_X 1
#define TOUCH_MIRROR_Y 1

/* Pixel format corrections */
#define LCD_SWAP_COLOR_BYTES 1
#define LCD_SWAP_RB 0
