#pragma once

#include "esp_lcd_panel_vendor.h"

/* ======= CYD 2.8 (ILI9341 + XPT2046) PINOUT ======= */
#define CYD_PIN_NUM_MOSI     13
#define CYD_PIN_NUM_MISO     12
#define CYD_PIN_NUM_SCLK     14

#define CYD_PIN_NUM_LCD_CS   15
#define CYD_PIN_NUM_LCD_DC    2
#define CYD_PIN_NUM_LCD_RST   4
#define CYD_PIN_NUM_BCKL     21

/* Touch uses separate SPI pins */
#define CYD_PIN_NUM_TCH_MOSI 32
#define CYD_PIN_NUM_TCH_MISO 39
#define CYD_PIN_NUM_TCH_SCLK 25
#define CYD_PIN_NUM_TCH_CS   33
#define CYD_PIN_NUM_TCH_IRQ  36

#define LCD_H_RES 320
#define LCD_V_RES 240

/* Raw touch bounds observed on this panel */
#define TOUCH_RAW_X_MIN 21
#define TOUCH_RAW_X_MAX 220
#define TOUCH_RAW_Y_MIN 23
#define TOUCH_RAW_Y_MAX 289

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
