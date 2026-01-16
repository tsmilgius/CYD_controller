#pragma once

/* ======= CYD 2.8 (ILI9341 + XPT2046) PINOUT ======= */

/* LCD SPI pins */
#define CYD_PIN_NUM_MOSI     13
#define CYD_PIN_NUM_MISO     12
#define CYD_PIN_NUM_SCLK     14

/* LCD control pins */
#define CYD_PIN_NUM_LCD_CS   15
#define CYD_PIN_NUM_LCD_DC    2
#define CYD_PIN_NUM_LCD_RST   4
#define CYD_PIN_NUM_BCKL     21

/* Touch SPI pins (separate bus) */
#define CYD_PIN_NUM_TCH_MOSI 32
#define CYD_PIN_NUM_TCH_MISO 39
#define CYD_PIN_NUM_TCH_SCLK 25
#define CYD_PIN_NUM_TCH_CS   33
#define CYD_PIN_NUM_TCH_IRQ  36
