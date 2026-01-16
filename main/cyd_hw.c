#include "cyd_hw.h"
#include "cyd_config.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#include "esp_lcd_ili9341.h"
#include "esp_lcd_touch_xpt2046.h"

void cyd_hw_init_backlight(void)
{
    gpio_config_t bk = {
        .pin_bit_mask = 1ULL << CYD_PIN_NUM_BCKL,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&bk));
    gpio_set_level(CYD_PIN_NUM_BCKL, 1);
}

esp_lcd_panel_handle_t cyd_hw_init_lcd(esp_lcd_panel_io_handle_t *out_lcd_io)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = CYD_PIN_NUM_MOSI,
        .miso_io_num = CYD_PIN_NUM_MISO,
        .sclk_io_num = CYD_PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 40 * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t lcd_io = NULL;
    esp_lcd_panel_io_spi_config_t lcd_io_cfg = {
        .dc_gpio_num = CYD_PIN_NUM_LCD_DC,
        .cs_gpio_num = CYD_PIN_NUM_LCD_CS,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &lcd_io_cfg, &lcd_io));

    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = CYD_PIN_NUM_LCD_RST,
        .color_space = LCD_COLOR_SPACE,
        .bits_per_pixel = 16,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(lcd_io, &panel_cfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, LCD_INVERT_COLOR));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, LCD_MIRROR_X, LCD_MIRROR_Y));

    if (out_lcd_io) {
        *out_lcd_io = lcd_io;
    }

    return panel;
}

esp_lcd_touch_handle_t cyd_hw_init_touch(void)
{
    spi_bus_config_t touch_buscfg = {
        .mosi_io_num = CYD_PIN_NUM_TCH_MOSI,
        .miso_io_num = CYD_PIN_NUM_TCH_MISO,
        .sclk_io_num = CYD_PIN_NUM_TCH_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &touch_buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t touch_io = NULL;
    esp_lcd_panel_io_spi_config_t touch_io_cfg = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(CYD_PIN_NUM_TCH_CS);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &touch_io_cfg, &touch_io));

    esp_lcd_touch_config_t touch_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = {.reset = 0, .interrupt = 0},
        .flags = {
            .swap_xy = TOUCH_SWAP_XY,
            .mirror_x = TOUCH_MIRROR_X,
            .mirror_y = TOUCH_MIRROR_Y,
        },
        .process_coordinates = NULL,
    };

    esp_lcd_touch_handle_t touch = NULL;
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(touch_io, &touch_cfg, &touch));
    return touch;
}
