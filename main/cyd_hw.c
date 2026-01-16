#include "cyd_hw.h"
#include "cyd_config.h"
#include "cyd_pins.h"
#include "cyd_display_config.h"
#include "cyd_calibration.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_lcd_ili9341.h"
#include "esp_lcd_touch_xpt2046.h"

static const char *TAG = "cyd_hw";

esp_err_t cyd_hw_init_backlight(void)
{
    gpio_config_t bk = {
        .pin_bit_mask = 1ULL << CYD_PIN_NUM_BCKL,
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t ret = gpio_config(&bk);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure backlight GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = gpio_set_level(CYD_PIN_NUM_BCKL, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set backlight level: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Backlight initialized");
    return ESP_OK;
}

esp_err_t cyd_hw_init_lcd(esp_lcd_panel_io_handle_t *out_lcd_io, esp_lcd_panel_handle_t *out_panel)
{
    if (out_panel == NULL) {
        ESP_LOGE(TAG, "out_panel cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    spi_bus_config_t buscfg = {
        .mosi_io_num = CYD_PIN_NUM_MOSI,
        .miso_io_num = CYD_PIN_NUM_MISO,
        .sclk_io_num = CYD_PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_BUFFER_LINES * 2,
    };
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

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
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &lcd_io_cfg, &lcd_io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LCD IO: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = CYD_PIN_NUM_LCD_RST,
        .color_space = LCD_COLOR_SPACE,
        .bits_per_pixel = 16,
    };

    ret = esp_lcd_new_panel_ili9341(lcd_io, &panel_cfg, &panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ILI9341 panel: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = esp_lcd_panel_reset(panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset panel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_lcd_panel_init(panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init panel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_lcd_panel_invert_color(panel, LCD_INVERT_COLOR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to invert color: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_lcd_panel_disp_on_off(panel, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to turn on display: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_lcd_panel_mirror(panel, LCD_MIRROR_X, LCD_MIRROR_Y);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mirror panel: %s", esp_err_to_name(ret));
        return ret;
    }

    if (out_lcd_io) {
        *out_lcd_io = lcd_io;
    }
    *out_panel = panel;

    ESP_LOGI(TAG, "LCD initialized (%dx%d)", LCD_H_RES, LCD_V_RES);
    return ESP_OK;
}

esp_err_t cyd_hw_init_touch(esp_lcd_touch_handle_t *out_touch)
{
    if (out_touch == NULL) {
        ESP_LOGE(TAG, "out_touch cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    spi_bus_config_t touch_buscfg = {
        .mosi_io_num = CYD_PIN_NUM_TCH_MOSI,
        .miso_io_num = CYD_PIN_NUM_TCH_MISO,
        .sclk_io_num = CYD_PIN_NUM_TCH_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &touch_buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize touch SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_lcd_panel_io_handle_t touch_io = NULL;
    esp_lcd_panel_io_spi_config_t touch_io_cfg = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(CYD_PIN_NUM_TCH_CS);
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &touch_io_cfg, &touch_io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create touch IO: %s", esp_err_to_name(ret));
        return ret;
    }

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
    ret = esp_lcd_touch_new_spi_xpt2046(touch_io, &touch_cfg, &touch);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create XPT2046 touch: %s", esp_err_to_name(ret));
        return ret;
    }
    
    *out_touch = touch;
    ESP_LOGI(TAG, "Touch initialized");
    return ESP_OK;
}

bool cyd_hw_map_touch_coords(uint16_t *x, uint16_t *y)
{
    if (TOUCH_RAW_X_MAX <= TOUCH_RAW_X_MIN || TOUCH_RAW_Y_MAX <= TOUCH_RAW_Y_MIN) {
        return false;
    }

    int raw_x = *x;
    int raw_y = *y;

    if (raw_x < TOUCH_RAW_X_MIN) raw_x = TOUCH_RAW_X_MIN;
    if (raw_x > TOUCH_RAW_X_MAX) raw_x = TOUCH_RAW_X_MAX;
    if (raw_y < TOUCH_RAW_Y_MIN) raw_y = TOUCH_RAW_Y_MIN;
    if (raw_y > TOUCH_RAW_Y_MAX) raw_y = TOUCH_RAW_Y_MAX;

    *x = (uint16_t)(((raw_x - TOUCH_RAW_X_MIN) * (LCD_H_RES - 1)) /
                    (TOUCH_RAW_X_MAX - TOUCH_RAW_X_MIN));
    *y = (uint16_t)(((raw_y - TOUCH_RAW_Y_MIN) * (LCD_V_RES - 1)) /
                    (TOUCH_RAW_Y_MAX - TOUCH_RAW_Y_MIN));
    return true;
}

void cyd_hw_correct_color_buffer(uint16_t *buf, size_t count)
{
    if (!buf || count == 0) {
        return;
    }
    
    for (size_t i = 0; i < count; i++) {
        uint16_t v = buf[i];
        
        if (LCD_SWAP_COLOR_BYTES) {
            v = (uint16_t)((v >> 8) | (v << 8));
        }
        
        if (LCD_SWAP_RB) {
            uint16_t r = (uint16_t)((v >> 11) & 0x1F);
            uint16_t g = (uint16_t)((v >> 5) & 0x3F);
            uint16_t b = (uint16_t)(v & 0x1F);
            v = (uint16_t)((b << 11) | (g << 5) | r);
        }
        
        buf[i] = v;
    }
}
