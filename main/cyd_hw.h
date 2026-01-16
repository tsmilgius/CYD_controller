#pragma once

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/**
 * @brief Initialize backlight GPIO
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t cyd_hw_init_backlight(void);

/**
 * @brief Initialize LCD display
 * 
 * @param[out] out_lcd_io Optional pointer to receive LCD IO handle
 * @param[out] out_panel Pointer to receive LCD panel handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t cyd_hw_init_lcd(esp_lcd_panel_io_handle_t *out_lcd_io, esp_lcd_panel_handle_t *out_panel);

/**
 * @brief Initialize touch controller
 * 
 * @param[out] out_touch Pointer to receive touch handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t cyd_hw_init_touch(esp_lcd_touch_handle_t *out_touch);

/**
 * @brief Map raw touch coordinates to screen coordinates
 * 
 * @param[in,out] x Pointer to X coordinate (raw in, mapped out)
 * @param[in,out] y Pointer to Y coordinate (raw in, mapped out)
 * @return true if mapping was successful, false if calibration is invalid
 */
bool cyd_hw_map_touch_coords(uint16_t *x, uint16_t *y);

/**
 * @brief Apply color correction to a pixel buffer
 * 
 * Applies byte swapping and R/B channel swapping as configured
 * 
 * @param[in,out] buf Pointer to 16-bit pixel buffer
 * @param[in] count Number of pixels in buffer
 */
void cyd_hw_correct_color_buffer(uint16_t *buf, size_t count);
