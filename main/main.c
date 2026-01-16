#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "esp_lcd_panel_ops.h"

#include "esp_lcd_touch.h"

#include "lvgl.h"

#include "cyd_config.h"
#include "cyd_hw.h"
#include "ui.h"

static const char *TAG = "cyd_lvgl";

static lv_display_t *s_disp;
static esp_lcd_touch_handle_t s_touch;
static esp_lcd_panel_handle_t s_panel;

static bool map_touch_coords(uint16_t *x, uint16_t *y)
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

/* 1ms LVGL tick */
static void lv_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(1);
}

/* SPI transfer done -> pranešam LVGL, kad flush baigtas */
static bool on_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                               esp_lcd_panel_io_event_data_t *edata,
                               void *user_ctx)
{
    (void)panel_io; (void)edata;
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

/* LVGL flush -> piešiam į panelę */
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    if (LCD_SWAP_COLOR_BYTES || LCD_SWAP_RB) {
        uint16_t *buf = (uint16_t *)px_map;
        size_t w = (size_t)(area->x2 - area->x1 + 1);
        size_t h = (size_t)(area->y2 - area->y1 + 1);
        size_t count = w * h;
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
    esp_lcd_panel_draw_bitmap(panel,
                              area->x1, area->y1,
                              area->x2 + 1, area->y2 + 1,
                              px_map);
    /* flush_ready iškviesim per on_color_trans_done callback */
}

/* Touch read (kol kas paliekam deprecated – tai tik WARNING) */
static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;

    if (!s_touch) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    uint16_t x[1], y[1];
    uint8_t cnt = 0;
    static uint16_t last_x;
    static uint16_t last_y;
    static bool last_pressed;

    /* kai kurie touch driveriai reikalauja read_data() prieš get_coordinates() */
    esp_lcd_touch_read_data(s_touch);

    bool pressed = esp_lcd_touch_get_coordinates(s_touch, x, y, NULL, &cnt, 1);

    if (pressed && cnt > 0) {
        map_touch_coords(&x[0], &y[0]);

        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = x[0];
        data->point.y = y[0];

        if (pressed != last_pressed || x[0] != last_x || y[0] != last_y) {
            lv_obj_t *cursor = ui_get_cursor();
            if (cursor) {
                lv_obj_set_pos(cursor, x[0] - 6, y[0] - 6);
            }
            lv_obj_t *label = ui_get_touch_label();
            if (label) {
                char buf[64];
                snprintf(buf, sizeof(buf), "touch: %u, %u", x[0], y[0]);
                lv_label_set_text(label, buf);
            }
            last_x = x[0];
            last_y = y[0];
            last_pressed = pressed;
        }
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
        last_pressed = false;
    }
}

void app_main(void)
{
    cyd_hw_init_backlight();

    esp_lcd_panel_io_handle_t lcd_io = NULL;
    s_panel = cyd_hw_init_lcd(&lcd_io);

    /* LVGL init */
    lv_init();

    /* Tick timer 1ms */
    const esp_timer_create_args_t tick_args = {
        .callback = lv_tick_cb,
        .name = "lv_tick",
    };
    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, 1000));

    /* LVGL display */
    s_disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_user_data(s_disp, s_panel);
    lv_display_set_flush_cb(s_disp, lvgl_flush_cb);

    static lv_color_t buf1[LCD_H_RES * 40];
    static lv_color_t buf2[LCD_H_RES * 40];
    lv_display_set_buffers(s_disp, buf1, buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);

    /* Register "flush done" callback in LCD IO */
    esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = on_color_trans_done,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(lcd_io, &cbs, s_disp));

    s_touch = cyd_hw_init_touch();

    /* LVGL input */
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(indev, s_disp);
    lv_indev_set_read_cb(indev, lvgl_touch_read_cb);

    ui_init();

    ESP_LOGI(TAG, "LVGL running");

    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
