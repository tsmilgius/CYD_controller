#include <stdio.h>
#include <string.h>
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

/* LVGL tick callback - periodic timer for LVGL timekeeping */
static void lv_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

/* SPI transfer done callback - notify LVGL that flush is complete */
static bool on_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                               esp_lcd_panel_io_event_data_t *edata,
                               void *user_ctx)
{
    (void)panel_io; (void)edata;
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

/* LVGL flush callback - draw pixels to display with color correction */
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    
    /* Apply color correction if needed */
    if (LCD_SWAP_COLOR_BYTES || LCD_SWAP_RB) {
        uint16_t *buf = (uint16_t *)px_map;
        size_t w = (size_t)(area->x2 - area->x1 + 1);
        size_t h = (size_t)(area->y2 - area->y1 + 1);
        size_t count = w * h;
        cyd_hw_correct_color_buffer(buf, count);
    }
    
    esp_lcd_panel_draw_bitmap(panel,
                              area->x1, area->y1,
                              area->x2 + 1, area->y2 + 1,
                              px_map);
    /* flush_ready will be called via on_color_trans_done callback */
}

/* LVGL touch read callback - handles touch input and updates UI debug elements */
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

    /* Some touch drivers require read_data() before get_coordinates() */
    esp_lcd_touch_read_data(s_touch);

    bool pressed = esp_lcd_touch_get_coordinates(s_touch, x, y, NULL, &cnt, 1);

    if (pressed && cnt > 0) {
        cyd_hw_map_touch_coords(&x[0], &y[0]);

        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = x[0];
        data->point.y = y[0];

        if (pressed != last_pressed || x[0] != last_x || y[0] != last_y) {
            lv_obj_t *cursor = ui_get_cursor();
            if (cursor) {
                lv_obj_set_pos(cursor, x[0] - CURSOR_OFFSET, y[0] - CURSOR_OFFSET);
            }
            lv_obj_t *label = ui_get_touch_label();
            if (label) {
                char buf[TOUCH_LABEL_MAX_LEN];
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
    esp_err_t ret;
    
    /* Initialize backlight */
    ret = cyd_hw_init_backlight();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize backlight");
        return;
    }

    /* Initialize LCD */
    esp_lcd_panel_io_handle_t lcd_io = NULL;
    ret = cyd_hw_init_lcd(&lcd_io, &s_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LCD");
        return;
    }

    /* Initialize LVGL */
    lv_init();

    /* Create LVGL tick timer (1ms) */
    const esp_timer_create_args_t tick_args = {
        .callback = lv_tick_cb,
        .name = "lv_tick",
    };
    esp_timer_handle_t tick_timer;
    ret = esp_timer_create(&tick_args, &tick_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LVGL timer");
        return;
    }
    ret = esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start LVGL timer");
        return;
    }

    /* Create LVGL display */
    s_disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    if (!s_disp) {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        return;
    }
    lv_display_set_user_data(s_disp, s_panel);
    lv_display_set_flush_cb(s_disp, lvgl_flush_cb);

    /* Allocate LVGL draw buffers */
    static lv_color_t buf1[LCD_H_RES * LCD_BUFFER_LINES];
    static lv_color_t buf2[LCD_H_RES * LCD_BUFFER_LINES];
    lv_display_set_buffers(s_disp, buf1, buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);

    /* Register flush done callback in LCD IO */
    esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = on_color_trans_done,
    };
    ret = esp_lcd_panel_io_register_event_callbacks(lcd_io, &cbs, s_disp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register LCD callbacks");
        return;
    }

    /* Initialize touch controller */
    ret = cyd_hw_init_touch(&s_touch);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize touch, continuing without touch input");
        s_touch = NULL;
    }

    /* Create LVGL input device */
    lv_indev_t *indev = lv_indev_create();
    if (!indev) {
        ESP_LOGE(TAG, "Failed to create LVGL input device");
        return;
    }
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(indev, s_disp);
    lv_indev_set_read_cb(indev, lvgl_touch_read_cb);

    /* Initialize UI */
    ui_init();

    ESP_LOGI(TAG, "LVGL running");

    /* Main LVGL task loop */
    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(LVGL_TASK_DELAY_MS));
    }
}
