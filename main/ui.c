#include "ui.h"
#include "cyd_config.h"

static lv_obj_t *s_touch_label;
static lv_obj_t *s_cursor;

void ui_init(void)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_make(10, 0, 120), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    s_touch_label = lv_label_create(screen);
    lv_obj_align(s_touch_label, LV_ALIGN_TOP_MID, 0, 10);
    lv_label_set_text(s_touch_label, "touch: -");
    lv_obj_set_style_text_color(s_touch_label, lv_color_white(), 0);

    s_cursor = lv_obj_create(screen);
    lv_obj_set_size(s_cursor, 12, 12);
    lv_obj_set_style_radius(s_cursor, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(s_cursor, LV_OPA_COVER, 0);

    lv_obj_t *label = lv_label_create(screen);
    lv_label_set_text(label, "Labas, LVGL 9.4 ant CYD!");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
}

lv_obj_t *ui_get_touch_label(void)
{
    return s_touch_label;
}

lv_obj_t *ui_get_cursor(void)
{
    return s_cursor;
}
