#pragma once

/* Main configuration file - includes all CYD-specific settings */
#include "cyd_pins.h"
#include "cyd_display_config.h"
#include "cyd_calibration.h"

/* UI Configuration */
#define CURSOR_OFFSET 6  /* Cursor centering offset in pixels */
#define TOUCH_LABEL_MAX_LEN 64  /* Maximum length for touch coordinate label */
#define LVGL_TASK_DELAY_MS 10  /* LVGL task handler delay */
#define LVGL_TICK_PERIOD_MS 1  /* LVGL tick timer period */
