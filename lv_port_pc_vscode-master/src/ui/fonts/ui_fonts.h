#ifndef UI_FONTS_H
#define UI_FONTS_H

#include "lvgl/lvgl.h"

extern const lv_font_t ui_font_SC_20;
extern const lv_font_t ui_font_SC_27;
extern const lv_font_t ui_font_SC_30;
extern const lv_font_t ui_font_SC_35;
extern const lv_font_t ui_font_SC_50;
extern const lv_font_t ui_font_SC_125;

const lv_font_t * ui_font_get_sc_20(void);
const lv_font_t * ui_font_get_sc_27(void);
const lv_font_t * ui_font_get_sc_30(void);
const lv_font_t * ui_font_get_sc_35(void);
const lv_font_t * ui_font_get_sc_50(void);
const lv_font_t * ui_font_get_sc_125(void);

#endif
