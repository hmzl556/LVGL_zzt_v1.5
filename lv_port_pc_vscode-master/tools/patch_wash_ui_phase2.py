#!/usr/bin/env python3
"""Patch ui-商用洗.c phase 2: payment, globals, selfcheck/cycle, vendor."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WASH = ROOT / "src/ui/ui-商用洗.c"
DRY = ROOT / "src/ui/ui-烘干硬件.c"
HELPERS = ROOT / "tools/wash_cycle_helpers.c.inc"
EXTRACTED = ROOT / "tools/extracted_cycle_selfcheck.c.inc"

def replace_once(text, old, new, label):
    if old not in text:
        raise ValueError(f"Patch failed [{label}]")
    return text.replace(old, new, 1)

dry = DRY.read_text(encoding="utf-8")
wash = WASH.read_text(encoding="utf-8")
helpers = HELPERS.read_text(encoding="utf-8")
extracted = EXTRACTED.read_text(encoding="utf-8")

# vendor callbacks from drying
vendor_block = """
static void cb_admin_open_vendor_maint(lv_event_t * e)
{
    (void)e;
    admin_panel_show(VENDOR_SERIAL);
}

static void admin_vendor_serial_back_to_menu1(void)
{
    if(g_admin_ta_vendor_serial != NULL) {
        lv_textarea_set_text(g_admin_ta_vendor_serial, "");
    }
    admin_panel_show(MENU1);
}

static void admin_vendor_menu_back_to_menu1(void)
{
    admin_panel_show(MENU1);
}

static void admin_vendor_serial_try(void)
{
    if(g_admin_ta_vendor_serial == NULL) return;
    const char * t = lv_textarea_get_text(g_admin_ta_vendor_serial);
    if(t == NULL || lv_strlen(t) != 6) return;
    if(lv_strcmp(t, VENDOR_SERIAL_CODE) != 0) {
        lv_textarea_set_text(g_admin_ta_vendor_serial, "");
        return;
    }
    lv_textarea_set_text(g_admin_ta_vendor_serial, "");
    admin_panel_show(VENDOR_MENU);
}

static void cb_admin_open_selfcheck(lv_event_t * e)
{
    (void)e;
    ui_screen_load(g_scr_selfcheck);
}

static void cb_admin_open_cycle(lv_event_t * e)
{
    (void)e;
    selfcheck_timer_stop_all();
    cycle_ui_reset();
    for(int i = 0; i < TOTAL_PROGRAMS; i++) {
        g_cycle_cfg[i] = g_prog_cfg[i];
    }
    g_cycle_active = true;
    g_cycle_ui_state = CYCLE_UI_SETUP;
    g_cycle_prog_sel = 0;
    cycle_wash_sync_prog_pick_ui();
    cycle_run_count_label_sync();
    cycle_encoder_group_build();
    ui_screen_load(g_scr_cycle);
}

"""

# pay_sync_pay_ui from drying
pay_sync = """
static void pay_sync_pay_ui(void)
{
	if(g_pay_qr_img == NULL) return;
	bool alipay = ui_payment_alipay_get();
	bool wechat = ui_payment_wechat_get();
	if(!alipay && !wechat) alipay = true;
	int32_t idx = wheel_mod_total(g_wheel_sel);

	if(alipay && wechat) {
		if(g_pay_dual_row != NULL) lv_obj_remove_flag(g_pay_dual_row, LV_OBJ_FLAG_HIDDEN);
		if(g_pay_single_col != NULL) lv_obj_add_flag(g_pay_single_col, LV_OBJ_FLAG_HIDDEN);
		if(g_pay_qr_alipay != NULL) lv_image_set_src(g_pay_qr_alipay, g_alipay_qr_imgs[(unsigned)idx]);
		if(g_pay_qr_wechat != NULL) lv_image_set_src(g_pay_qr_wechat, g_wechat_qr_imgs[(unsigned)idx]);
	}
	else {
		if(g_pay_dual_row != NULL) lv_obj_add_flag(g_pay_dual_row, LV_OBJ_FLAG_HIDDEN);
		if(g_pay_single_col != NULL) lv_obj_remove_flag(g_pay_single_col, LV_OBJ_FLAG_HIDDEN);
		lv_image_set_src(g_pay_qr_img, alipay ? g_alipay_qr_imgs[(unsigned)idx] : g_wechat_qr_imgs[(unsigned)idx]);
		if(g_lbl_pay_hint != NULL) {
			lv_label_set_text(g_lbl_pay_hint, ui_translation(alipay ? STR_PAY_HINT_ALIPAY : STR_PAY_HINT_WECHAT));
		}
	}
}

"""

# build_pay from drying (lines 12103-12230)
i = dry.find("static void build_pay(void)")
j = dry.find("static void build_pay_done(void)", i)
build_pay = dry[i:j]

# payment checkbox callbacks
i2 = dry.find("static void cb_admin_payment_alipay_changed")
j2 = dry.find("static void admin_payment_back_to_menu2", i2)
pay_cb = dry[i2:j2]

# ui_admin_resume_unlocked from drying
ui_admin_resume = """
static void ui_admin_resume_unlocked(void)
{
	if(g_scr_admin == NULL) return;
	lv_screen_load(g_scr_admin);
	ui_set_encoder_group(g_group_admin);
	ui_idle_on_screen_changed(g_scr_admin);
}

"""

# --- globals: groups ---
wash = replace_once(wash,
    "static lv_group_t* g_group_admin;",
    "static lv_group_t* g_group_admin;\nstatic lv_group_t* g_group_selfcheck;\nstatic lv_group_t* g_group_cycle;",
    "groups")

# --- globals after menu2 btns ---
wash = replace_once(wash,
    "static lv_obj_t * g_admin_menu2_btns[8];\nstatic lv_obj_t * s_admin_group_prev_focus;",
    """static lv_obj_t * g_admin_menu2_btns[8];
static lv_obj_t * g_admin_panel_vendor_serial;
static lv_obj_t * g_admin_panel_vendor_menu;
static lv_obj_t * g_admin_ta_vendor_serial;
static lv_obj_t * g_admin_lbl_vendor_serial_title;
static lv_obj_t * g_admin_btn_vendor_self_check;
static lv_obj_t * g_admin_btn_vendor_self_learn;

static lv_obj_t * g_scr_selfcheck;
static lv_obj_t * g_lbl_clock_selfcheck;
static lv_obj_t * g_selfcheck_btn_back;
static lv_obj_t * g_selfcheck_btn_runpause;
static lv_obj_t * g_selfcheck_btn_power;
static lv_obj_t * g_selfcheck_panel_model;
static lv_obj_t * g_selfcheck_panel_init;
static lv_obj_t * g_selfcheck_panel_steps;
static lv_obj_t * g_selfcheck_panel_end;
static lv_obj_t * g_selfcheck_lbl_model;
static lv_obj_t * g_selfcheck_lbl_init_dry_time;
static lv_obj_t * g_selfcheck_lbl_init_temp;
static lv_obj_t * g_selfcheck_lbl_init_add_count;
static lv_obj_t * g_selfcheck_step_btns[SELFCHECK_STEP_COUNT];
static lv_obj_t * g_selfcheck_lbl_done_title;
static lv_obj_t * g_selfcheck_row_status;
static lv_obj_t * g_selfcheck_lbl_doorlock;
static lv_obj_t * g_selfcheck_lbl_temp_live;
static lv_obj_t * g_selfcheck_row_indicator;
static lv_obj_t * g_selfcheck_lbl_dry;
static lv_obj_t * g_selfcheck_lbl_cooling;
static selfcheck_ui_state_t g_selfcheck_ui_state = SELFCHECK_UI_MODEL;
static lv_timer_t * g_selfcheck_timer;
static lv_timer_t * g_selfcheck_blink_timer;
static lv_obj_t * g_selfcheck_blink_target;
static uint8_t g_selfcheck_flash_count;
static bool g_selfcheck_flash_on;
static bool g_selfcheck_blink_on;
static int16_t g_selfcheck_live_temp_c = SELFCHECK_LIVE_TEMP_DEFAULT_C;

static lv_obj_t * g_scr_cycle;
static lv_obj_t * g_lbl_clock_cycle;
static lv_obj_t * g_cycle_btn_back;
static lv_obj_t * g_cycle_btn_runpause;
static lv_obj_t * g_cycle_btn_power;
static lv_obj_t * g_cycle_panel_program;
static lv_obj_t * g_cycle_panel_fault;
static lv_obj_t * g_cycle_lbl_fault_alt;
static lv_obj_t * g_cycle_panel_fault_run;
static lv_obj_t * g_cycle_lbl_fault_alt_run;

static lv_obj_t * s_admin_group_prev_focus;""",
    "vendor/selfcheck globals")

# --- payment UI globals ---
wash = replace_once(wash,
    "static lv_obj_t * g_pay_qr_img;",
    """#define PAY_DUAL_COL_GAP   140
#define PAY_DUAL_HINT_W    450
static lv_obj_t * g_lbl_pay_price_alipay;
static lv_obj_t * g_lbl_pay_price_wechat;
static lv_obj_t * g_lbl_pay_hint_alipay_l1;
static lv_obj_t * g_lbl_pay_hint_alipay_l2;
static lv_obj_t * g_lbl_pay_hint_wechat_l1;
static lv_obj_t * g_lbl_pay_hint_wechat_l2;
static lv_obj_t * g_pay_single_col;
static lv_obj_t * g_pay_dual_row;
static lv_obj_t * g_pay_qr_img;
static lv_obj_t * g_pay_qr_alipay;
static lv_obj_t * g_pay_qr_wechat;""",
    "pay globals")

# --- QR images ---
wash = replace_once(wash,
    """LV_IMAGE_DECLARE(QR_01_dawu);
LV_IMAGE_DECLARE(QR_02_dantuoshui);
LV_IMAGE_DECLARE(QR_03_biaozhunxi);
LV_IMAGE_DECLARE(QR_04_tongzijie);
LV_IMAGE_DECLARE(QR_05_kuaixi);
LV_IMAGE_DECLARE(QR_code_xiaoya);""",
    """LV_IMAGE_DECLARE(QR_Alipay_01_dawu);
LV_IMAGE_DECLARE(QR_Alipay_02_dantuoshui);
LV_IMAGE_DECLARE(QR_Alipay_03_biaozhunxi);
LV_IMAGE_DECLARE(QR_Alipay_04_tongzijie);
LV_IMAGE_DECLARE(QR_Alipay_05_kuaixi);
LV_IMAGE_DECLARE(QR_WeChat_01_dawu);
LV_IMAGE_DECLARE(QR_WeChat_02_dantuoshui);
LV_IMAGE_DECLARE(QR_WeChat_03_biaozhunxi);
LV_IMAGE_DECLARE(QR_WeChat_04_tongzijie);
LV_IMAGE_DECLARE(QR_WeChat_05_kuaixi);
LV_IMAGE_DECLARE(QR_code_xiaoya);""",
    "qr declares")

wash = replace_once(wash,
    """static const lv_image_dsc_t * const g_program_qr_imgs[TOTAL_PROGRAMS] = {
    &QR_01_dawu, &QR_02_dantuoshui, &QR_03_biaozhunxi, &QR_04_tongzijie, &QR_05_kuaixi
};""",
    """static const lv_image_dsc_t * const g_alipay_qr_imgs[TOTAL_PROGRAMS] = {
    &QR_Alipay_01_dawu, &QR_Alipay_02_dantuoshui, &QR_Alipay_03_biaozhunxi,
    &QR_Alipay_04_tongzijie, &QR_Alipay_05_kuaixi
};
static const lv_image_dsc_t * const g_wechat_qr_imgs[TOTAL_PROGRAMS] = {
    &QR_WeChat_01_dawu, &QR_WeChat_02_dantuoshui, &QR_WeChat_03_biaozhunxi,
    &QR_WeChat_04_tongzijie, &QR_WeChat_05_kuaixi
};""",
    "qr arrays")

# --- forward decls ---
wash = replace_once(wash,
    "static void pay_sync_price_label(void);  //按当前选中程序刷新支付页金额标签\nstatic void pay_sync_qr_image(void);  //按当前选中程序刷新支付页二维码",
    "static void pay_sync_price_label(void);\nstatic void pay_sync_pay_ui(void);\nstatic void cb_admin_payment_alipay_changed(lv_event_t * e);\nstatic void cb_admin_payment_wechat_changed(lv_event_t * e);\nstatic void cb_admin_open_vendor_maint(lv_event_t * e);\nstatic void cb_admin_open_selfcheck(lv_event_t * e);\nstatic void cb_admin_open_cycle(lv_event_t * e);\nstatic void build_selfcheck(void);\nstatic void build_cycle(void);\nstatic void cycle_ui_reset(void);\nstatic void cycle_wash_sync_prog_pick_ui(void);\nstatic void cycle_finish_current_run(void);\nstatic void cycle_enter_fault_display(void);\nstatic void cycle_abort_run(void);",
    "forward decls")

# --- pay_sync_price_label: add alipay/wechat labels ---
wash = replace_once(wash,
    "\tlv_label_set_text(g_lbl_pay_price, buf);             /* 写入金额文字 */\n}",
    "\tlv_label_set_text(g_lbl_pay_price, buf);\n\tif(g_lbl_pay_price_alipay != NULL) lv_label_set_text(g_lbl_pay_price_alipay, buf);\n\tif(g_lbl_pay_price_wechat != NULL) lv_label_set_text(g_lbl_pay_price_wechat, buf);\n}",
    "pay price sync")

# --- replace pay_sync_qr_image with pay_sync_pay_ui ---
wash = replace_once(wash,
    """static void pay_sync_qr_image(void)
{
	if(g_pay_qr_img == NULL) return;
	int32_t idx = wheel_mod_total(g_wheel_sel);
	lv_image_set_src(g_pay_qr_img, g_program_qr_imgs[(unsigned)idx]);
}
""",
    pay_sync,
    "pay_sync_pay_ui")

# --- ui_screen_load pay ---
wash = replace_once(wash,
    "\t\tpay_sync_qr_image();                             /* 刷新二维码图（非 label） */",
    "\t\tpay_sync_pay_ui();",
    "screen load pay")

# --- create_screens ---
wash = replace_once(wash,
    "\tg_scr_admin = lv_obj_create(NULL);",
    "\tg_scr_admin = lv_obj_create(NULL);\n\tg_scr_selfcheck = lv_obj_create(NULL);\n\tg_scr_cycle = lv_obj_create(NULL);",
    "create screens obj")

wash = replace_once(wash,
    "\tstyle_screen_base(g_scr_admin);                //管理员",
    "\tstyle_screen_base(g_scr_admin);\n\tstyle_screen_base(g_scr_selfcheck);\n\tstyle_screen_base(g_scr_cycle);",
    "style screens")

wash = replace_once(wash,
    "\tlv_obj_set_size(g_scr_admin, UI_FIXED_W, UI_FIXED_H);",
    "\tlv_obj_set_size(g_scr_admin, UI_FIXED_W, UI_FIXED_H);\n\tlv_obj_set_size(g_scr_selfcheck, UI_FIXED_W, UI_FIXED_H);\n\tlv_obj_set_size(g_scr_cycle, UI_FIXED_W, UI_FIXED_H);",
    "size screens")

# --- ui_screen_load selfcheck/cycle ---
wash = replace_once(wash,
    "\telse if(scr == g_scr_admin) {",
    """\telse if(scr == g_scr_selfcheck) {
\t\tui_set_encoder_group(g_group_selfcheck);
\t\tif(g_selfcheck_btn_back != NULL) lv_group_focus_obj(g_selfcheck_btn_back);
\t}
\telse if(scr == g_scr_cycle) {
\t\tui_set_encoder_group(g_group_cycle);
\t\tcycle_encoder_group_build();
\t\tif(g_cycle_btn_back != NULL) lv_group_focus_obj(g_cycle_btn_back);
\t}
\telse if(scr == g_scr_admin) {""",
    "screen load selfcheck")

# --- cb_running_countdown cycle hook ---
wash = replace_once(wash,
    "\tif(g_running_remain_sec == 0u) {\n\t\trunning_countdown_reset_all();  //重置倒计时状态\n\t\tui_send_beep_seq(7);  //发送结束蜂鸣\n\t\tui_screen_load(g_scr_end);  //加载目标屏幕\n\t}",
    "\tif(g_running_remain_sec == 0u) {\n\t\trunning_countdown_reset_all();\n\t\tif(g_cycle_active && g_cycle_ui_state == CYCLE_UI_RUNNING) {\n\t\t\tcycle_finish_current_run();\n\t\t\treturn;\n\t\t}\n\t\tui_send_beep_seq(7);\n\t\tui_screen_load(g_scr_end);\n\t}",
    "countdown cycle")

# --- ui_fsm runpause cycle ---
wash = replace_once(wash,
    "static void ui_fsm_runpause_apply_running_page(void)\n{\n\tif(g_ui_child_lock) return;",
    "static void ui_fsm_runpause_apply_running_page(void)\n{\n\tif(g_ui_child_lock) return;\n\tif(g_cycle_active) {\n\t\tcycle_abort_run();\n\t\treturn;\n\t}",
    "runpause cycle")

# --- menu1: sound -> vendor ---
wash = replace_once(wash,
    "        STR_ADMIN_M1_MACHINE_ID, STR_ADMIN_M1_PROGRAM, STR_ADMIN_M1_BRIGHTNESS, STR_ADMIN_M1_SOUND,",
    "        STR_ADMIN_M1_MACHINE_ID, STR_ADMIN_M1_PROGRAM, STR_ADMIN_M1_BRIGHTNESS, STR_ADMIN_M1_VENDOR_MAINT,",
    "menu1 ids")

wash = replace_once(wash,
    "            lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_sound, LV_EVENT_CLICKED, NULL);",
    "            lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_vendor_maint, LV_EVENT_CLICKED, NULL);",
    "menu1 vendor cb")

# --- admin_panel_show: remove SOUND_CONTROL, add VENDOR ---
wash = replace_once(wash,
    "    case SOUND_CONTROL:\n        admin_sound_back_to_menu1();\n        break;",
    "    case VENDOR_SERIAL:\n        admin_vendor_serial_back_to_menu1();\n        break;\n    case VENDOR_MENU:\n        admin_vendor_menu_back_to_menu1();\n        break;",
    "admin back vendor")

# --- admin_encoder_rebuild sound -> vendor serial ---
if "g_admin_view == SOUND_CONTROL" in wash:
    wash = wash.replace("g_admin_view == SOUND_CONTROL", "g_admin_view == VENDOR_SERIAL")

# --- hide vendor panels in admin_panel_show hide all ---
wash = replace_once(wash,
    "    if(g_admin_panel_sound != NULL) lv_obj_add_flag(g_admin_panel_sound, LV_OBJ_FLAG_HIDDEN);",
    "    if(g_admin_panel_vendor_serial != NULL) lv_obj_add_flag(g_admin_panel_vendor_serial, LV_OBJ_FLAG_HIDDEN);\n    if(g_admin_panel_vendor_menu != NULL) lv_obj_add_flag(g_admin_panel_vendor_menu, LV_OBJ_FLAG_HIDDEN);",
    "hide vendor panels")

# --- show vendor panels ---
if "else if(view == SOUND_CONTROL" in wash:
    wash = wash.replace(
        "else if(view == SOUND_CONTROL && g_admin_panel_sound != NULL) {\n        lv_obj_remove_flag(g_admin_panel_sound, LV_OBJ_FLAG_HIDDEN);\n        admin_sound_sync_ui();\n    }",
        """else if(view == VENDOR_SERIAL && g_admin_panel_vendor_serial != NULL) {
        lv_obj_remove_flag(g_admin_panel_vendor_serial, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_ta_vendor_serial != NULL) {
            lv_textarea_set_text(g_admin_ta_vendor_serial, "");
        }
        if(g_admin_kb != NULL) {
            lv_obj_remove_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
            lv_keyboard_set_textarea(g_admin_kb, g_admin_ta_vendor_serial);
        }
    }
    else if(view == VENDOR_MENU && g_admin_panel_vendor_menu != NULL) {
        lv_obj_remove_flag(g_admin_panel_vendor_menu, LV_OBJ_FLAG_HIDDEN);
    }""")

# --- cb_admin_ta_ready vendor serial ---
wash = replace_once(wash,
    "\t\tadmin_password_try();",
    "\t\tif(g_admin_view == VENDOR_SERIAL) {\n\t\t\tadmin_vendor_serial_try();\n\t\t\treturn;\n\t\t}\n\t\tadmin_password_try();",
    "ta ready vendor")

# --- ui_lang_apply_all pay_sync ---
wash = replace_once(wash,
    "    home_sync_program_labels();",
    "    home_sync_program_labels();\n    pay_sync_pay_ui();",
    "lang apply pay")

# --- replace build_pay ---
wash = replace_once(wash,
    "static void build_pay(void)\n{\n\t\tlv_obj_t * root = lv_obj_create(g_scr_pay);",
    build_pay.rstrip() + "\n\nstatic void build_pay_OLD_REMOVED(void)\n{\n\t\tlv_obj_t * root = lv_obj_create(g_scr_pay);",
    "build_pay")

# Remove old build_pay body - find build_pay_OLD and delete until build_pay_done
import re
wash = re.sub(
    r"static void build_pay_OLD_REMOVED\(void\)\s*\{.*?(?=static void build_pay_done)",
    "",
    wash,
    count=1,
    flags=re.DOTALL,
)

# --- insert vendor panels in build_admin before program_admin_build_panel ---
vendor_panels = """
    g_admin_panel_vendor_serial = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_vendor_serial, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_vendor_serial, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_vendor_serial, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_vendor_serial, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_vendor_serial, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_vendor_serial, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_vendor_serial, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_vendor_serial_title = lv_label_create(g_admin_panel_vendor_serial);
    ui_lang_bind_label(g_admin_lbl_vendor_serial_title, STR_ADMIN_M1_VENDOR_MAINT);
    lv_obj_set_style_text_color(g_admin_lbl_vendor_serial_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_vendor_serial_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_vendor_serial_title, LV_ALIGN_TOP_MID, 0, 74);

    lv_obj_t * lbl_vendor_hint = lv_label_create(g_admin_panel_vendor_serial);
    ui_lang_bind_label(lbl_vendor_hint, STR_VENDOR_SERIAL_HINT);
    lv_obj_set_style_text_color(lbl_vendor_hint, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_vendor_hint, s_font_sc_30);
    lv_obj_align(lbl_vendor_hint, LV_ALIGN_TOP_MID, 0, 160);

    g_admin_ta_vendor_serial = lv_textarea_create(g_admin_panel_vendor_serial);
    lv_obj_set_size(g_admin_ta_vendor_serial, 320, 48);
    lv_obj_align(g_admin_ta_vendor_serial, LV_ALIGN_TOP_MID, 0, 160);
    lv_textarea_set_one_line(g_admin_ta_vendor_serial, true);
    lv_textarea_set_max_length(g_admin_ta_vendor_serial, 6);
    lv_textarea_set_accepted_chars(g_admin_ta_vendor_serial, "0123456789");
    lv_obj_set_style_text_font(g_admin_ta_vendor_serial, s_font_sc_30, LV_PART_MAIN);
    lv_obj_add_event_cb(g_admin_ta_vendor_serial, cb_admin_ta_ready, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(g_admin_ta_vendor_serial, cb_admin_ta_key_enter, LV_EVENT_KEY | LV_EVENT_PREPROCESS, NULL);
    lv_obj_add_event_cb(g_admin_ta_vendor_serial, cb_admin_ta_kb_focus, LV_EVENT_ALL, NULL);

    g_admin_panel_vendor_menu = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_vendor_menu, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_vendor_menu, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_vendor_menu, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_vendor_menu, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_vendor_menu, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_vendor_menu, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_vendor_menu, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t * lbl_vendor_menu_title = lv_label_create(g_admin_panel_vendor_menu);
    ui_lang_bind_label(lbl_vendor_menu_title, STR_ADMIN_M1_VENDOR_MAINT);
    lv_obj_set_style_text_color(lbl_vendor_menu_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_vendor_menu_title, s_font_sc_30);
    lv_obj_align(lbl_vendor_menu_title, LV_ALIGN_TOP_MID, 0, 24);

    g_admin_btn_vendor_self_check = make_admin_menu_btn(g_admin_panel_vendor_menu, ui_translation(STR_VENDOR_SELF_CHECK));
    lv_obj_set_size(g_admin_btn_vendor_self_check, 280, 56);
    lv_obj_align(g_admin_btn_vendor_self_check, LV_ALIGN_CENTER, 0, -40);
    admin_menu_btn_bind_i18n(g_admin_btn_vendor_self_check, STR_VENDOR_SELF_CHECK);
    lv_obj_add_event_cb(g_admin_btn_vendor_self_check, cb_admin_open_selfcheck, LV_EVENT_CLICKED, NULL);

    g_admin_btn_vendor_self_learn = make_admin_menu_btn(g_admin_panel_vendor_menu, ui_translation(STR_VENDOR_SELF_LEARN));
    lv_obj_set_size(g_admin_btn_vendor_self_learn, 280, 56);
    lv_obj_align(g_admin_btn_vendor_self_learn, LV_ALIGN_CENTER, 0, 40);
    admin_menu_btn_bind_i18n(g_admin_btn_vendor_self_learn, STR_VENDOR_SELF_LEARN);
    lv_obj_add_event_cb(g_admin_btn_vendor_self_learn, cb_admin_open_cycle, LV_EVENT_CLICKED, NULL);

"""

wash = replace_once(wash,
    "    program_admin_build_panel(root, body_y, body_h);",
    vendor_panels + "    program_admin_build_panel(root, body_y, body_h);",
    "vendor panels build_admin")

# --- insert extracted + helpers before build_off ---
big_block = ui_admin_resume + helpers + extracted + vendor_block + pay_cb
wash = replace_once(wash,
    "static void build_off(void)",
    big_block + "\nstatic void build_off(void)",
    "insert selfcheck/cycle block")

# --- ui_init groups and build ---
wash = replace_once(wash,
    "\t\tg_group_admin = lv_group_create();",
    "\t\tg_group_selfcheck = lv_group_create();\n\t\tg_group_cycle = lv_group_create();\n\t\tg_group_admin = lv_group_create();",
    "init groups")

wash = replace_once(wash,
    "\t\tlv_group_set_wrap(g_group_admin, false);",
    "\t\tlv_group_set_wrap(g_group_selfcheck, false);\n\t\tlv_group_set_wrap(g_group_cycle, false);\n\t\tlv_group_set_wrap(g_group_admin, false);",
    "init group wrap")

wash = replace_once(wash,
    "\t\tbuild_pay();                                       /* 支付页：金额/扫码提示 label */",
    "\t\tbuild_selfcheck();\n\t\tbuild_cycle();\n\t\tbuild_pay();",
    "init build selfcheck")

# --- payment settings add checkbox callbacks if missing ---
if "cb_admin_payment_alipay_changed" not in wash.split("build_admin")[1][:5000]:
    pass  # inserted in pay_cb block

# --- replace cycle stubs at end ---
wash = replace_once(wash,
    """/* PC 仿真：循环程序占位 API（商用洗暂无循环程序页） */
bool ui_cycle_is_active(void)
{
\treturn false;
}

void ui_cycle_report_fault(uint8_t fault_code_1_based)
{
\t(void)fault_code_1_based;
}""",
    """bool ui_cycle_is_active(void)
{
\treturn g_cycle_active;
}

void ui_cycle_report_fault(uint8_t fault_code_1_based)
{
\tif(!g_cycle_active) return;
\tif(fault_code_1_based < 1u || fault_code_1_based > CYCLE_FAULT_CODE_MAX) return;
\tg_cycle_session.fault_code = fault_code_1_based;
\tg_cycle_session.fault_at_count = g_cycle_session.run_count;
\tif(g_cycle_ui_state == CYCLE_UI_RUNNING) {
\t\trunning_countdown_reset_all();
\t\tfsm_state_change(FSM_STANDBY);
\t}
\tcycle_enter_fault_display();
}

void ui_cycle_run_complete(void)
{
\tif(!g_cycle_active) return;
\tcycle_finish_current_run();
}

void ui_cycle_mode_enter(void) { /* TODO: MCU */ }
void ui_cycle_mode_exit(void) { /* TODO: MCU */ }

int16_t ui_selfcheck_live_temp_get(void) { return g_selfcheck_live_temp_c; }
void ui_selfcheck_live_temp_set(int16_t temp_c)
{
\tg_selfcheck_live_temp_c = temp_c;
\tselfcheck_temp_label_sync();
}""",
    "cycle stubs")

# --- hook payment checkbox in build_admin if not present ---
if "cb_admin_payment_alipay_changed" in wash and "cb_admin_payment_alipay_changed, LV_EVENT_VALUE_CHANGED" not in wash:
    wash = replace_once(wash,
        "lv_obj_add_event_cb(g_admin_cb_payment_alipay,",
        "lv_obj_add_event_cb(g_admin_cb_payment_alipay, cb_admin_payment_alipay_changed, LV_EVENT_VALUE_CHANGED, NULL);\n    lv_obj_add_event_cb(g_admin_cb_payment_alipay,",
        "pay alipay hook")

WASH.write_text(wash, encoding="utf-8")
print("Phase 2 complete")
