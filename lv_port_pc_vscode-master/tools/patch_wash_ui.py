#!/usr/bin/env python3
"""Patch ui-商用洗.c with features from ui-烘干硬件.c"""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WASH = ROOT / "src/ui/ui-商用洗.c"
DRY = ROOT / "src/ui/ui-烘干硬件.c"

def lines(path):
    return path.read_text(encoding="utf-8").splitlines(keepends=True)

def replace_once(text, old, new, label):
    if old not in text:
        raise ValueError(f"Patch failed [{label}]")
    return text.replace(old, new, 1)

def extract_lines(src_lines, start, end):
    return "".join(src_lines[start - 1 : end])

dry_lines = lines(DRY)
wash = WASH.read_text(encoding="utf-8")

# ========== Phase 1: strings & enums ==========
wash = replace_once(wash,
    "    STR_PAY_HINT,\n    STR_PAY_DONE,",
    """    STR_PAY_HINT,
    STR_PAY_HINT_ALIPAY,
    STR_PAY_HINT_WECHAT,
    STR_PAY_HINT_ALIPAY_1,
    STR_PAY_HINT_ALIPAY_2,
    STR_PAY_HINT_WECHAT_1,
    STR_PAY_HINT_WECHAT_2,
    STR_PAY_DONE,""", "enum pay")

wash = replace_once(wash,
    "    STR_ADMIN_M1_SOUND,\n    STR_ADMIN_M1_LANGUAGE,",
    """    STR_ADMIN_M1_VENDOR_MAINT,
    STR_ADMIN_M1_LANGUAGE,
    STR_VENDOR_SERIAL_HINT,
    STR_VENDOR_SELF_CHECK,
    STR_VENDOR_SELF_LEARN,
    STR_SELF_CHECK_TITLE,
    STR_SELF_CHECK_DONE,
    STR_CYCLE_TITLE,
    STR_CYCLE_RUN_COUNT_FMT,""", "enum vendor")

wash = replace_once(wash,
    '        [STR_PAY_HINT]       = "请扫描屏幕上二维码\\n支付完成机器自动运行",\n        [STR_PAY_DONE]       = "支付完成",',
    '''        [STR_PAY_HINT]       = "请扫描屏幕上二维码\\n支付完成机器自动运行",
        [STR_PAY_HINT_ALIPAY] = "请扫描屏幕上支付宝支付二维码\\n支付完成机器自动运行",
        [STR_PAY_HINT_WECHAT] = "请扫描屏幕上微信支付二维码\\n支付完成机器自动运行",
        [STR_PAY_HINT_ALIPAY_1] = "请扫描屏幕上支付宝支付二维码",
        [STR_PAY_HINT_ALIPAY_2] = "支付完成机器自动运行",
        [STR_PAY_HINT_WECHAT_1] = "请扫描屏幕上微信支付二维码",
        [STR_PAY_HINT_WECHAT_2] = "支付完成机器自动运行",
        [STR_PAY_DONE]       = "支付完成",''', "zh pay")

wash = replace_once(wash,
    '        [STR_ADMIN_M1_SOUND]         = "声音控制",\n        [STR_ADMIN_M1_LANGUAGE]      = "语言设置",',
    '''        [STR_ADMIN_M1_VENDOR_MAINT]  = "厂商维护",
        [STR_ADMIN_M1_LANGUAGE]      = "语言设置",
        [STR_VENDOR_SERIAL_HINT]      = "请输入特殊出厂序列号",
        [STR_VENDOR_SELF_CHECK]     = "1、自检程序",
        [STR_VENDOR_SELF_LEARN]     = "2、循环程序",
        [STR_SELF_CHECK_TITLE]      = "自检程序",
        [STR_SELF_CHECK_DONE]       = "自检完成",
        [STR_CYCLE_TITLE]           = "循环程序",
        [STR_CYCLE_RUN_COUNT_FMT]   = "已完成：%u 次",''', "zh vendor")

wash = replace_once(wash,
    '        [STR_PAY_HINT]       = "Scan the QR code on screen\\nMachine starts after payment",\n        [STR_PAY_DONE]       = "Payment Done",',
    '''        [STR_PAY_HINT]       = "Scan the QR code on screen\\nMachine starts after payment",
        [STR_PAY_HINT_ALIPAY] = "Scan the Alipay QR code on screen\\nMachine starts after payment",
        [STR_PAY_HINT_WECHAT] = "Scan the WeChat Pay QR code on screen\\nMachine starts after payment",
        [STR_PAY_HINT_ALIPAY_1] = "Scan the Alipay QR code on screen",
        [STR_PAY_HINT_ALIPAY_2] = "Machine starts after payment",
        [STR_PAY_HINT_WECHAT_1] = "Scan the WeChat Pay QR code on screen",
        [STR_PAY_HINT_WECHAT_2] = "Machine starts after payment",
        [STR_PAY_DONE]       = "Payment Done",''', "en pay")

wash = replace_once(wash,
    '        [STR_ADMIN_M1_SOUND]         = "Sound",\n        [STR_ADMIN_M1_LANGUAGE]      = "Language",',
    '''        [STR_ADMIN_M1_VENDOR_MAINT]  = "Vendor Maint.",
        [STR_ADMIN_M1_LANGUAGE]      = "Language",
        [STR_VENDOR_SERIAL_HINT]      = "Enter factory serial no.",
        [STR_VENDOR_SELF_CHECK]     = "1. Self-check Prog.",
        [STR_VENDOR_SELF_LEARN]     = "2. Cycle Prog.",
        [STR_SELF_CHECK_TITLE]      = "Self-check Prog.",
        [STR_SELF_CHECK_DONE]       = "Self Check Done",
        [STR_CYCLE_TITLE]           = "Cycle Prog.",
        [STR_CYCLE_RUN_COUNT_FMT]   = "Completed: %u",''', "en vendor")

wash = replace_once(wash,
    "    SOUND_CONTROL,\t\t//声音控制设置页面",
    "    VENDOR_SERIAL,\t\t//厂商维护：出厂序列号\n    VENDOR_MENU,\t\t//厂商维护：功能选择",
    "admin enum")

wash = replace_once(wash,
    "} admin_view_t;\n\ntypedef enum {\n    ADMIN_FACTORY_PHASE_PROMPT,",
    """} admin_view_t;

#define MACHINE_MODEL           "XQG150-001"
#define VENDOR_SERIAL_CODE      "111111"
#define SELFCHECK_STEP_UI_MS    3000u
#define SELFCHECK_INIT_UI_MS    3000u
#define SELFCHECK_FLASH_MS      500u
#define SELFCHECK_BLINK_MS      500u
#define SELFCHECK_STATUS_ROW_Y  ((lv_coord_t)((UI_FIXED_H * 30) / 100))
#define SELFCHECK_STATUS_ROW_W  ((lv_coord_t)((UI_FIXED_W * 87) / 100))
#define SELFCHECK_LIVE_TEMP_DEFAULT_C  45
#define SELFCHECK_STEP_COUNT    3

typedef enum {
    SELFCHECK_UI_MODEL,
    SELFCHECK_UI_FLASH,
    SELFCHECK_UI_INIT,
    SELFCHECK_UI_S1,
    SELFCHECK_UI_S2,
    SELFCHECK_UI_S3,
    SELFCHECK_UI_DONE,
} selfcheck_ui_state_t;

typedef enum {
    CYCLE_UI_SETUP,
    CYCLE_UI_RUNNING,
    CYCLE_UI_FAULT,
    CYCLE_UI_ABORTED,
} cycle_ui_state_t;

typedef enum {
    CYCLE_FAULT_SHOW_CODE,
    CYCLE_FAULT_GAP1,
    CYCLE_FAULT_SHOW_COUNT,
    CYCLE_FAULT_GAP2,
} cycle_fault_phase_t;

#define CYCLE_FAULT_CODE_MAX  14u
#define CYCLE_FAULT_BLINK_MS  500u
#define CYCLE_PROG_FIELD_CNT  6u

typedef enum {
    ADMIN_FACTORY_PHASE_PROMPT,""", "selfcheck typedefs phase2")

# cycle_session_t must come after ui_program_admin_t
wash = replace_once(wash,
    "} ui_program_admin_t;\n\nstatic ui_program_profile_t g_program_profiles",
    """} ui_program_admin_t;

typedef struct {
    int32_t prog_idx;
    ui_program_admin_t cfg;
    uint32_t run_count;
    uint32_t fault_at_count;
    uint8_t fault_code;
} cycle_session_t;

static ui_program_profile_t g_program_profiles""", "cycle_session_t")

print("Phase 1 done")
WASH.write_text(wash, encoding="utf-8")
