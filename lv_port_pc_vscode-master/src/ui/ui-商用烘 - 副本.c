
#include "ui.h"
#include "LVGLPort.h"
#include "fonts/ui_fonts.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "agreement.h"
#include "task_fsm.h"
#include "init.h"

#include <stdlib.h>

/* 1：PC 调试：支付页点启停 → 支付完成 → 2s 进运行页。
 * 0：正式发布：支付页点启停 → 无任何效果。*/
#define UI_DEBUG_PAY_SKIP_TO_DONE  1

/* 1：PC 仿真运行页温度初值。
 * 0：实机：取消仿真，由 ui_running_live_temp_set 推送。*/
#define UI_RUNNING_LIVE_PC_SIM  1

// #define BEEP_POWER 1
// #define BEEP_EXCEPTION 2
// #define BEEP_CHILD_LOCK_ON 3
// #define BEEP_CHECK_AND_FACTORY 4
// #define BEEP_START 5
// #define BEEP_PAUSE 6
// #define BEEP_END 7
// #define BEEP_PROGRAM_CHOOSE 8
// #define BEEP_ADDITIONAL_SETTING 9
// #define BEEP_RESERVE 10
// #define BEEP_CHILD_LOCK_OFF 11

static bool need_scr_load = false;
uint16_t param_change[8];

/* 向蜂鸣器任务发送音效序号（RTOS 队列；PC 仿真为空实现） */
static void ui_send_beep_seq(int32_t seq)
{
#if USE_RTOS_FREERTOS
    rtos *app = get_rtos();                              /* 获取 RTOS 应用句柄 */
    if(app == NULL || app->q_beep == NULL) {             /* 未初始化则直接返回 */
        return;
    }
    BeepReq req = { seq, 0 };                            /* 组装蜂鸣请求：序号 + 保留 */
    (void)xQueueSend(app->q_beep, &req, pdMS_TO_TICKS(10)); /* 非阻塞入队，超时 10ms */
#else
    (void)seq;                                           /* PC 仿真：忽略序号 */
#endif
}

/* ============================================================================
 * 防缠绕功能 — 硬件联动接口（仅在本文件 ui.c 说明与实现）
 * ============================================================================
 *
 * 【对外符号】硬件模块在自家 .c 顶部自行添加 extern 声明即可链接，无需改 ui.h：
 *
 *   typedef void (*ui_auto_dispense_changed_cb_t)(bool enabled);
 *
 *   extern void ui_auto_dispense_hw_register(ui_auto_dispense_changed_cb_t cb);
 *   extern bool ui_auto_dispense_get(void);
 *   extern bool ui_auto_dispense_set(bool enabled);
 *   extern void ui_auto_dispense_sync_to_hw(void);
 *
 * enabled == true  ：防缠绕开启（管理员页选「开启」）
 * enabled == false ：防缠绕关闭（管理员页选「关闭」或恢复出厂默认）
 *
 * 【职责划分】
 *   - UI：维护显示与用户选择；变更时调用 ui_auto_dispense_set。
 *   - 硬件：实现 cb，在 cb 内写 Modbus 保持寄存器、控制防缠绕电机/逻辑、写 Flash 等。
 *     勿在 cb 内调用 LVGL API 或长时间阻塞；跨任务请用队列投递。
 *
 * 【推荐启动顺序（真机示例）】
 *
 *   static void on_auto_dispense_changed(bool enabled)
 *   {
 *       holding_register[MB_HOLD_xxx] = enabled ? 1u : 0u;
 *   }
 *
 *   void board_init(void)
 *   {
 *       ui_auto_dispense_hw_register(on_auto_dispense_changed);
 *       bool saved = nvm_read_auto_dispense();
 *       ui_auto_dispense_set(saved);
 *       ui_init();
 *   }
 *
 * 【用户点「开启/关闭」】UI 内部 → ui_auto_dispense_set → 刷新按钮 → cb(enabled)。
 * 【恢复出厂】admin 恢复默认 → ui_auto_dispense_set(false) → cb(false)。
 * 【只读】if(ui_auto_dispense_get()) { ... }
 * 【通信重连】ui_auto_dispense_sync_to_hw() 再推送当前状态，不改动 UI。
 * 【PC 仿真】未 register 时 get/set 仍有效，仅无硬件回调。
 * ============================================================================ */

typedef void (*ui_auto_dispense_changed_cb_t)(bool enabled);

static bool g_ui_auto_dispense_enabled = false;
static ui_auto_dispense_changed_cb_t s_auto_dispense_hw_cb;

static void admin_auto_dispense_sync_btn_ui(void);  //防缠绕功能页：刷新开启/关闭按钮选中样式
static void admin_payment_sync_method_ui(void);  //支付设置页：刷新支付方式复选框
static void admin_payment_sync_list_ui(void);
static void cb_admin_brightness_slider_draw_grip(lv_event_t * e);
static void cb_admin_brightness_slider_ext_draw_size(lv_event_t * e);
static void admin_wifi_timer_stop(void);
static void cb_admin_wifi_timer(lv_timer_t * t);
static void admin_wifi_apply_switch_layout(void);
static void admin_wifi_sync_switch_ui(void);
static void admin_sw_apply_knob_pad(lv_obj_t * sw, lv_coord_t knob_diam, lv_style_selector_t selector);
static int admin_data_upload_index_from_cb(lv_obj_t * cb);
static void admin_data_sync_upload_items_ui(void);  //数据设置页：刷新上传项复选框
static void admin_data_sync_strategy_ui(void);  //数据设置页：刷新上传策略互斥复选框
static void admin_4g_sync_switch_ui(void);  //4G 设置页：刷新开关与 ui_4g_get 一致

/* 通知已注册的硬件回调（未注册则无操作） */
static void ui_auto_dispense_apply_hw(bool enabled)
{
    if(s_auto_dispense_hw_cb != NULL) {
        s_auto_dispense_hw_cb(enabled);
    }
}

/* 注册防缠绕功能状态变化回调；保存后立即以当前 enabled 调用一次 cb */
void ui_auto_dispense_hw_register(ui_auto_dispense_changed_cb_t cb)
{
    s_auto_dispense_hw_cb = cb;
    if(cb != NULL) {
        cb(g_ui_auto_dispense_enabled);
    }
}

/* 读取防缠绕功能是否开启（true=开启，false=关闭） */
bool ui_auto_dispense_get(void)
{
    return g_ui_auto_dispense_enabled;
}

/* 设置防缠绕功能开关；与当前相同返回 false 且不回调；否则刷新管理员页按钮并通知硬件 */
bool ui_auto_dispense_set(bool enabled)
{
    if(enabled == g_ui_auto_dispense_enabled) {
        return false;
    }
    g_ui_auto_dispense_enabled = enabled;
    admin_auto_dispense_sync_btn_ui();
    ui_auto_dispense_apply_hw(enabled);
    return true;
}

/* 将当前防缠绕开关状态再次推送给硬件（不修改 UI，用于通信重连等） */
void ui_auto_dispense_sync_to_hw(void)
{
    ui_auto_dispense_apply_hw(g_ui_auto_dispense_enabled);
}

/* ============================================================================
 * 新风护理 — 硬件联动接口（仅在本文件 ui.c 说明与实现）
 * ============================================================================
 *
 * 【对外符号】硬件模块在自家 .c 顶部自行添加 extern 声明即可链接，无需改 ui.h：
 *
 *   typedef void (*ui_fresh_air_care_changed_cb_t)(bool enabled);
 *
 *   extern void ui_fresh_air_care_hw_register(ui_fresh_air_care_changed_cb_t cb);
 *   extern bool ui_fresh_air_care_get(void);
 *   extern bool ui_fresh_air_care_set(bool enabled);
 *   extern void ui_fresh_air_care_sync_to_hw(void);
 *
 * enabled == true  ：新风护理开启（管理员页选「开启」、上电默认、恢复出厂默认）
 * enabled == false ：新风护理关闭（管理员页选「关闭」）
 *
 * 【职责划分】
 *   - UI：维护显示与用户选择；变更时调用 ui_fresh_air_care_set。
 *   - 硬件：实现 cb，在 cb 内写 Modbus 保持寄存器、控制洗衣完成后内筒间歇转动、写 Flash 等。
 *     勿在 cb 内调用 LVGL API 或长时间阻塞；跨任务请用队列投递。
 *
 * 【推荐启动顺序（真机示例）】
 *
 *   static void on_fresh_air_care_changed(bool enabled)
 *   {
 *       holding_register[MB_HOLD_xxx] = enabled ? 1u : 0u;
 *       drum_intermittent_set(enabled);
 *   }
 *
 *   void board_init(void)
 *   {
 *       ui_fresh_air_care_hw_register(on_fresh_air_care_changed);
 *       bool saved = nvm_read_fresh_air_care();
 *       ui_fresh_air_care_set(saved);
 *       ui_init();
 *   }
 *
 * 【用户点「开启/关闭」】UI 内部 → ui_fresh_air_care_set → 刷新按钮 → cb(enabled)。
 * 【恢复出厂】admin 恢复默认 → ui_fresh_air_care_set(true) → cb(true)。
 * 【只读】if(ui_fresh_air_care_get()) { ... }  // 洗衣完成后是否间歇转动内筒
 * 【通信重连】ui_fresh_air_care_sync_to_hw() 再推送当前状态，不改动 UI。
 * 【PC 仿真】未 register 时 get/set 仍有效，仅无硬件回调。
 * ============================================================================ */

typedef void (*ui_fresh_air_care_changed_cb_t)(bool enabled);

static bool g_ui_fresh_air_care_enabled = true;  /* 上电默认开启 */
static ui_fresh_air_care_changed_cb_t s_fresh_air_care_hw_cb;

static void admin_fresh_air_care_sync_btn_ui(void);  //新风护理页：刷新开启/关闭按钮选中样式

/* 通知已注册的新风护理硬件回调（未注册则无操作） */
static void ui_fresh_air_care_apply_hw(bool enabled)
{
    if(s_fresh_air_care_hw_cb != NULL) {
        s_fresh_air_care_hw_cb(enabled);
    }
}

/* 注册新风护理状态变化回调；保存后立即以当前 enabled 调用一次 cb */
void ui_fresh_air_care_hw_register(ui_fresh_air_care_changed_cb_t cb)
{
    s_fresh_air_care_hw_cb = cb;
    if(cb != NULL) {
        cb(g_ui_fresh_air_care_enabled);
    }
}

/* 读取新风护理是否开启（true=开启，false=关闭） */
bool ui_fresh_air_care_get(void)
{
    return g_ui_fresh_air_care_enabled;
}

/* 设置新风护理开关；与当前相同返回 false 且不回调；否则刷新管理员页按钮并通知硬件 */
bool ui_fresh_air_care_set(bool enabled)
{
    if(enabled == g_ui_fresh_air_care_enabled) {
        return false;
    }
    g_ui_fresh_air_care_enabled = enabled;
    admin_fresh_air_care_sync_btn_ui();
    ui_fresh_air_care_apply_hw(enabled);
    return true;
}

/* 将当前新风护理开关状态再次推送给硬件（不修改 UI，用于通信重连等） */
void ui_fresh_air_care_sync_to_hw(void)
{
    ui_fresh_air_care_apply_hw(g_ui_fresh_air_care_enabled);
}

/* ============================================================================
 * 支付设置 — 状态维护（支付宝/微信开关、支付超时秒数）
 * ============================================================================ */

#define PAYMENT_TIMEOUT_SEC_DEFAULT  180u
#define PAYMENT_TIMEOUT_SEC_MIN      1u
#define PAYMENT_TIMEOUT_SEC_MAX      (59u * 60u + 59u)
#define TP_MMSS_MAX_MIN              59u
#define TP_ROLLER_OPTS_0_9           "0\n1\n2\n3\n4\n5\n6\n7\n8\n9"
#define TP_ROLLER_OPTS_0_5           "0\n1\n2\n3\n4\n5"

static bool g_ui_payment_alipay_enabled = true;
static bool g_ui_payment_wechat_enabled = true;
static bool g_ui_payment_order_enabled = true;
static uint16_t g_ui_payment_timeout_sec = PAYMENT_TIMEOUT_SEC_DEFAULT;

/* 读取支付宝支付是否启用 */
bool ui_payment_alipay_get(void)
{
    return g_ui_payment_alipay_enabled;
}

/* 设置支付宝支付开关；与当前相同返回 false */
bool ui_payment_alipay_set(bool enabled)
{
    if(enabled == g_ui_payment_alipay_enabled) {
        return false;
    }
    g_ui_payment_alipay_enabled = enabled;
    admin_payment_sync_method_ui();
    return true;
}

/* 读取微信支付是否启用 */
bool ui_payment_wechat_get(void)
{
    return g_ui_payment_wechat_enabled;
}

/* 设置微信支付开关；与当前相同返回 false */
bool ui_payment_wechat_set(bool enabled)
{
    if(enabled == g_ui_payment_wechat_enabled) {
        return false;
    }
    g_ui_payment_wechat_enabled = enabled;
    admin_payment_sync_method_ui();
    return true;
}

/* 读取支付超时秒数（60/120/180/240/300） */
uint16_t ui_payment_timeout_sec_get(void)
{
    return g_ui_payment_timeout_sec;
}

/* 设置支付超时秒数；非法值按 180 秒处理；与当前相同返回 false */
bool ui_payment_timeout_sec_set(uint16_t sec)
{
    if(sec < PAYMENT_TIMEOUT_SEC_MIN) sec = PAYMENT_TIMEOUT_SEC_MIN;
    if(sec > PAYMENT_TIMEOUT_SEC_MAX) sec = PAYMENT_TIMEOUT_SEC_MAX;
    if(sec == g_ui_payment_timeout_sec) {
        return false;
    }
    g_ui_payment_timeout_sec = sec;
    return true;
}

/* ============================================================================
 * 数据设置 — 状态维护（上传项开关、上传策略）
 * ============================================================================ */

typedef enum {
    UI_DATA_STRATEGY_REALTIME,
    UI_DATA_STRATEGY_SCHEDULED,
    UI_DATA_STRATEGY_WIFI_ONLY,
    UI_DATA_STRATEGY_4G_ONLY,
    UI_DATA_STRATEGY_FORBIDDEN,
} ui_data_upload_strategy_t;

static bool g_ui_data_upload_basic = true;
static bool g_ui_data_upload_sensor = true;
static bool g_ui_data_upload_fault = true;
static bool g_ui_data_upload_auto_dispense = false;
static bool g_ui_data_upload_payment_order = false;
static bool g_ui_data_upload_user_op = false;
static bool g_ui_data_upload_device = true;
#define ADMIN_DATA_UPLOAD_COUNT 7
static ui_data_upload_strategy_t g_ui_data_upload_strategy[ADMIN_DATA_UPLOAD_COUNT] = {
    UI_DATA_STRATEGY_4G_ONLY, UI_DATA_STRATEGY_4G_ONLY, UI_DATA_STRATEGY_4G_ONLY,
    UI_DATA_STRATEGY_4G_ONLY, UI_DATA_STRATEGY_4G_ONLY, UI_DATA_STRATEGY_4G_ONLY,
    UI_DATA_STRATEGY_4G_ONLY
};
static int g_admin_data_strategy_item = 0;

/* 读取「基础运行数据」上传项是否开启 */
bool ui_data_upload_basic_get(void)
{
    return g_ui_data_upload_basic;
}

/* 设置「基础运行数据」上传项；与当前相同返回 false */
bool ui_data_upload_basic_set(bool enabled)
{
    if(enabled == g_ui_data_upload_basic) {
        return false;
    }
    g_ui_data_upload_basic = enabled;
    admin_data_sync_upload_items_ui();
    return true;
}

/* 读取「传感器数据」上传项是否开启 */
bool ui_data_upload_sensor_get(void)
{
    return g_ui_data_upload_sensor;
}

/* 设置「传感器数据」上传项；与当前相同返回 false */
bool ui_data_upload_sensor_set(bool enabled)
{
    if(enabled == g_ui_data_upload_sensor) {
        return false;
    }
    g_ui_data_upload_sensor = enabled;
    admin_data_sync_upload_items_ui();
    return true;
}

/* 读取「故障日志」上传项是否开启 */
bool ui_data_upload_fault_get(void)
{
    return g_ui_data_upload_fault;
}

/* 设置「故障日志」上传项；与当前相同返回 false */
bool ui_data_upload_fault_set(bool enabled)
{
    if(enabled == g_ui_data_upload_fault) {
        return false;
    }
    g_ui_data_upload_fault = enabled;
    admin_data_sync_upload_items_ui();
    return true;
}

/* 读取「自动投放数据」上传项是否开启 */
bool ui_data_upload_auto_dispense_get(void)
{
    return g_ui_data_upload_auto_dispense;
}

/* 设置「自动投放数据」上传项；与当前相同返回 false */
bool ui_data_upload_auto_dispense_set(bool enabled)
{
    if(enabled == g_ui_data_upload_auto_dispense) {
        return false;
    }
    g_ui_data_upload_auto_dispense = enabled;
    admin_data_sync_upload_items_ui();
    return true;
}

/* 读取「支付订单」上传项是否开启 */
bool ui_data_upload_payment_order_get(void)
{
    return g_ui_data_upload_payment_order;
}

/* 设置「支付订单」上传项；与当前相同返回 false */
bool ui_data_upload_payment_order_set(bool enabled)
{
    if(enabled == g_ui_data_upload_payment_order) {
        return false;
    }
    g_ui_data_upload_payment_order = enabled;
    admin_data_sync_upload_items_ui();
    return true;
}

/* 读取「用户操作记录」上传项是否开启 */
bool ui_data_upload_user_op_get(void)
{
    return g_ui_data_upload_user_op;
}

/* 设置「用户操作记录」上传项；与当前相同返回 false */
bool ui_data_upload_user_op_set(bool enabled)
{
    if(enabled == g_ui_data_upload_user_op) {
        return false;
    }
    g_ui_data_upload_user_op = enabled;
    admin_data_sync_upload_items_ui();
    return true;
}

/* 读取「设备状态」上传项是否开启 */
bool ui_data_upload_device_get(void)
{
    return g_ui_data_upload_device;
}

/* 设置「设备状态」上传项；与当前相同返回 false */
bool ui_data_upload_device_set(bool enabled)
{
    if(enabled == g_ui_data_upload_device) {
        return false;
    }
    g_ui_data_upload_device = enabled;
    admin_data_sync_upload_items_ui();
    return true;
}

ui_data_upload_strategy_t ui_data_upload_strategy_get(void)
{
    if(g_admin_data_strategy_item < 0 || g_admin_data_strategy_item >= ADMIN_DATA_UPLOAD_COUNT) {
        return UI_DATA_STRATEGY_4G_ONLY;
    }
    return g_ui_data_upload_strategy[g_admin_data_strategy_item];
}

bool ui_data_upload_strategy_set(ui_data_upload_strategy_t strategy)
{
    if(strategy > UI_DATA_STRATEGY_FORBIDDEN) {
        strategy = UI_DATA_STRATEGY_4G_ONLY;
    }
    if(g_admin_data_strategy_item < 0 || g_admin_data_strategy_item >= ADMIN_DATA_UPLOAD_COUNT) {
        return false;
    }
    if(strategy == g_ui_data_upload_strategy[g_admin_data_strategy_item]) {
        return false;
    }
    g_ui_data_upload_strategy[g_admin_data_strategy_item] = strategy;
    admin_data_sync_strategy_ui();
    return true;
}

static void ui_data_upload_strategy_reset_all(ui_data_upload_strategy_t strategy)
{
    unsigned i;
    if(strategy > UI_DATA_STRATEGY_FORBIDDEN) {
        strategy = UI_DATA_STRATEGY_4G_ONLY;
    }
    for(i = 0; i < ADMIN_DATA_UPLOAD_COUNT; i++) {
        g_ui_data_upload_strategy[i] = strategy;
    }
    g_admin_data_strategy_item = 0;
    admin_data_sync_strategy_ui();
}

/* ============================================================================
 * 4G 开关 — 状态维护（管理员 4G 设置页）
 * ============================================================================ */


typedef void (*ui_wifi_changed_cb_t)(bool enabled);
static bool g_ui_wifi_enabled = false;
static ui_wifi_changed_cb_t s_wifi_hw_cb;
int8_t g_ui_wifi_connect_result = 0;
int8_t g_ui_4g_connect_result = 0;

static void ui_wifi_apply_hw(bool enabled)
{
    if(s_wifi_hw_cb != NULL) s_wifi_hw_cb(enabled);
}
void ui_wifi_hw_register(ui_wifi_changed_cb_t cb)
{
    s_wifi_hw_cb = cb;
    if(cb != NULL) cb(g_ui_wifi_enabled);
}
bool ui_wifi_get(void) { return g_ui_wifi_enabled; }
bool ui_wifi_set(bool enabled)
{
    if(enabled == g_ui_wifi_enabled) return false;
    g_ui_wifi_enabled = enabled;
    admin_wifi_sync_switch_ui();
    ui_wifi_apply_hw(enabled);
    return true;
}
void ui_wifi_sync_to_hw(void) { ui_wifi_apply_hw(g_ui_wifi_enabled); }

typedef void (*ui_4g_changed_cb_t)(bool enabled);

static bool g_ui_4g_enabled = true;  /* 上电默认开启 */
static ui_4g_changed_cb_t s_4g_hw_cb;

/* 通知已注册的硬件回调（未注册则无操作） */
static void ui_4g_apply_hw(bool enabled)
{
    if(s_4g_hw_cb != NULL) {
        s_4g_hw_cb(enabled);
    }
}

/* 注册 4G 开关状态变化回调；保存后立即以当前 enabled 调用一次 cb */
void ui_4g_hw_register(ui_4g_changed_cb_t cb)
{
    s_4g_hw_cb = cb;
    if(cb != NULL) {
        cb(g_ui_4g_enabled);
    }
}

/* 读取 4G 是否开启 */
bool ui_4g_get(void)
{
    return g_ui_4g_enabled;
}

/* 设置 4G 开关；与当前相同返回 false；否则刷新 4G 页开关并通知硬件 */
bool ui_4g_set(bool enabled)
{
    if(enabled == g_ui_4g_enabled) {
        return false;
    }
    g_ui_4g_enabled = enabled;
    admin_4g_sync_switch_ui();
    ui_4g_apply_hw(enabled);
    return true;
}

/* 将当前 4G 开关状态再次推送给硬件（不修改 UI，用于通信重连等） */
void ui_4g_sync_to_hw(void)
{
    ui_4g_apply_hw(g_ui_4g_enabled);
}

/* ============================================================================
 * 运行常亮开关 — 硬件联动接口（仅在本文件 ui.c 说明与实现）
 * ============================================================================
 *
 * 【对外符号】硬件模块在自家 .c 顶部自行添加 extern 声明即可链接，无需改 ui.h：
 *
 *   typedef void (*ui_screen_run_always_on_changed_cb_t)(bool enabled);
 *
 *   extern void ui_screen_run_always_on_hw_register(ui_screen_run_always_on_changed_cb_t cb);
 *   extern bool ui_screen_run_always_on_get(void);
 *   extern bool ui_screen_run_always_on_set(bool enabled);
 *   extern void ui_screen_run_always_on_sync_to_hw(void);
 *
 * enabled == true  ：用户选择「运行过程中屏幕常亮」（管理员页开关 ON、上电默认、恢复出厂默认）
 * enabled == false ：用户选择关闭常亮（管理员页开关 OFF）
 *
 * 【本期 UI 行为】
 *   - 开关可切换并刷新样式，维护 g_ui_screen_run_always_on 状态
 *   - **暂不接入** ui_idle_screen_keeps_awake()，不改变运行页/熄屏/待机逻辑
 *   - 后续若需「关开关后运行中无操作可熄屏」，再在 ui_idle_screen_keeps_awake 中读取本接口
 *
 * 【职责划分】
 *   - UI：维护显示与用户选择；变更时调用 ui_screen_run_always_on_set。
 *   - 硬件：实现 cb，在 cb 内写寄存器、Flash 等；勿在 cb 内调用 LVGL API。
 *
 * 【推荐启动顺序（真机示例）】
 *
 *   static void on_run_always_on_changed(bool enabled)
 *   {
 *       holding_register[MB_HOLD_xxx] = enabled ? 1u : 0u;
 *   }
 *
 *   void board_init(void)
 *   {
 *       ui_screen_run_always_on_hw_register(on_run_always_on_changed);
 *       bool saved = nvm_read_run_always_on();
 *       ui_screen_run_always_on_set(saved);
 *       ui_init();
 *   }
 *
 * 【用户点开关】UI 内部 → ui_screen_run_always_on_set → 刷新 lv_switch → cb(enabled)。
 * 【恢复出厂】admin 恢复默认 → ui_screen_run_always_on_set(true) → cb(true)。
 * 【只读】if(ui_screen_run_always_on_get()) { ... }
 * 【通信重连】ui_screen_run_always_on_sync_to_hw() 再推送当前状态，不改动 UI。
 * ============================================================================ */

typedef void (*ui_screen_run_always_on_changed_cb_t)(bool enabled);

static bool g_ui_screen_run_always_on = true;  /* 上电默认开启：运行页常亮 */
static ui_screen_run_always_on_changed_cb_t s_screen_run_always_on_hw_cb;

static void admin_brightness_sync_switch_ui(void);  //屏幕亮度页：刷新常亮开关样式

/* 通知已注册的运行常亮硬件回调（未注册则无操作） */
static void ui_screen_run_always_on_apply_hw(bool enabled)
{
    if(s_screen_run_always_on_hw_cb != NULL) {
        s_screen_run_always_on_hw_cb(enabled);
    }
}

/* 注册运行常亮状态变化回调；保存后立即以当前 enabled 调用一次 cb */
void ui_screen_run_always_on_hw_register(ui_screen_run_always_on_changed_cb_t cb)
{
    s_screen_run_always_on_hw_cb = cb;
    if(cb != NULL) {
        cb(g_ui_screen_run_always_on);
    }
}

/* 读取运行常亮开关是否开启（true=开启，false=关闭） */
bool ui_screen_run_always_on_get(void)
{
    return g_ui_screen_run_always_on;
}

/* 设置运行常亮开关；与当前相同返回 false；否则刷新管理员页开关并通知硬件 */
bool ui_screen_run_always_on_set(bool enabled)
{
    if(enabled == g_ui_screen_run_always_on) {
        return false;
    }
    g_ui_screen_run_always_on = enabled;
    admin_brightness_sync_switch_ui();
    ui_screen_run_always_on_apply_hw(enabled);
    return true;
}

/* 将当前运行常亮开关状态再次推送给硬件（不修改 UI，用于通信重连等） */
void ui_screen_run_always_on_sync_to_hw(void)
{
    ui_screen_run_always_on_apply_hw(g_ui_screen_run_always_on);
}

/* ============================================================================
 * 屏幕亮度 — 硬件联动接口（仅在本文件 ui.c 说明与实现）
 * ============================================================================
 *
 * 【对外符号】硬件模块在自家 .c 顶部自行添加 extern 声明即可链接，无需改 ui.h：
 *
 *   typedef void (*ui_screen_brightness_changed_cb_t)(uint8_t percent);
 *
 *   extern void ui_screen_brightness_hw_register(ui_screen_brightness_changed_cb_t cb);
 *   extern uint8_t ui_screen_brightness_get(void);
 *   extern bool ui_screen_brightness_set(uint8_t percent);
 *   extern void ui_screen_brightness_sync_to_hw(void);
 *
 * percent：0~100，与管理员页亮度滑动条一致；100 档（步进 1）；上电默认 100，恢复出厂默认 100。
 *
 * 【职责划分】
 *   - UI：维护滑动条与用户选择；变更时调用 ui_screen_brightness_set。
 *   - 硬件：实现 cb，在 cb 内调背光 PWM、写寄存器、Flash 等；勿在 cb 内调用 LVGL。
 *
 * 【推荐启动顺序（真机示例）】
 *
 *   static void on_screen_brightness_changed(uint8_t percent)
 *   {
 *       backlight_pwm_set(percent);   // 0~100 映射到实际 PWM
 *   }
 *
 *   void board_init(void)
 *   {
 *       ui_screen_brightness_hw_register(on_screen_brightness_changed);
 *       uint8_t saved = nvm_read_brightness();
 *       ui_screen_brightness_set(saved);
 *       ui_init();
 *   }
 *
 * 【用户拖滑动条】UI 内部 → ui_screen_brightness_set → 刷新滑条 → cb(percent)。
 * 【恢复出厂】admin 恢复默认 → ui_screen_brightness_set(100) → cb(100)。
 * 【只读】uint8_t p = ui_screen_brightness_get();
 * 【通信重连】ui_screen_brightness_sync_to_hw() 用当前 UI 值再调一次 cb，不重绘界面。
 * 【PC 仿真】未 register 时 get/set 仍有效，仅无硬件回调。
 * ============================================================================ */

typedef void (*ui_screen_brightness_changed_cb_t)(uint8_t percent);

static uint8_t g_ui_screen_brightness = 100;  /* 上电默认 100% */
static ui_screen_brightness_changed_cb_t s_screen_brightness_hw_cb;
static bool g_admin_brightness_ui_loading;    /* sync 滑条时抑制 VALUE_CHANGED 回调 */

static void admin_brightness_sync_slider_ui(void);  //屏幕亮度页：刷新亮度滑动条位置

/* 通知已注册的屏幕亮度硬件回调（未注册则无操作） */
static void ui_screen_brightness_apply_hw(uint8_t percent)
{
    if(s_screen_brightness_hw_cb != NULL) {
        s_screen_brightness_hw_cb(percent);
    }
}

/* 注册屏幕亮度变化回调；保存后立即以当前 percent 调用一次 cb */
void ui_screen_brightness_hw_register(ui_screen_brightness_changed_cb_t cb)
{
    s_screen_brightness_hw_cb = cb;
    if(cb != NULL) {
        cb(g_ui_screen_brightness);
    }
}

/* 读取当前屏幕亮度百分比（0~100） */
uint8_t ui_screen_brightness_get(void)
{
    return g_ui_screen_brightness;
}

#define UI_SCREEN_BRIGHTNESS_STEP  1  /* 亮度滑条步进值（100 档：0~100） */

/* 亮度百分比对齐到步进 */
static uint8_t ui_screen_brightness_snap(uint8_t percent)
{
    uint8_t snapped = (uint8_t)(((percent + (UI_SCREEN_BRIGHTNESS_STEP / 2u)) / UI_SCREEN_BRIGHTNESS_STEP) *
                                UI_SCREEN_BRIGHTNESS_STEP);
    if(snapped > 100u) {
        snapped = 100u;
    }
    return snapped;
}

/* 设置屏幕亮度；钳位 0~100 并按步进对齐；与当前相同返回 false；否则刷新滑条并通知硬件 */
bool ui_screen_brightness_set(uint8_t percent)
{
    percent = ui_screen_brightness_snap(percent);
    if(percent == g_ui_screen_brightness) {
        return false;
    }
    g_ui_screen_brightness = percent;
    admin_brightness_sync_slider_ui();
    ui_screen_brightness_apply_hw(percent);
    return true;
}

/* 将当前屏幕亮度再次推送给硬件（不修改 UI，用于通信重连等） */
void ui_screen_brightness_sync_to_hw(void)
{
    ui_screen_brightness_apply_hw(g_ui_screen_brightness);
}

/* ============================================================================
 * 触控声音开关 — 硬件联动接口（仅在本文件 ui.c 说明与实现）
 * ============================================================================
 *
 * 【对外符号】硬件模块在自家 .c 顶部自行添加 extern 声明即可链接，无需改 ui.h：
 *
 *   typedef void (*ui_touch_sound_changed_cb_t)(bool enabled);
 *
 *   extern void ui_touch_sound_hw_register(ui_touch_sound_changed_cb_t cb);
 *   extern bool ui_touch_sound_get(void);
 *   extern bool ui_touch_sound_set(bool enabled);
 *   extern void ui_touch_sound_sync_to_hw(void);
 *
 * enabled == true  ：触控声音开启（管理员页开关 ON、上电默认、恢复出厂默认）
 * enabled == false ：触控声音关闭
 *
 * 【本期 UI 行为】仅维护开关显示与状态，不触发实际蜂鸣/触控音效。
 *
 * 【用户点开关】UI → ui_touch_sound_set → 刷新 lv_switch → cb(enabled)。
 * 【恢复出厂】admin_factory_run_restore → ui_touch_sound_set(true) → cb(true)。
 * 【只读】if(ui_touch_sound_get()) { ... }
 * 【通信重连】ui_touch_sound_sync_to_hw() 再推送当前状态，不改动 UI。
 * ============================================================================ */

typedef void (*ui_touch_sound_changed_cb_t)(bool enabled);

static bool g_ui_touch_sound = true;  /* 上电默认开启 */
static ui_touch_sound_changed_cb_t s_touch_sound_hw_cb;

static void admin_sound_sync_touch_switch_ui(void);  //声音控制页：刷新触控声音开关
static void admin_sound_sync_voice_broadcast_switch_ui(void);  //声音控制页：刷新声音播报开关
static void admin_sound_sync_volume_slider_ui(void);  //声音控制页：刷新音量滑条
static void admin_sound_sync_touch_sound_volume_slider_ui(void);  //声音控制页：刷新触控声音滑条

/* 通知已注册的触控声音硬件回调（未注册则无操作） */
static void ui_touch_sound_apply_hw(bool enabled)
{
    if(s_touch_sound_hw_cb != NULL) {
        s_touch_sound_hw_cb(enabled);
    }
}

/* 注册触控声音开关变化回调；保存后立即以当前 enabled 调用一次 cb */
void ui_touch_sound_hw_register(ui_touch_sound_changed_cb_t cb)
{
    s_touch_sound_hw_cb = cb;
    if(cb != NULL) {
        cb(g_ui_touch_sound);
    }
}

/* 读取触控声音开关是否开启（true=开启，false=关闭） */
bool ui_touch_sound_get(void)
{
    return g_ui_touch_sound;
}

/* 设置触控声音开关；与当前相同返回 false；否则刷新管理员页开关并通知硬件 */
bool ui_touch_sound_set(bool enabled)
{
    if(enabled == g_ui_touch_sound) {
        return false;
    }
    g_ui_touch_sound = enabled;
    admin_sound_sync_touch_switch_ui();
    ui_touch_sound_apply_hw(enabled);
    return true;
}

/* 将当前触控声音开关状态再次推送给硬件（不修改 UI，用于通信重连等） */
void ui_touch_sound_sync_to_hw(void)
{
    ui_touch_sound_apply_hw(g_ui_touch_sound);
}

/* ============================================================================
 * 声音播报开关 — 硬件联动接口（仅在本文件 ui.c 说明与实现）
 * ============================================================================
 *
 *   typedef void (*ui_voice_broadcast_changed_cb_t)(bool enabled);
 *
 *   extern void ui_voice_broadcast_hw_register(ui_voice_broadcast_changed_cb_t cb);
 *   extern bool ui_voice_broadcast_get(void);
 *   extern bool ui_voice_broadcast_set(bool enabled);
 *   extern void ui_voice_broadcast_sync_to_hw(void);
 *
 * enabled == true  ：声音播报开启（上电默认、恢复出厂默认）
 * enabled == false ：声音播报关闭
 *
 * 【本期 UI 行为】仅维护开关显示与状态，不播放语音。
 * 【恢复出厂】ui_voice_broadcast_set(true)。
 * ============================================================================ */

typedef void (*ui_voice_broadcast_changed_cb_t)(bool enabled);

static bool g_ui_voice_broadcast = true;
static ui_voice_broadcast_changed_cb_t s_voice_broadcast_hw_cb;

/* 通知已注册的声音播报硬件回调（未注册则无操作） */
static void ui_voice_broadcast_apply_hw(bool enabled)
{
    if(s_voice_broadcast_hw_cb != NULL) {
        s_voice_broadcast_hw_cb(enabled);
    }
}

/* 注册声音播报开关变化回调；保存后立即以当前 enabled 调用一次 cb */
void ui_voice_broadcast_hw_register(ui_voice_broadcast_changed_cb_t cb)
{
    s_voice_broadcast_hw_cb = cb;
    if(cb != NULL) {
        cb(g_ui_voice_broadcast);
    }
}

/* 读取声音播报开关是否开启（true=开启，false=关闭） */
bool ui_voice_broadcast_get(void)
{
    return g_ui_voice_broadcast;
}

/* 设置声音播报开关；与当前相同返回 false；否则刷新管理员页开关并通知硬件 */
bool ui_voice_broadcast_set(bool enabled)
{
    if(enabled == g_ui_voice_broadcast) {
        return false;
    }
    g_ui_voice_broadcast = enabled;
    admin_sound_sync_voice_broadcast_switch_ui();
    ui_voice_broadcast_apply_hw(enabled);
    return true;
}

/* 将当前声音播报开关状态再次推送给硬件（不修改 UI，用于通信重连等） */
void ui_voice_broadcast_sync_to_hw(void)
{
    ui_voice_broadcast_apply_hw(g_ui_voice_broadcast);
}

/* ============================================================================
 * 音量滑条 — 硬件联动接口（仅在本文件 ui.c 说明与实现）
 * ============================================================================
 *
 *   typedef void (*ui_sound_volume_changed_cb_t)(uint8_t percent);
 *
 *   extern void ui_sound_volume_hw_register(ui_sound_volume_changed_cb_t cb);
 *   extern uint8_t ui_sound_volume_get(void);
 *   extern bool ui_sound_volume_set(uint8_t percent);
 *   extern void ui_sound_volume_sync_to_hw(void);
 *
 * percent：0~100，100 档（步进 1）；上电默认 100，恢复出厂默认 100。
 * 【用户拖滑条】UI → ui_sound_volume_set → 刷新滑条 → cb(percent)。
 * ============================================================================ */

typedef void (*ui_sound_volume_changed_cb_t)(uint8_t percent);

static uint8_t g_ui_sound_volume = 100;
static ui_sound_volume_changed_cb_t s_sound_volume_hw_cb;
static bool g_admin_sound_volume_ui_loading;

/* 通知已注册的音量滑条硬件回调（未注册则无操作） */
static void ui_sound_volume_apply_hw(uint8_t percent)
{
    if(s_sound_volume_hw_cb != NULL) {
        s_sound_volume_hw_cb(percent);
    }
}

/* 注册音量滑条变化回调；保存后立即以当前 percent 调用一次 cb */
void ui_sound_volume_hw_register(ui_sound_volume_changed_cb_t cb)
{
    s_sound_volume_hw_cb = cb;
    if(cb != NULL) {
        cb(g_ui_sound_volume);
    }
}

/* 读取当前音量百分比（0~100） */
uint8_t ui_sound_volume_get(void)
{
    return g_ui_sound_volume;
}

#define UI_SOUND_VOLUME_STEP  1  /* 音量滑条步进值（100 档：0~100，同亮度页） */

/* 音量百分比对齐到步进 */
static uint8_t ui_sound_volume_snap(uint8_t percent)
{
    uint8_t snapped = (uint8_t)(((percent + (UI_SOUND_VOLUME_STEP / 2u)) / UI_SOUND_VOLUME_STEP) *
                                UI_SOUND_VOLUME_STEP);
    if(snapped > 100u) {
        snapped = 100u;
    }
    return snapped;
}

/* 设置音量；钳位 0~100 并按步进对齐；与当前相同返回 false；否则刷新滑条并通知硬件 */
bool ui_sound_volume_set(uint8_t percent)
{
    percent = ui_sound_volume_snap(percent);
    if(percent == g_ui_sound_volume) {
        return false;
    }
    g_ui_sound_volume = percent;
    admin_sound_sync_volume_slider_ui();
    ui_sound_volume_apply_hw(percent);
    return true;
}

/* 将当前音量再次推送给硬件（不修改 UI，用于通信重连等） */
void ui_sound_volume_sync_to_hw(void)
{
    ui_sound_volume_apply_hw(g_ui_sound_volume);
}

/* ============================================================================
 * 触控声音滑条 — 硬件联动接口（仅在本文件 ui.c 说明与实现）
 * ============================================================================
 *
 *   typedef void (*ui_touch_sound_volume_changed_cb_t)(uint8_t percent);
 *
 *   extern void ui_touch_sound_volume_hw_register(ui_touch_sound_volume_changed_cb_t cb);
 *   extern uint8_t ui_touch_sound_volume_get(void);
 *   extern bool ui_touch_sound_volume_set(uint8_t percent);
 *   extern void ui_touch_sound_volume_sync_to_hw(void);
 *
 * percent：0~100，100 档（步进 1）；上电默认 100，恢复出厂默认 100。
 * 与 ui_touch_sound_*（bool 开关）区分：本组为触控音强度/音量百分比。
 * ============================================================================ */

typedef void (*ui_touch_sound_volume_changed_cb_t)(uint8_t percent);

static uint8_t g_ui_touch_sound_volume = 100;
static ui_touch_sound_volume_changed_cb_t s_touch_sound_volume_hw_cb;
static bool g_admin_touch_sound_volume_ui_loading;

/* 通知已注册的触控声音滑条硬件回调（未注册则无操作） */
static void ui_touch_sound_volume_apply_hw(uint8_t percent)
{
    if(s_touch_sound_volume_hw_cb != NULL) {
        s_touch_sound_volume_hw_cb(percent);
    }
}

/* 注册触控声音滑条变化回调；保存后立即以当前 percent 调用一次 cb */
void ui_touch_sound_volume_hw_register(ui_touch_sound_volume_changed_cb_t cb)
{
    s_touch_sound_volume_hw_cb = cb;
    if(cb != NULL) {
        cb(g_ui_touch_sound_volume);
    }
}

/* 读取当前触控声音滑条百分比（0~100） */
uint8_t ui_touch_sound_volume_get(void)
{
    return g_ui_touch_sound_volume;
}

#define UI_TOUCH_SOUND_VOLUME_STEP  1  /* 触控声音滑条步进值（100 档：0~100，同亮度页） */

/* 触控声音百分比对齐到步进 */
static uint8_t ui_touch_sound_volume_snap(uint8_t percent)
{
    uint8_t snapped = (uint8_t)(((percent + (UI_TOUCH_SOUND_VOLUME_STEP / 2u)) / UI_TOUCH_SOUND_VOLUME_STEP) *
                                UI_TOUCH_SOUND_VOLUME_STEP);
    if(snapped > 100u) {
        snapped = 100u;
    }
    return snapped;
}

/* 设置触控声音滑条；钳位 0~100 并按步进对齐；与当前相同返回 false；否则刷新滑条并通知硬件 */
bool ui_touch_sound_volume_set(uint8_t percent)
{
    percent = ui_touch_sound_volume_snap(percent);
    if(percent == g_ui_touch_sound_volume) {
        return false;
    }
    g_ui_touch_sound_volume = percent;
    admin_sound_sync_touch_sound_volume_slider_ui();
    ui_touch_sound_volume_apply_hw(percent);
    return true;
}

/* 将当前触控声音滑条值再次推送给硬件（不修改 UI，用于通信重连等） */
void ui_touch_sound_volume_sync_to_hw(void)
{
    ui_touch_sound_volume_apply_hw(g_ui_touch_sound_volume);
}












/* ========== 语言切换（中/英）========== */
typedef enum {
    UI_LANG_ZH = 0,
    UI_LANG_EN = 1,
} ui_lang_t;

typedef enum {
    STR_PROG_DAWU,
    STR_PROG_DANTUO,
    STR_PROG_BIAOZHUN,
    STR_PROG_TONGZIJIE,
    STR_PROG_KUAIXI,
    STR_LANG_INDICATOR,
    STR_PAY_HINT,
    STR_PAY_HINT_ALIPAY,
    STR_PAY_HINT_WECHAT,
    STR_PAY_HINT_ALIPAY_1,
    STR_PAY_HINT_ALIPAY_2,
    STR_PAY_HINT_WECHAT_1,
    STR_PAY_HINT_WECHAT_2,
    STR_PAY_DONE,
    STR_RUN_WASHING,
    STR_RUN_RINSING,
    STR_RUN_SPINNING,
    STR_RUN_STAGES,
    STR_RUN_STAGES_SPIN_ONLY,
    STR_RUN_STAGES_RINSE_SPIN,
    STR_END_TITLE,
    STR_END_HINT,
    STR_ALARM_SOFTENER_LOW,
    STR_ALARM_DETERGENT_LOW,
    STR_ALARM_E1_TITLE, STR_ALARM_E1_LINE1, STR_ALARM_E1_LINE2,
    STR_ALARM_E2_TITLE, STR_ALARM_E2_LINE1, STR_ALARM_E2_LINE2,
    STR_ALARM_E3_TITLE, STR_ALARM_E3_LINE1, STR_ALARM_E3_LINE2,
    STR_ALARM_E4_TITLE, STR_ALARM_E4_LINE1, STR_ALARM_E4_LINE2,
    STR_ALARM_E5_TITLE, STR_ALARM_E5_LINE1, STR_ALARM_E5_LINE2,
    STR_ALARM_E6_TITLE, STR_ALARM_E6_LINE1, STR_ALARM_E6_LINE2,
    STR_ALARM_E7_TITLE, STR_ALARM_E7_LINE1, STR_ALARM_E7_LINE2,
    STR_ALARM_E8_TITLE, STR_ALARM_E8_LINE1, STR_ALARM_E8_LINE2,
    STR_ALARM_E9_TITLE, STR_ALARM_E9_LINE1, STR_ALARM_E9_LINE2,
    STR_ALARM_E10_TITLE, STR_ALARM_E10_LINE1, STR_ALARM_E10_LINE2,
    STR_ALARM_E11_TITLE, STR_ALARM_E11_LINE1, STR_ALARM_E11_LINE2,
    STR_ALARM_E12_TITLE, STR_ALARM_E12_LINE1, STR_ALARM_E12_LINE2,
    STR_ALARM_E13_TITLE, STR_ALARM_E13_LINE1, STR_ALARM_E13_LINE2,
    STR_ALARM_E14_TITLE, STR_ALARM_E14_LINE1, STR_ALARM_E14_LINE2,
    STR_ALARM_FAULT_CALL,
    STR_ALARM_FAULT_SERVICE,
    STR_ALARM_FAULT_PHONE,
    STR_ADMIN_MENU_TITLE,
    STR_ADMIN_LANG_TITLE,
    STR_ADMIN_LANG_HINT,
    STR_ADMIN_LANG_BTN_ZH,
    STR_ADMIN_LANG_BTN_EN,
    STR_ADMIN_M1_MACHINE_ID,
    STR_ADMIN_M1_PROGRAM,
    STR_ADMIN_M1_BRIGHTNESS,
    STR_ADMIN_M1_SOUND,
    STR_ADMIN_M1_LANGUAGE,
    STR_ADMIN_M1_STANDBY,
    STR_ADMIN_M1_FACTORY_RESET,
    STR_ADMIN_M1_CONTACT,
    STR_ADMIN_M2_AUTO_DISPENSE,
    STR_ADMIN_M2_FRESH_AIR,
    STR_ADMIN_M2_UPGRADE,
    STR_ADMIN_M2_NETWORK,
    STR_ADMIN_M2_DATA,
    STR_ADMIN_M2_PAYMENT,
    STR_ADMIN_M2_PASSWORD,
    STR_ADMIN_M2_VENDOR_MAINT,
    STR_VENDOR_SERIAL_HINT,
    STR_VENDOR_SELF_CHECK,
    STR_VENDOR_SELF_LEARN,
    STR_SELF_CHECK_TITLE,
    STR_SELF_CHECK_DONE,
    STR_CYCLE_TITLE,
    STR_CYCLE_RUN_COUNT_FMT,
    STR_BTN_CONFIRM,
    STR_BTN_CANCEL,
    STR_BTN_OK,
    STR_BTN_RESET,
    STR_BTN_ON,
    STR_BTN_OFF,
    STR_BTN_QUERY,
    STR_PWD_ENTER_ADMIN,
    STR_PWD_WRONG_RETRY,
    STR_PWD_ENTER_OLD,
    STR_PWD_ENTER_NEW,
    STR_PWD_ENTER_NEW_AGAIN,
    STR_PWD_WRONG_REENTER,
    STR_PWD_MISMATCH,
    STR_PWD_CHANGE_OK,
    STR_MACHINE_ID_TITLE,
    STR_MACHINE_ID_CUR_NONE,
    STR_MACHINE_ID_CUR_FMT,
    STR_PROG_FIELD_PRICE,
    STR_PROG_FIELD_DRY_TEMP,
    STR_PROG_FIELD_INIT_DRY,
    STR_PROG_FIELD_COOL_TIME,
    STR_PROG_FIELD_ADD_COUNT,
    STR_PROG_FIELD_ADD_TIME,
    STR_PROG_FIELD_ADD_PRICE,
    STR_PROG_WATER_SMART,
    STR_BRIGHTNESS_LINE1,
    STR_BRIGHTNESS_LINE2,
    STR_SOUND_TOUCH,
    STR_SOUND_VOICE,
    STR_DORMANCY_TITLE,
    STR_DORMANCY_CUR_FMT,
    STR_DORMANCY_ROLLER,
    STR_DORM_1MIN,
    STR_DORM_2MIN,
    STR_DORM_5MIN,
    STR_DORM_10MIN,
    STR_DORM_15MIN,
    STR_DORM_30MIN,
    STR_DORM_1H,
    STR_DORM_2H,
    STR_DORM_NO_SLEEP,
    STR_DORM_TIME_SET,
    STR_DORM_TIME_HINT,
    STR_DORM_NO_SLEEP_HINT,
    STR_DORM_TIME_PREFIX,
    STR_DORM_TIME_SUFFIX,
    STR_FACTORY_CONFIRM_Q,
    STR_FACTORY_CONFIRM_HINT,
    STR_FACTORY_RESTORING,
    STR_FACTORY_DONE,
    STR_CONTACT_HOTLINE,
    STR_CONTACT_SLOGAN,
    STR_AUTO_DISP_LINE1,
    STR_FRESH_AIR_LINE1,
    STR_UPGRADE_CONFIRM_Q,
    STR_UPGRADE_IN_PROGRESS,
    STR_UPGRADE_LATEST,
    STR_PAYMENT_METHOD,
    STR_PAYMENT_TIMEOUT,
    STR_PAYMENT_ORDER,
    STR_PAYMENT_ALIPAY,
    STR_PAYMENT_WECHAT,
    STR_PAYMENT_ORDER_HINT,
    STR_PAYMENT_TIMEOUT_ROLLER,
    STR_PAYMENT_TIME_PREFIX,
    STR_PAYMENT_TIME_SUFFIX,
    STR_ORDER_STATUS_RUNNING,
    STR_ORDER_STATUS_DONE,
    STR_ORDER_DETAIL_HDR,
    STR_ORDER_ITEMS_FMT,
    STR_ORDER_PAID_FMT,
    STR_ORDER_TIME_START,
    STR_ORDER_TIME_END,
    STR_ORDER_INVOICE,
    STR_ORDER_DETAIL_BTN,
    STR_ORDER_TOTAL_FMT,
    STR_DATA_UPLOAD_HDR,
    STR_DATA_STRATEGY_HDR,
    STR_DATA_BASIC,
    STR_DATA_SENSOR,
    STR_DATA_FAULT,
    STR_DATA_AUTO_DISPENSE,
    STR_DATA_PAYMENT_ORDER,
    STR_DATA_USER_OP,
    STR_DATA_DEVICE,
    STR_DATA_REALTIME,
    STR_DATA_SCHEDULED,
    STR_DATA_WIFI_ONLY,
    STR_DATA_4G_ONLY,
    STR_DATA_FORBIDDEN,
    STR_WIFI_SETTINGS,
    STR_WIFI_PROMPT,
    STR_WIFI_PROVISIONING,
    STR_WIFI_SUCCESS,
    STR_WIFI_FAIL,
    STR_4G_SETTINGS,
    STR_4G_PROMPT,
    STR_4G_PROVISIONING,
    STR_4G_SUCCESS,
    STR_4G_FAIL,
    STR_MACHINE_ID_SUCCESS,
    STR_ADD_TIME_TITLE,
    STR_ADD_TIME_TOTAL_PRICE,
    STR_ADD_TIME_TOTAL_TIME,
    STR_ADD_TIME_BTN_UP,
    STR_ADD_TIME_BTN_DOWN,
    STR_COUNT
} ui_str_id_t;

#define UI_LANG_BIND_MAX  128u

typedef struct {
    lv_obj_t * lbl;
    ui_str_id_t id;
} ui_lang_bind_t;

static ui_lang_t g_ui_lang = UI_LANG_ZH;
static ui_lang_bind_t g_lang_binds[UI_LANG_BIND_MAX];
static unsigned g_lang_bind_count;
static ui_str_id_t g_admin_pwd_err_id = STR_COUNT;
static ui_str_id_t g_admin_pwd_chg_old_err_id = STR_COUNT;
static ui_str_id_t g_admin_pwd_chg_new_msg_id = STR_COUNT;

/* 中英对照字符串表：[语言][字符串ID]；修改文案只需改此表 */
static const char * const g_ui_strings[2][STR_COUNT] = {
    [UI_LANG_ZH] = {
        [STR_PROG_DAWU]      = "低温",           /* 程序0：轮播/运行页程序名 */
        [STR_PROG_DANTUO]    = "中温",
        [STR_PROG_BIAOZHUN]  = "高温",
        [STR_PROG_TONGZIJIE] = "冷风",
        [STR_PROG_KUAIXI]    = "风自洁",
        [STR_LANG_INDICATOR] = "China",          /* 主页第4列：当前为中文界面时的指示文字 */
        [STR_PAY_HINT]       = "请扫描屏幕上二维码\n支付完成机器自动运行",
        [STR_PAY_HINT_ALIPAY] = "请扫描屏幕上支付宝支付二维码\n支付完成机器自动运行",
        [STR_PAY_HINT_WECHAT] = "请扫描屏幕上微信支付二维码\n支付完成机器自动运行",
        [STR_PAY_HINT_ALIPAY_1] = "请扫描屏幕上支付宝支付二维码",
        [STR_PAY_HINT_ALIPAY_2] = "支付完成机器自动运行",
        [STR_PAY_HINT_WECHAT_1] = "请扫描屏幕上微信支付二维码",
        [STR_PAY_HINT_WECHAT_2] = "支付完成机器自动运行",
        [STR_PAY_DONE]       = "支付完成",
        [STR_RUN_WASHING]    = "烘干中...",
        [STR_RUN_RINSING]    = "漂洗中...",
        [STR_RUN_SPINNING]   = "打冷风中...",
        [STR_RUN_STAGES]     = "烘干        打冷风",
        [STR_RUN_STAGES_SPIN_ONLY]  = "打冷风",
        [STR_RUN_STAGES_RINSE_SPIN] = "烘干",
        [STR_END_TITLE]      = "烘干完成",
        [STR_END_HINT]       = "请及时取衣",
        [STR_ALARM_SOFTENER_LOW] = "检测到柔顺剂不足，请及时添加",
        [STR_ALARM_DETERGENT_LOW] = "检测到洗涤剂不足，请及时添加",
        [STR_ALARM_E1_TITLE]  = "门锁异常",
        [STR_ALARM_E1_LINE1]  = "请检查门是否关好，关紧门锁",
        [STR_ALARM_E1_LINE2]  = "或门锁与主控板连线是否松动",
        [STR_ALARM_E2_TITLE]  = "加热器过热",
        [STR_ALARM_E2_LINE1]  = "请关机，检查加热器是否正常",
        [STR_ALARM_E2_LINE2]  = "",
        [STR_ALARM_E3_TITLE]  = "加热器异常过热",
        [STR_ALARM_E3_LINE1]  = "请关机，检查加热器和安全温控开关",
        [STR_ALARM_E3_LINE2]  = "",
        [STR_ALARM_E4_TITLE]  = "出风口过热",
        [STR_ALARM_E4_LINE1]  = "请关机，检查加热器是否正常工作，出风口是否通畅",
        [STR_ALARM_E4_LINE2]  = "",
        [STR_ALARM_E5_TITLE]  = "电机异常",
        [STR_ALARM_E5_LINE1]  = "请检查电机与主控板连线是否正常",
        [STR_ALARM_E5_LINE2]  = "",
        [STR_ALARM_E6_TITLE]  = "显示屏通讯故障",
        [STR_ALARM_E6_LINE1]  = "请检查主控板与显示屏连线",
        [STR_ALARM_E6_LINE2]  = "",
        [STR_ALARM_E7_TITLE]  = "温度传感器异常",
        [STR_ALARM_E7_LINE1]  = "请关机，检查温度传感器是否已坏",
        [STR_ALARM_E7_LINE2]  = "或与主控板接线是否连接正常",
        [STR_ALARM_E8_TITLE]  = "皮带轮微动开关异常",
        [STR_ALARM_E8_LINE1]  = "请检查内筒皮带是否正常",
        [STR_ALARM_E8_LINE2]  = "或微动开关接线是否正常",
        [STR_ALARM_E9_TITLE]  = "电机通讯故障",
        [STR_ALARM_E9_LINE1]  = "请重新运行机器",
        [STR_ALARM_E9_LINE2]  = "并检查电机驱动器与主控板的通讯连线是否正常",
        [STR_ALARM_E10_TITLE] = "驱动板通讯异常",
        [STR_ALARM_E10_LINE1] = "请重新运行机器并检查驱动板的连线",
        [STR_ALARM_E10_LINE2] = "",
        [STR_ALARM_E11_TITLE] = "电脑主控板通讯异常",
        [STR_ALARM_E11_LINE1] = "请重新运行机器并检查电源板的连线",
        [STR_ALARM_E11_LINE2] = "",
        [STR_ALARM_E12_TITLE] = "物联网模块配置异常",
        [STR_ALARM_E12_LINE1] = "请断电并重新进行网络配置",
        [STR_ALARM_E12_LINE2] = "",
        [STR_ALARM_E13_TITLE] = "支付通讯异常",
        [STR_ALARM_E13_LINE1] = "请检查网络，并重试支付",
        [STR_ALARM_E13_LINE2] = "",
        [STR_ALARM_E14_TITLE] = "支付超时",
        [STR_ALARM_E14_LINE1] = "请重新选程序并支付",
        [STR_ALARM_E14_LINE2] = "",
        [STR_ALARM_FAULT_CALL]    = "如果仍未解决请拨打:",
        [STR_ALARM_FAULT_SERVICE] = "售后工程师为您提供专业服务。",
        [STR_ALARM_FAULT_PHONE]   = "400-999-999",
        [STR_ADMIN_MENU_TITLE]       = "管理员设置",
        [STR_ADMIN_LANG_TITLE]       = "语言设置",
        [STR_ADMIN_LANG_HINT]        = "系统语言切换",
        [STR_ADMIN_LANG_BTN_ZH]      = "中文",
        [STR_ADMIN_LANG_BTN_EN]      = "英文",
        [STR_ADMIN_M1_MACHINE_ID]    = "机器ID设置",
        [STR_ADMIN_M1_PROGRAM]       = "程序设置",
        [STR_ADMIN_M1_BRIGHTNESS]    = "屏幕亮度",
        [STR_ADMIN_M1_SOUND]         = "声音控制",
        [STR_ADMIN_M1_LANGUAGE]      = "语言设置",
        [STR_ADMIN_M1_STANDBY]       = "待机时间",
        [STR_ADMIN_M1_FACTORY_RESET] = "恢复默认",
        [STR_ADMIN_M1_CONTACT]       = "联系我们",
        [STR_ADMIN_M2_AUTO_DISPENSE] = "防缠绕",
        [STR_ADMIN_M2_FRESH_AIR]     = "新风护理",
        [STR_ADMIN_M2_UPGRADE]       = "系统升级",
        [STR_ADMIN_M2_NETWORK]       = "网络设置",
        [STR_ADMIN_M2_DATA]          = "数据设置",
        [STR_ADMIN_M2_PAYMENT]       = "支付设置",
        [STR_ADMIN_M2_PASSWORD]      = "密码修改",
        [STR_ADMIN_M2_VENDOR_MAINT]  = "厂商维护",
        [STR_VENDOR_SERIAL_HINT]      = "请输入特殊出厂序列号",
        [STR_VENDOR_SELF_CHECK]     = "1、自检程序",
        [STR_VENDOR_SELF_LEARN]     = "2、循环程序",
        [STR_SELF_CHECK_TITLE]      = "自检程序",
        [STR_SELF_CHECK_DONE]       = "自检完成",
        [STR_CYCLE_TITLE]           = "循环程序",
        [STR_CYCLE_RUN_COUNT_FMT]   = "已完成：%u 次",
        [STR_BTN_CONFIRM]            = "确认",
        [STR_BTN_CANCEL]             = "取消",
        [STR_BTN_OK]                 = "确定",
        [STR_BTN_RESET]              = "重置",
        [STR_BTN_ON]                 = "开启",
        [STR_BTN_OFF]                = "关闭",
        [STR_BTN_QUERY]              = "查询",
        [STR_PWD_ENTER_ADMIN]        = "请输入管理员密码",
        [STR_PWD_WRONG_RETRY]        = "密码错误，请稍后再试",
        [STR_PWD_ENTER_OLD]          = "请输入原密码",
        [STR_PWD_ENTER_NEW]          = "请输入新密码",
        [STR_PWD_ENTER_NEW_AGAIN]    = "再次输入新密码：",
        [STR_PWD_WRONG_REENTER]      = "密码错误，请重新输入",
        [STR_PWD_MISMATCH]           = "两次输入不一致，请重新输入",
        [STR_PWD_CHANGE_OK]          = "密码修改成功",
        [STR_MACHINE_ID_TITLE]       = "机器 ID 设置",
        [STR_MACHINE_ID_CUR_NONE]    = "当前 ID：未配置",
        [STR_MACHINE_ID_CUR_FMT]     = "当前 ID：%06u",
        [STR_PROG_FIELD_PRICE]       = "程序金额",
        [STR_PROG_FIELD_DRY_TEMP]    = "烘干温度",
        [STR_PROG_FIELD_INIT_DRY]    = "初始烘干时间",
        [STR_PROG_FIELD_COOL_TIME]   = "冷却时间",
        [STR_PROG_FIELD_ADD_COUNT]   = "追加次数",
        [STR_PROG_FIELD_ADD_TIME]    = "追加时间",
        [STR_PROG_FIELD_ADD_PRICE]   = "追加时间金额",
        [STR_PROG_WATER_SMART]       = "智能设定",
        [STR_BRIGHTNESS_LINE1]       = "运行过程中屏幕处于常亮状态",
        [STR_BRIGHTNESS_LINE2]       = "如若关闭，则在运行过程中无操作自动熄灭屏幕",
        [STR_SOUND_TOUCH]            = "触控声音：",
        [STR_SOUND_VOICE]            = "声音播报：",
        [STR_DORMANCY_TITLE]         = "待机时间设置",
        [STR_DORMANCY_CUR_FMT]       = "当前：%s",
        [STR_DORMANCY_ROLLER]        = "1分钟\n2分钟\n5分钟\n10分钟\n15分钟\n30分钟\n1小时\n2小时\n不熄屏",
        [STR_DORM_1MIN]              = "1分钟",
        [STR_DORM_2MIN]              = "2分钟",
        [STR_DORM_5MIN]              = "5分钟",
        [STR_DORM_10MIN]             = "10分钟",
        [STR_DORM_15MIN]             = "15分钟",
        [STR_DORM_30MIN]             = "30分钟",
        [STR_DORM_1H]                = "1小时",
        [STR_DORM_2H]                = "2小时",
        [STR_DORM_NO_SLEEP]          = "不熄屏",
        [STR_DORM_TIME_SET]          = "时间设置",
        [STR_DORM_TIME_HINT]         = "可设置屏幕进入休眠状态的时间，默认时间5分钟",
        [STR_DORM_NO_SLEEP_HINT]     = "屏幕一直处于常亮状态",
        [STR_DORM_TIME_PREFIX]       = "机器将在",
        [STR_DORM_TIME_SUFFIX]       = "后熄屏",
        [STR_FACTORY_CONFIRM_Q]      = "是否需要恢复默认设置？",
        [STR_FACTORY_CONFIRM_HINT]   = "(所有设置都将恢复出厂设置)",
        [STR_FACTORY_RESTORING]      = "正在恢复出厂设置......",
        [STR_FACTORY_DONE]           = "所有设置已恢复出厂设置",
        [STR_CONTACT_HOTLINE]        = "24小时服务热线：400-999-999",
        [STR_CONTACT_SLOGAN]         = "小鸭专属热线将为您提供优质的服务体验！",
        [STR_AUTO_DISP_LINE1]        = "开启后，有效避免衣物打结，提升烘干均匀度",
        [STR_FRESH_AIR_LINE1]        = "开启后，烘干完成后内筒间歇性转动",
        [STR_UPGRADE_CONFIRM_Q]      = "是否将系统升级为最新版本",
        [STR_UPGRADE_IN_PROGRESS]    = "正在进行系统升级......",
        [STR_UPGRADE_LATEST]         = "系统已是最新版本",
        [STR_PAYMENT_METHOD]         = "支付方式",
        [STR_PAYMENT_TIMEOUT]        = "支付超时",
        [STR_PAYMENT_ORDER]          = "订单查询",
        [STR_PAYMENT_ALIPAY]         = "支付宝支付",
        [STR_PAYMENT_WECHAT]         = "微信支付",
        [STR_PAYMENT_ORDER_HINT]     = "本机可查询最近100条订单",
        [STR_PAYMENT_TIMEOUT_ROLLER] = "60秒\n120秒\n180秒\n240秒\n300秒",
        [STR_PAYMENT_TIME_PREFIX]    = "支付将在",
        [STR_PAYMENT_TIME_SUFFIX]    = "分钟后取消",
        [STR_ORDER_STATUS_RUNNING]   = "进行中",
        [STR_ORDER_STATUS_DONE]      = "已完成",
        [STR_ORDER_DETAIL_HDR]       = "订单详情:",
        [STR_ORDER_ITEMS_FMT]        = "共%d件衣物",
        [STR_ORDER_PAID_FMT]         = "实付 %s",
        [STR_ORDER_TIME_START]       = "下单时间：",
        [STR_ORDER_TIME_END]         = "结束时间：",
        [STR_ORDER_INVOICE]          = "开发票",
        [STR_ORDER_DETAIL_BTN]       = "订单详情",
        [STR_ORDER_TOTAL_FMT]        = "合计：%s",
        [STR_DATA_UPLOAD_HDR]        = "上传项",
        [STR_DATA_STRATEGY_HDR]    = "上传策略",
        [STR_DATA_BASIC]             = "基础运行数据",
        [STR_DATA_SENSOR]            = "传感器数据",
        [STR_DATA_FAULT]             = "故障日志",
        [STR_DATA_AUTO_DISPENSE]     = "自动投放数据",
        [STR_DATA_PAYMENT_ORDER]     = "支付订单",
        [STR_DATA_USER_OP]           = "用户操作记录",
        [STR_DATA_DEVICE]            = "设备状态",
        [STR_DATA_REALTIME]          = "实时上传",
        [STR_DATA_SCHEDULED]         = "定时上传",
        [STR_DATA_WIFI_ONLY]         = "仅 WiFi 上传",
        [STR_DATA_4G_ONLY]           = "仅 4G 上传",
        [STR_DATA_FORBIDDEN]         = "禁止上传",
        [STR_WIFI_SETTINGS]          = "WIFI设置",
        [STR_WIFI_PROMPT]            = "使用右上方的切换开关打开无线，连接到附近的无线网络",
        [STR_WIFI_PROVISIONING]      = "正在连接无线网络......",
        [STR_WIFI_SUCCESS]           = "配网成功",
        [STR_WIFI_FAIL]              = "连接失败，请重新连接！",
        [STR_4G_SETTINGS]            = "4G设置",
        [STR_4G_PROMPT]              = "使用右上方的切换开关打开4G，连接到附近的网络",
        [STR_4G_PROVISIONING]        = "正在配网中......",
        [STR_4G_SUCCESS]             = "配网成功",
        [STR_4G_FAIL]                = "连接失败，请重新连接！",
        [STR_MACHINE_ID_SUCCESS]     = "ID设置成功",
        [STR_ADD_TIME_TITLE]         = "请选择烘干追加时间，默认不追加，直接点击启停即可",
        [STR_ADD_TIME_TOTAL_PRICE]   = "总金额",
        [STR_ADD_TIME_TOTAL_TIME]    = "运行总时间",
        [STR_ADD_TIME_BTN_UP]        = "上",
        [STR_ADD_TIME_BTN_DOWN]      = "下",
    },
    [UI_LANG_EN] = {
        [STR_PROG_DAWU]      = "Low Temp",
        [STR_PROG_DANTUO]    = "Med Temp",
        [STR_PROG_BIAOZHUN]  = "High Temp",
        [STR_PROG_TONGZIJIE] = "Cool Air",
        [STR_PROG_KUAIXI]    = "Air Clean",
        [STR_LANG_INDICATOR] = "English",       /* 英文界面时第4列显示 English */
        [STR_PAY_HINT]       = "Scan the QR code on screen\nMachine starts after payment",
        [STR_PAY_HINT_ALIPAY] = "Scan the Alipay QR code on screen\nMachine starts after payment",
        [STR_PAY_HINT_WECHAT] = "Scan the WeChat Pay QR code on screen\nMachine starts after payment",
        [STR_PAY_HINT_ALIPAY_1] = "Scan the Alipay QR code on screen",
        [STR_PAY_HINT_ALIPAY_2] = "Machine starts after payment",
        [STR_PAY_HINT_WECHAT_1] = "Scan the WeChat Pay QR code on screen",
        [STR_PAY_HINT_WECHAT_2] = "Machine starts after payment",
        [STR_PAY_DONE]       = "Payment Done",
        [STR_RUN_WASHING]    = "Drying...",
        [STR_RUN_RINSING]    = "Rinsing...",
        [STR_RUN_SPINNING]   = "Cooling...",
        [STR_RUN_STAGES]     = "Dry        Cool Air",
        [STR_RUN_STAGES_SPIN_ONLY]  = "Cool Air",
        [STR_RUN_STAGES_RINSE_SPIN] = "Dry",
        [STR_END_TITLE]      = "Drying Complete",
        [STR_END_HINT]       = "Please take clothes promptly",
        [STR_ALARM_SOFTENER_LOW] = "Low softener detected, Please refill promptly.",
        [STR_ALARM_DETERGENT_LOW] = "Low detergent detected, Please refill promptly.",
        [STR_ALARM_E1_TITLE]  = "Door Lock Fault",
        [STR_ALARM_E1_LINE1]  = "Please check that the door is closed and locked tightly",
        [STR_ALARM_E1_LINE2]  = "Or check whether the door lock wiring to the main board is loose",
        [STR_ALARM_E2_TITLE]  = "Heater Overheat",
        [STR_ALARM_E2_LINE1]  = "Please power off and check whether the heater is normal",
        [STR_ALARM_E2_LINE2]  = "",
        [STR_ALARM_E3_TITLE]  = "Heater Overheat Fault",
        [STR_ALARM_E3_LINE1]  = "Please power off and check the heater and safety thermostat",
        [STR_ALARM_E3_LINE2]  = "",
        [STR_ALARM_E4_TITLE]  = "Air Outlet Overheat",
        [STR_ALARM_E4_LINE1]  = "Please power off and check whether the heater works properly and the air outlet is clear",
        [STR_ALARM_E4_LINE2]  = "",
        [STR_ALARM_E5_TITLE]  = "Motor Fault",
        [STR_ALARM_E5_LINE1]  = "Please check whether the motor wiring to the main board is normal",
        [STR_ALARM_E5_LINE2]  = "",
        [STR_ALARM_E6_TITLE]  = "Display Communication Fault",
        [STR_ALARM_E6_LINE1]  = "Please check the wiring between the main board and the display",
        [STR_ALARM_E6_LINE2]  = "",
        [STR_ALARM_E7_TITLE]  = "Temperature Sensor Fault",
        [STR_ALARM_E7_LINE1]  = "Please power off and check whether the temperature sensor is damaged",
        [STR_ALARM_E7_LINE2]  = "Or check whether the sensor wiring to the main board is connected properly",
        [STR_ALARM_E8_TITLE]  = "Belt Pulley Micro-switch Fault",
        [STR_ALARM_E8_LINE1]  = "Please check whether the drum belt is normal",
        [STR_ALARM_E8_LINE2]  = "Or check whether the micro-switch wiring is normal",
        [STR_ALARM_E9_TITLE]  = "Motor Communication Fault",
        [STR_ALARM_E9_LINE1]  = "Please restart the machine",
        [STR_ALARM_E9_LINE2]  = "And check the communication wiring between the motor driver and the main board",
        [STR_ALARM_E10_TITLE] = "Driver Board Communication Fault",
        [STR_ALARM_E10_LINE1] = "Please restart the machine and check the driver board wiring",
        [STR_ALARM_E10_LINE2] = "",
        [STR_ALARM_E11_TITLE] = "Main Board Communication Fault",
        [STR_ALARM_E11_LINE1] = "Please restart the machine and check the power board wiring",
        [STR_ALARM_E11_LINE2] = "",
        [STR_ALARM_E12_TITLE] = "IoT Module Configuration Fault",
        [STR_ALARM_E12_LINE1] = "Please power off and reconfigure the network",
        [STR_ALARM_E12_LINE2] = "",
        [STR_ALARM_E13_TITLE] = "Payment Communication Fault",
        [STR_ALARM_E13_LINE1] = "Please check the network and retry payment",
        [STR_ALARM_E13_LINE2] = "",
        [STR_ALARM_E14_TITLE] = "Payment Timeout",
        [STR_ALARM_E14_LINE1] = "Please select a program again and pay",
        [STR_ALARM_E14_LINE2] = "",
        [STR_ALARM_FAULT_CALL]    = "If the issue persists, please call:",
        [STR_ALARM_FAULT_SERVICE] = "After-sales engineers provide professional service.",
        [STR_ALARM_FAULT_PHONE]   = "400-999-999",
        [STR_ADMIN_MENU_TITLE]       = "Admin Settings",
        [STR_ADMIN_LANG_TITLE]       = "Language",
        [STR_ADMIN_LANG_HINT]        = "System Language",
        [STR_ADMIN_LANG_BTN_ZH]      = "Chinese",
        [STR_ADMIN_LANG_BTN_EN]      = "English",
        [STR_ADMIN_M1_MACHINE_ID]    = "Machine ID",
        [STR_ADMIN_M1_PROGRAM]       = "Programs",
        [STR_ADMIN_M1_BRIGHTNESS]    = "Brightness",
        [STR_ADMIN_M1_SOUND]         = "Sound",
        [STR_ADMIN_M1_LANGUAGE]      = "Language",
        [STR_ADMIN_M1_STANDBY]       = "Standby",
        [STR_ADMIN_M1_FACTORY_RESET] = "Factory Reset",
        [STR_ADMIN_M1_CONTACT]       = "Contact",
        [STR_ADMIN_M2_AUTO_DISPENSE] = "Anti-Tangle",
        [STR_ADMIN_M2_FRESH_AIR]     = "Fresh Air",
        [STR_ADMIN_M2_UPGRADE]       = "Upgrade",
        [STR_ADMIN_M2_NETWORK]       = "Network",
        [STR_ADMIN_M2_DATA]          = "Data",
        [STR_ADMIN_M2_PAYMENT]       = "Payment",
        [STR_ADMIN_M2_PASSWORD]      = "Password",
        [STR_ADMIN_M2_VENDOR_MAINT]  = "Vendor Maint.",
        [STR_VENDOR_SERIAL_HINT]      = "Enter factory serial no.",
        [STR_VENDOR_SELF_CHECK]     = "1. Self-check Prog.",
        [STR_VENDOR_SELF_LEARN]     = "2. Cycle Prog.",
        [STR_SELF_CHECK_TITLE]      = "Self-check Prog.",
        [STR_SELF_CHECK_DONE]       = "Self Check Done",
        [STR_CYCLE_TITLE]           = "Cycle Prog.",
        [STR_CYCLE_RUN_COUNT_FMT]   = "Completed: %u",
        [STR_BTN_CONFIRM]            = "Confirm",
        [STR_BTN_CANCEL]             = "Cancel",
        [STR_BTN_OK]                 = "OK",
        [STR_BTN_RESET]              = "Reset",
        [STR_BTN_ON]                 = "On",
        [STR_BTN_OFF]                = "Off",
        [STR_BTN_QUERY]              = "Query",
        [STR_PWD_ENTER_ADMIN]        = "Enter admin password",
        [STR_PWD_WRONG_RETRY]        = "Wrong password, try again later",
        [STR_PWD_ENTER_OLD]          = "Enter current password",
        [STR_PWD_ENTER_NEW]          = "Enter new password",
        [STR_PWD_ENTER_NEW_AGAIN]    = "Confirm new password:",
        [STR_PWD_WRONG_REENTER]      = "Wrong password, please re-enter",
        [STR_PWD_MISMATCH]           = "Passwords do not match, please re-enter",
        [STR_PWD_CHANGE_OK]          = "Password changed successfully",
        [STR_MACHINE_ID_TITLE]       = "Machine ID Settings",
        [STR_MACHINE_ID_CUR_NONE]    = "Current ID: Not configured",
        [STR_MACHINE_ID_CUR_FMT]     = "Current ID: %06u",
        [STR_PROG_FIELD_PRICE]       = "Price",
        [STR_PROG_FIELD_DRY_TEMP]    = "Dry Temp",
        [STR_PROG_FIELD_INIT_DRY]    = "Init Dry Time",
        [STR_PROG_FIELD_COOL_TIME]   = "Cool Time",
        [STR_PROG_FIELD_ADD_COUNT]   = "Add Cycles",
        [STR_PROG_FIELD_ADD_TIME]    = "Add Time",
        [STR_PROG_FIELD_ADD_PRICE]   = "Add Price",
        [STR_PROG_WATER_SMART]       = "Smart",
        [STR_BRIGHTNESS_LINE1]       = "Screen stays on during operation",
        [STR_BRIGHTNESS_LINE2]       = "If off, screen turns off after inactivity during operation",
        [STR_SOUND_TOUCH]            = "Touch Sound:",
        [STR_SOUND_VOICE]            = "Voice Broadcast:",
        [STR_DORMANCY_TITLE]         = "Standby Timeout",
        [STR_DORMANCY_CUR_FMT]       = "Current: %s",
        [STR_DORMANCY_ROLLER]        = "1 min\n2 min\n5 min\n10 min\n15 min\n30 min\n1 hour\n2 hour\nAlways On",
        [STR_DORM_1MIN]              = "1 min",
        [STR_DORM_2MIN]              = "2 min",
        [STR_DORM_5MIN]              = "5 min",
        [STR_DORM_10MIN]             = "10 min",
        [STR_DORM_15MIN]             = "15 min",
        [STR_DORM_30MIN]             = "30 min",
        [STR_DORM_1H]                = "1 hour",
        [STR_DORM_2H]                = "2 hour",
        [STR_DORM_NO_SLEEP]          = "Always On",
        [STR_DORM_TIME_SET]          = "Time",
        [STR_DORM_TIME_HINT]         = "Set how long before the screen sleeps. Default 5 min",
        [STR_DORM_NO_SLEEP_HINT]     = "Keep the screen always on",
        [STR_DORM_TIME_PREFIX]       = "Screen will turn off in",
        [STR_DORM_TIME_SUFFIX]       = "",
        [STR_FACTORY_CONFIRM_Q]      = "Restore factory settings?",
        [STR_FACTORY_CONFIRM_HINT]   = "(All settings will be reset to factory defaults)",
        [STR_FACTORY_RESTORING]      = "Restoring factory settings......",
        [STR_FACTORY_DONE]           = "All settings restored to factory defaults",
        [STR_CONTACT_HOTLINE]        = "24-hour hotline: 400-999-999",
        [STR_CONTACT_SLOGAN]         = "Duckling hotline provides premium service!",
        [STR_AUTO_DISP_LINE1]        = "When enabled, prevents clothes from tangling and improves drying uniformity",
        [STR_FRESH_AIR_LINE1]        = "When enabled, drum rotates intermittently after drying",
        [STR_UPGRADE_CONFIRM_Q]      = "Upgrade system to the latest version?",
        [STR_UPGRADE_IN_PROGRESS]    = "Upgrading system......",
        [STR_UPGRADE_LATEST]         = "System is up to date",
        [STR_PAYMENT_METHOD]         = "Payment Method",
        [STR_PAYMENT_TIMEOUT]        = "Payment Timeout",
        [STR_PAYMENT_ORDER]          = "Order History",
        [STR_PAYMENT_ALIPAY]         = "Alipay",
        [STR_PAYMENT_WECHAT]         = "WeChat Pay",
        [STR_PAYMENT_ORDER_HINT]     = "Query up to 100 recent orders on this machine",
        [STR_PAYMENT_TIMEOUT_ROLLER] = "60s\n120s\n180s\n240s\n300s",
        [STR_PAYMENT_TIME_PREFIX]    = "Payment cancels in",
        [STR_PAYMENT_TIME_SUFFIX]    = "min",
        [STR_ORDER_STATUS_RUNNING]   = "In progress",
        [STR_ORDER_STATUS_DONE]      = "Completed",
        [STR_ORDER_DETAIL_HDR]       = "Order detail:",
        [STR_ORDER_ITEMS_FMT]        = "%d items",
        [STR_ORDER_PAID_FMT]         = "Paid %s",
        [STR_ORDER_TIME_START]       = "Start:",
        [STR_ORDER_TIME_END]         = "End:",
        [STR_ORDER_INVOICE]          = "Invoice",
        [STR_ORDER_DETAIL_BTN]       = "Details",
        [STR_ORDER_TOTAL_FMT]        = "Total: %s",
        [STR_DATA_UPLOAD_HDR]        = "Upload Items",
        [STR_DATA_STRATEGY_HDR]    = "Upload Strategy",
        [STR_DATA_BASIC]             = "Basic Runtime Data",
        [STR_DATA_SENSOR]            = "Sensor Data",
        [STR_DATA_FAULT]             = "Fault Logs",
        [STR_DATA_AUTO_DISPENSE]     = "Auto Dispense Data",
        [STR_DATA_PAYMENT_ORDER]     = "Payment Orders",
        [STR_DATA_USER_OP]           = "User Activity",
        [STR_DATA_DEVICE]            = "Device Status",
        [STR_DATA_REALTIME]          = "Real-time Upload",
        [STR_DATA_SCHEDULED]         = "Scheduled Upload",
        [STR_DATA_WIFI_ONLY]         = "Wi-Fi Only",
        [STR_DATA_4G_ONLY]           = "4G Only",
        [STR_DATA_FORBIDDEN]         = "Upload Disabled",
        [STR_WIFI_SETTINGS]          = "WiFi Settings",
        [STR_WIFI_PROMPT]            = "Use the switch at top-right to enable WiFi",
        [STR_WIFI_PROVISIONING]      = "Connecting to WiFi......",
        [STR_WIFI_SUCCESS]           = "Connected",
        [STR_WIFI_FAIL]              = "Connection failed, please retry!",
        [STR_4G_SETTINGS]            = "4G Settings",
        [STR_4G_PROMPT]              = "Use the switch at top-right to enable 4G and connect to nearby networks",
        [STR_4G_PROVISIONING]        = "Connecting......",
        [STR_4G_SUCCESS]             = "Connected successfully",
        [STR_4G_FAIL]                = "Connection failed, please retry!",
        [STR_MACHINE_ID_SUCCESS]     = "ID set successfully",
        [STR_ADD_TIME_TITLE]         = "Select extra dry time. Default is none — tap Start/Stop to continue",
        [STR_ADD_TIME_TOTAL_PRICE]   = "Total Price",
        [STR_ADD_TIME_TOTAL_TIME]    = "Total Time",
        [STR_ADD_TIME_BTN_UP]        = "Up",
        [STR_ADD_TIME_BTN_DOWN]      = "Down",
    },
};

#define TOTAL_PROGRAMS 5    //总程序数
static const ui_str_id_t g_mode_name_ids[TOTAL_PROGRAMS] = {
    STR_PROG_DAWU, STR_PROG_DANTUO, STR_PROG_BIAOZHUN, STR_PROG_TONGZIJIE, STR_PROG_KUAIXI
};

static void carousel_update_card_images(void);  //按当前选中程序刷新 5 张可见卡片图与名称标签
static void running_screen_sync_mode_name(void);  //运行页程序名标签与 g_wheel_sel 保持一致
static void running_status_sync_labels(void);  //按程序与剩余时间刷新运行页底部左右文案
static void alarm_content_update(void);  //刷新轮播当前子页内容（缺液图/文案）
static void alarm_fault_panel_relayout_content(uint8_t fault_idx);  //说明2 显隐 + 页脚电话/售后重排
static void alarm_fault_panels_relayout_content(void);  //全部故障子页重排内容
static void home_sync_program_labels(void);  //按当前选中程序刷新主页底部时间/温度/价格标签

static ui_lang_t ui_lang_get(void);	//获取当前界面语言
static void ui_lang_set(ui_lang_t lang);	//设置界面语言并刷新文案（与当前相同则不变）
static void ui_lang_toggle(void);	//在中/英之间切换 g_ui_lang
static const char * ui_translation(ui_str_id_t id);	//按当前语言取字符串；id 非法时返回空串
static void ui_lang_bind_label(lv_obj_t * lbl, ui_str_id_t id);	//将 label 登记到绑定表，并立即设置对应语言的文案
static void ui_lang_apply_all(void);	//切换语言后刷新：已绑定标签 + 轮播名 + 运行页程序名 + 主页底部参数
static void ui_lang_refresh_rollers(void);	//待机/支付 roller 选项随语言切换
static void program_admin_refresh_i18n(void);	//程序设置页字段名与程序 Tab 文案
static void admin_refresh_visible_status_text(void);	//恢复出厂/升级/4G 等动态状态文案
static void orange_btn_bind_i18n(lv_obj_t * btn, ui_str_id_t id);	//橙色按钮内 label 绑定 i18n
static void admin_prog_btn_bind_i18n(lv_obj_t * btn, ui_str_id_t id);	//程序 Tab 按钮绑定 i18n
static const char * ui_program_name_get(int32_t idx);	//按程序索引取显示名（大物/Heavy 等），idx 先取模到 0..4
static int32_t wheel_mod_total(int32_t v);  //程序索引在 0..TOTAL_PROGRAMS-1 内循环取模

/* 获取当前界面语言（默认中文 UI_LANG_ZH） */
static ui_lang_t ui_lang_get(void)
{
    return g_ui_lang;                                /* 返回全局语言状态 */
}

/* 在中/英之间切换 g_ui_lang */
static void ui_lang_toggle(void)
{
    g_ui_lang = (g_ui_lang == UI_LANG_ZH) ? UI_LANG_EN : UI_LANG_ZH; /* 中文↔英文 */
}

/* 按当前语言取字符串；id 非法时返回空串 */
static const char * ui_translation(ui_str_id_t id)
{
    if((unsigned)id >= STR_COUNT) return "";           /* 越界保护 */
    return g_ui_strings[g_ui_lang][id];                /* 查表：语言 × 字符串 ID */
}

/* 将 label 登记到绑定表，并立即设置对应语言的文案 */
static void ui_lang_bind_label(lv_obj_t * lbl, ui_str_id_t id)
{
    if(lbl == NULL) return;                            /* 空指针不绑定 */
    if(g_lang_bind_count < UI_LANG_BIND_MAX) {         /* 表未满则登记 */
        g_lang_binds[g_lang_bind_count].lbl = lbl;     /* 记录控件指针 */
        g_lang_binds[g_lang_bind_count].id = id;       /* 记录字符串 ID */
        g_lang_bind_count++;                           /* 绑定条数 +1 */
    }
    lv_label_set_text(lbl, ui_translation(id));        /* 按当前语言刷新标签文字 */
}

/* 橙色实心/描边按钮内 label 绑定 i18n */
static void orange_btn_bind_i18n(lv_obj_t * btn, ui_str_id_t id)
{
    if(btn == NULL) return;
    ui_lang_bind_label(lv_obj_get_child(btn, 0), id);
}

/* 程序设置上栏程序 Tab 按钮绑定 i18n */
static void admin_prog_btn_bind_i18n(lv_obj_t * btn, ui_str_id_t id)
{
    if(btn == NULL) return;
    ui_lang_bind_label(lv_obj_get_child(btn, 0), id);
}

/* 设置界面语言；与当前相同时不刷新 */
static void ui_lang_set(ui_lang_t lang)
{
    if(g_ui_lang == lang) return;
    g_ui_lang = lang;
    ui_lang_apply_all();
}

/* 按程序索引取显示名，idx 先取模到 0..4 */
static const char * ui_program_name_get(int32_t idx)
{
    idx = wheel_mod_total(idx);                        /* 程序索引环上取模 */
    return ui_translation(g_mode_name_ids[(unsigned)idx]); /* 程序 ID → 当前语言字符串 */
}

#define UI_FIXED_W          1600u     //屏幕宽度像素
#define UI_FIXED_H          600u      //屏幕高度像素

#define MODE_SCALE_MIN      180       //最小缩放 (180/256倍)
#define MODE_SCALE_MAX      256       //最大缩放 (256/256倍)

#define MODE_IDX_DEFAULT    2         //商用洗默认选中「高温」（居中槽位）
#define CAROUSEL_VISIBLE_SLOTS 5      //轮播区同时可见卡片数（其余程序在逻辑环上隐藏）
#define CAROUSEL_CENTER_SLOT   2      //居中选中槽位下标 0..4

#define COL_BG              0x000000  //背景颜色-黑
#define COL_TEXT            0xFFFFFF  //文本颜色-白
#define COL_ORANGE          0xFF8C42  //按钮颜色-橙
#define COL_DIM             0x888888  //非选中状态颜色-灰
#define COL_SETTING_HINT    0xC6C4C4
#define ADMIN_SW_BORDER_W             3
#define ADMIN_SW_KNOB_SIZE           23
#define ADMIN_SW_PAD_MAIN             1
#define ADMIN_SW_SIZE_W              70
#define ADMIN_SW_SIZE_H              35
#define COL_CHILD_LOCK_RED  0xD03030  //童锁激活：圆形按钮填充色
#define COL_CHILD_LOCK_WHITE 0xF5F5F5 //童锁未激活：白按钮填充色
#define CHILD_LOCK_LONG_PRESS_MS 3000u	//童锁长按 3 秒
#define UI_DORMANCY_TIMEOUT_DEFAULT_MS  300000u /* 空闲无操作进待机，默认 5 分钟 */
#define UI_DORMANCY_DISABLED_MS           0u    /* 不熄屏：不启动空闲待机定时器 */

static const uint32_t g_dormancy_timeout_ms_tbl[] = {
	60000u, 120000u, 300000u, 600000u, 900000u,
	1800000u, 3600000u, 7200000u, UI_DORMANCY_DISABLED_MS
};
#define DORMANCY_ROLLER_CNT          ((uint32_t)(sizeof(g_dormancy_timeout_ms_tbl) / sizeof(g_dormancy_timeout_ms_tbl[0])))
#define DORMANCY_ROLLER_DEFAULT_IDX  2u  /* 5 分钟 */

static const ui_str_id_t g_dormancy_label_ids[DORMANCY_ROLLER_CNT] = {
	STR_DORM_1MIN, STR_DORM_2MIN, STR_DORM_5MIN, STR_DORM_10MIN, STR_DORM_15MIN,
	STR_DORM_30MIN, STR_DORM_1H, STR_DORM_2H, STR_DORM_NO_SLEEP
};

#define ADMIN_PWD_LEN      7   /* 6 位数字 + '\0' */
#define ADMIN_PWD_DEFAULT  "888888"

static lv_group_t* g_ui_group;
static lv_group_t* g_group_home;
static lv_group_t* g_group_off;
static lv_group_t* g_group_running;
static lv_group_t* g_group_end;
static lv_group_t* g_group_pay;
static lv_group_t* g_group_pay_done;
static lv_group_t* g_group_alarm;
static lv_group_t* g_group_admin;
static lv_group_t* g_group_add_time;

typedef enum {
    PASSWORD,			//密码页面
    MENU1,				//管理员设置页1（8 宫格）
    MENU2,				//管理员设置页2（7 按钮：4+3）
    MACHINE_ID,			//机器ID页面
    PROGRAM_SETTINGS,	//程序设置页面
    SCREEN_BRIGHTNESS,	//屏幕亮度设置页面
    SOUND_CONTROL,		//声音控制设置页面
    DORMANCY_STANDBY,	//待机时间设置页面
    LANGUAGE_SETTINGS,	//语言设置页面
    FACTORY_RESET,		//恢复默认设置页面
    CONTACT_US,			//联系我们页面
    AUTO_DISPENSE,		//防缠绕功能页面
    FRESH_AIR_CARE,		//新风护理页面
    SYSTEM_UPGRADE,		//系统升级页面
    PAYMENT_SETTINGS,	//支付设置页面
    DATA_SETTINGS,		//数据设置页面
    NETWORK_SETTINGS,	//网络设置入口页
    WIFI_SETTINGS,		//WIFI 设置页面（占位）
    SETTINGS_4G,		//4G 设置页面（三步配网）
    PASSWORD_CHANGE_OLD,	//密码修改：校验原密码
    PASSWORD_CHANGE_NEW,	//密码修改：设置新密码
    VENDOR_SERIAL,		//厂商维护：出厂序列号
    VENDOR_MENU,		//厂商维护：功能选择
} admin_view_t;

// 自检程序相关
#define MACHINE_MODEL           "XQG150-001"    //烘干机型号
#define VENDOR_SERIAL_CODE      "111111"        //出厂序列号
#define SELFCHECK_STEP_UI_MS    3000u           //每步显示时间
#define SELFCHECK_INIT_UI_MS    3000u           //初始化显示时间
#define SELFCHECK_FLASH_MS      500u            //闪烁间隔
#define SELFCHECK_BLINK_MS      500u            //闪烁间隔
#define SELFCHECK_STATUS_ROW_Y  ((lv_coord_t)((UI_FIXED_H * 30) / 100))//状态行高度
#define SELFCHECK_STATUS_ROW_W  ((lv_coord_t)((UI_FIXED_W * 87) / 100))//状态行宽度
#define SELFCHECK_LIVE_TEMP_DEFAULT_C  45          //自检筒温默认占位（℃）
#define SELFCHECK_STEP_COUNT    3               //步骤数

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
    ADMIN_FACTORY_PHASE_PROMPT,
    ADMIN_FACTORY_PHASE_RESTORING,
    ADMIN_FACTORY_PHASE_DONE,
} admin_factory_phase_t;

typedef enum {
    ADMIN_SYSTEM_UPGRADE_PHASE_PROMPT,
    ADMIN_SYSTEM_UPGRADE_PHASE_UPGRADING,
    ADMIN_SYSTEM_UPGRADE_PHASE_DONE,
} admin_system_upgrade_phase_t;

typedef enum {
    ADMIN_4G_PHASE_PROMPT,
    ADMIN_4G_PHASE_PROVISIONING,
    ADMIN_4G_PHASE_SUCCESS,
    ADMIN_4G_PHASE_FAILURE,
} admin_4g_phase_t;

typedef enum {
    ADMIN_WIFI_PHASE_PROMPT,
    ADMIN_WIFI_PHASE_PROVISIONING,
    ADMIN_WIFI_PHASE_SUCCESS,
    ADMIN_WIFI_PHASE_FAILURE,
} admin_wifi_phase_t;

typedef enum {
    ADMIN_DATA_PAGE_LIST,
    ADMIN_DATA_PAGE_STRATEGY,
} admin_data_page_t;

typedef enum {
    ADMIN_PAYMENT_PAGE_LIST,
    ADMIN_PAYMENT_PAGE_METHOD,
    ADMIN_PAYMENT_PAGE_TIMEOUT,
    ADMIN_PAYMENT_PAGE_ORDERS,
    ADMIN_PAYMENT_PAGE_ORDER_SUMMARY,
    ADMIN_PAYMENT_PAGE_ORDER_DETAIL,
} admin_payment_page_t;

#define ADMIN_PAYMENT_ORDER_CNT  5

static lv_obj_t * g_scr_admin;
static lv_obj_t * g_lbl_clock_admin;
static lv_obj_t * g_admin_btn_back;
static lv_obj_t * g_admin_btn_runpause;
static lv_obj_t * g_admin_btn_power;
static lv_obj_t * g_admin_panel_pwd;
static lv_obj_t * g_admin_panel_menu1;
static lv_obj_t * g_admin_panel_menu2;
static lv_obj_t * g_admin_panel_machine_id;
static lv_obj_t * g_admin_kb;
static lv_obj_t * g_admin_kb_ta;
static lv_obj_t * g_admin_lbl_serial_title;
static lv_obj_t * g_admin_input_wrap;
static lv_obj_t * g_admin_ta_pwd;
static lv_obj_t * g_admin_ta_machine_id;
static lv_obj_t * g_admin_lbl_msg_pwd;
static lv_obj_t * g_admin_lbl_msg_machine_id;
static lv_obj_t * g_admin_lbl_machine_id_cur;
static lv_obj_t * g_admin_btn_machine_confirm;
static lv_obj_t * g_admin_btn_machine_cancel;
static lv_obj_t * g_admin_mid_success_overlay;
static lv_obj_t * g_admin_lbl_menu1_title;
static lv_obj_t * g_admin_lbl_menu2_title;
static lv_obj_t * g_admin_menu1_btns[8];
static lv_obj_t * g_admin_menu2_btns[8];
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
static lv_group_t * g_group_selfcheck;
static selfcheck_ui_state_t g_selfcheck_ui_state = SELFCHECK_UI_MODEL;
static lv_timer_t * g_selfcheck_timer;
static lv_timer_t * g_selfcheck_blink_timer;
static lv_obj_t * g_selfcheck_blink_target;
static uint8_t g_selfcheck_flash_count;
static bool g_selfcheck_flash_on;
static bool g_selfcheck_blink_on;
static int16_t g_selfcheck_live_temp_c = SELFCHECK_LIVE_TEMP_DEFAULT_C;

/* ========== 循环程序（5.2.1 寿命试验）UI 控件与状态（类型见 ui_program_admin_t 之后） ========== */
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

static lv_obj_t * g_scr_cycle;
static lv_obj_t * g_lbl_clock_cycle;
static lv_obj_t * g_cycle_btn_back;
static lv_obj_t * g_cycle_btn_runpause;
static lv_obj_t * g_cycle_btn_power;
static lv_obj_t * g_cycle_panel_program;
static lv_obj_t * g_cycle_lbl_run_count;
static lv_obj_t * g_cycle_panel_fault;
static lv_obj_t * g_cycle_lbl_fault_alt;
static lv_obj_t * g_cycle_panel_fault_run;   /* 运行页故障叠层（运行中触发故障时不回设定页） */
static lv_obj_t * g_cycle_lbl_fault_alt_run;
static bool g_cycle_fault_on_running;
static lv_group_t * g_group_cycle;
static bool g_cycle_active;
static cycle_ui_state_t g_cycle_ui_state;
static lv_timer_t * g_cycle_fault_blink_timer;
static cycle_fault_phase_t g_cycle_fault_phase;
static lv_obj_t * g_cycle_kb;              /* 循环程序页数字键盘 */
static lv_obj_t * g_cycle_ta_active;       /* 当前绑定的 textarea */
static bool s_cycle_kb_encoder_inited;
#define CYCLE_FAULT_CODE_MAX  14u   /* 与 UI_ALARM_FAULT_COUNT 一致 */
#define CYCLE_FAULT_BLINK_MS  500u  /* 循环试验故障码闪烁间隔 */
#define CYCLE_PROG_FIELD_CNT  5u    /* 循环程序页参数列数（无程序金额/追加时间金额） */

static lv_obj_t * s_admin_group_prev_focus;     /* 编码器 focus_cb：检测 menu2 第1钮左转 */
/* 程序设置子页控件 */
static lv_obj_t * g_admin_panel_program;
static lv_obj_t * g_admin_panel_brightness;
static lv_obj_t * g_admin_panel_dormancy;

static lv_obj_t * g_admin_img_dormancy_title_box;
static lv_obj_t * g_admin_lbl_dormancy_title;
static lv_obj_t * g_admin_dormancy_set_box_wrap;
static lv_obj_t * g_admin_dormancy_row_time;
static lv_obj_t * g_admin_lbl_dormancy_time;
static lv_obj_t * g_admin_lbl_dormancy_time_hint;
static lv_obj_t * g_admin_dormancy_sw_time;
static lv_obj_t * g_admin_dormancy_row_no_sleep;
static lv_obj_t * g_admin_lbl_dormancy_no_sleep;
static lv_obj_t * g_admin_lbl_dormancy_no_sleep_hint;
static lv_obj_t * g_admin_dormancy_sw_no_sleep;
static bool g_admin_dormancy_ui_loading = false;

/* 待机时间选择子页控件 */
static lv_obj_t * g_admin_dormancy_tp_wrap;        /* 时间选择子页容器 */
static lv_obj_t * g_admin_dormancy_tp_lbl_prefix;  /* "机器将在" */
static lv_obj_t * g_admin_dormancy_tp_lbl_suffix;  /* "后熄屏" */
static lv_obj_t * g_admin_dormancy_tp_digit_imgs[4]; /* 4 个 time_set_box 数字图 */
static lv_obj_t * g_admin_dormancy_tp_digit_rollers[4]; /* 4 个数字滚动条（叠在图上） */
static lv_obj_t * g_admin_dormancy_tp_lbl_colon;   /* 冒号 ":" */
static lv_obj_t * g_admin_dormancy_tp_btn_ok;      /* 确定 */
static lv_obj_t * g_admin_dormancy_tp_btn_cancel;  /* 取消 */
static uint32_t g_admin_dormancy_tp_minutes;        /* 编辑中的分钟数 0~59 */
static uint32_t g_admin_dormancy_tp_seconds;        /* 编辑中的秒数 0~59 */
static bool g_admin_dormancy_tp_active = false;     /* 时间选择页是否显示中 */

/* 屏幕亮度子页控件（布局同待机时间：title_box + set_box；滑条仅触摸，无编码器） */
static lv_obj_t * g_admin_img_brightness_title_box;
static lv_obj_t * g_admin_brightness_set_box_wrap;
static lv_obj_t * g_admin_lbl_brightness_title;
static lv_obj_t * g_admin_lbl_brightness_main;      /* set_box 内主文字「屏幕亮度」 */
static lv_obj_t * g_admin_lbl_brightness_line1;     /* 说明小字（自动换行） */
static lv_obj_t * g_admin_lbl_brightness_line2;     /* 保留：旧两行文案已合并到 line1 */
static lv_obj_t * g_admin_sw_run_always_on;
static lv_obj_t * g_admin_brightness_fill;          /* 进度裁剪窗：宽度随数值变，露出固定渐变 */
static lv_obj_t * g_admin_brightness_fill_grad;     /* 整轨宽黑→蓝渐变（颜色不随滑钮压缩） */
static lv_obj_t * g_admin_slider_brightness;        /* 亮度交互滑条（透明轨，仅旋钮+触摸） */

/* 声音控制子页控件（布局同待机时间 title_box+set_box；滑条同亮度页） */
static lv_obj_t * g_admin_panel_sound;
static lv_obj_t * g_admin_img_sound_title_box;
static lv_obj_t * g_admin_lbl_sound_title;
static lv_obj_t * g_admin_sound_set_box_wrap;
static lv_obj_t * g_admin_lbl_sound_line1;          /* 触控声音 */
static lv_obj_t * g_admin_lbl_sound_line2;          /* 声音播报 */
static lv_obj_t * g_admin_sw_touch_sound;
static lv_obj_t * g_admin_sw_voice_broadcast;
static lv_obj_t * g_admin_sound_volume_fill;        /* 播报音量进度裁剪窗 */
static lv_obj_t * g_admin_sound_volume_fill_grad;
static lv_obj_t * g_admin_slider_sound_volume;
static lv_obj_t * g_admin_touch_sound_volume_fill;  /* 触控音量进度裁剪窗 */
static lv_obj_t * g_admin_touch_sound_volume_fill_grad;
static lv_obj_t * g_admin_slider_touch_sound_volume;

/* 语言设置子页控件（布局同待机时间 title_box+set_box；右侧按钮同 ID；文案同 WIFI 样式） */
static lv_obj_t * g_admin_panel_lang;
static lv_obj_t * g_admin_img_lang_title_box;
static lv_obj_t * g_admin_lbl_lang_title;
static lv_obj_t * g_admin_lang_set_box_wrap;
static lv_obj_t * g_admin_lbl_lang_line1;
static lv_obj_t * g_admin_btn_lang_zh;
static lv_obj_t * g_admin_btn_lang_en;

/* 恢复默认子页控件（布局同待机时间 title_box+set_box；确定/取消同 ID） */
static lv_obj_t * g_admin_panel_factory;
static lv_obj_t * g_admin_img_factory_title_box;
static lv_obj_t * g_admin_lbl_factory_title;
static lv_obj_t * g_admin_factory_set_box_wrap;
static lv_obj_t * g_admin_lbl_factory_line1;
static lv_obj_t * g_admin_lbl_factory_line2;
static lv_obj_t * g_admin_lbl_factory_status;
static lv_obj_t * g_admin_btn_factory_ok;
static lv_obj_t * g_admin_btn_factory_cancel;

/* 联系我们子页控件（布局同待机时间 title_box+set_box；文案/二维码坐标保持原值） */
static lv_obj_t * g_admin_panel_contact;
static lv_obj_t * g_admin_img_contact_title_box;
static lv_obj_t * g_admin_lbl_contact_title;
static lv_obj_t * g_admin_contact_set_box_wrap;
static lv_obj_t * g_admin_lbl_contact_line1;
static lv_obj_t * g_admin_lbl_contact_line2;
static lv_obj_t * g_admin_img_contact_qr;

/* 声音控制子页控件（布局同待机时间 title_box+set_box；滑条同亮度页） */
static lv_obj_t * g_admin_panel_sound;
static lv_obj_t * g_admin_img_sound_title_box;
static lv_obj_t * g_admin_lbl_sound_title;
static lv_obj_t * g_admin_sound_set_box_wrap;
static lv_obj_t * g_admin_lbl_sound_line1;          /* 触控声音 */
static lv_obj_t * g_admin_lbl_sound_line2;          /* 声音播报 */
static lv_obj_t * g_admin_sw_touch_sound;
static lv_obj_t * g_admin_sw_voice_broadcast;
static lv_obj_t * g_admin_sound_volume_fill;        /* 播报音量进度裁剪窗 */
static lv_obj_t * g_admin_sound_volume_fill_grad;
static lv_obj_t * g_admin_slider_sound_volume;
static lv_obj_t * g_admin_touch_sound_volume_fill;  /* 触控音量进度裁剪窗 */
static lv_obj_t * g_admin_touch_sound_volume_fill_grad;
static lv_obj_t * g_admin_slider_touch_sound_volume;

/* 语言设置子页控件（布局同待机时间 title_box+set_box；右侧按钮同 ID；文案同 WIFI 样式） */
static lv_obj_t * g_admin_panel_lang;
static lv_obj_t * g_admin_img_lang_title_box;
static lv_obj_t * g_admin_lbl_lang_title;
static lv_obj_t * g_admin_lang_set_box_wrap;
static lv_obj_t * g_admin_lbl_lang_line1;
static lv_obj_t * g_admin_btn_lang_zh;
static lv_obj_t * g_admin_btn_lang_en;

/* 恢复默认子页控件（布局同待机时间 title_box+set_box；确定/取消同 ID） */
static lv_obj_t * g_admin_panel_factory;
static lv_obj_t * g_admin_img_factory_title_box;
static lv_obj_t * g_admin_lbl_factory_title;
static lv_obj_t * g_admin_factory_set_box_wrap;
static lv_obj_t * g_admin_lbl_factory_line1;
static lv_obj_t * g_admin_lbl_factory_line2;
static lv_obj_t * g_admin_lbl_factory_status;
static lv_obj_t * g_admin_btn_factory_ok;
static lv_obj_t * g_admin_btn_factory_cancel;

/* 联系我们子页控件（布局同待机时间 title_box+set_box；文案/二维码坐标保持原值） */
static lv_obj_t * g_admin_panel_contact;
static lv_obj_t * g_admin_img_contact_title_box;
static lv_obj_t * g_admin_lbl_contact_title;
static lv_obj_t * g_admin_contact_set_box_wrap;
static lv_obj_t * g_admin_lbl_contact_line1;
static lv_obj_t * g_admin_lbl_contact_line2;
static lv_obj_t * g_admin_img_contact_qr;

/* 防缠绕功能子页控件 */
static lv_obj_t * g_admin_panel_auto_dispense;
static lv_obj_t * g_admin_lbl_auto_dispense_title;
static lv_obj_t * g_admin_lbl_auto_dispense_line1;
static lv_obj_t * g_admin_btn_auto_dispense_on;
static lv_obj_t * g_admin_btn_auto_dispense_off;
/* 新风护理子页控件 */
static lv_obj_t * g_admin_panel_fresh_air_care;
static lv_obj_t * g_admin_lbl_fresh_air_care_title;
static lv_obj_t * g_admin_lbl_fresh_air_care_line1;
static lv_obj_t * g_admin_btn_fresh_air_care_on;
static lv_obj_t * g_admin_btn_fresh_air_care_off;
/* 系统升级子页控件（布局同待机时间 title_box+set_box；确定钮同 ID 取消） */
static lv_obj_t * g_admin_panel_system_upgrade;
static lv_obj_t * g_admin_img_system_upgrade_title_box;
static lv_obj_t * g_admin_lbl_system_upgrade_title;
static lv_obj_t * g_admin_system_upgrade_set_box_wrap;
static lv_obj_t * g_admin_lbl_system_upgrade_line1;
static lv_obj_t * g_admin_lbl_system_upgrade_status;
static lv_obj_t * g_admin_btn_system_upgrade_ok;

/* 支付设置子页控件 */
static lv_obj_t * g_admin_panel_payment;
static lv_obj_t * g_admin_img_payment_title_box;
static lv_obj_t * g_admin_lbl_payment_title;
static lv_obj_t * g_admin_payment_set_box_wrap;
static lv_obj_t * g_admin_payment_list_view;
static lv_obj_t * g_admin_payment_method_view;
static lv_obj_t * g_admin_sw_payment_method;
static lv_obj_t * g_admin_sw_payment_timeout;
static lv_obj_t * g_admin_sw_payment_order;
static lv_obj_t * g_admin_lbl_payment_method;
static lv_obj_t * g_admin_lbl_payment_timeout;
static lv_obj_t * g_admin_lbl_payment_order;
static lv_obj_t * g_admin_sw_payment_wechat;
static lv_obj_t * g_admin_sw_payment_alipay;
static lv_obj_t * g_admin_lbl_payment_wechat;
static lv_obj_t * g_admin_lbl_payment_alipay;
static lv_obj_t * g_admin_payment_orders_view;
static lv_obj_t * g_admin_payment_order_rows[ADMIN_PAYMENT_ORDER_CNT];
static lv_obj_t * g_admin_payment_order_summary_view; /* 列表点击后的摘要中间页 */
static lv_obj_t * g_admin_lbl_order_sum_title;
static lv_obj_t * g_admin_order_sum_status_badge;
static lv_obj_t * g_admin_lbl_order_sum_status;
static lv_obj_t * g_admin_lbl_order_sum_prog;
static lv_obj_t * g_admin_lbl_order_sum_sub;
static lv_obj_t * g_admin_lbl_order_sum_time;
static lv_obj_t * g_admin_lbl_order_sum_total;
static lv_obj_t * g_admin_btn_order_invoice;
static lv_obj_t * g_admin_btn_order_detail;
static lv_obj_t * g_admin_payment_order_detail_view;
static lv_obj_t * g_admin_lbl_order_detail_id;
static lv_obj_t * g_admin_order_detail_status_badge;
static lv_obj_t * g_admin_lbl_order_detail_status;
static lv_obj_t * g_admin_lbl_order_detail_prog;
static lv_obj_t * g_admin_lbl_order_detail_params;
static lv_obj_t * g_admin_lbl_order_detail_items;
static lv_obj_t * g_admin_lbl_order_detail_paid;
static lv_obj_t * g_admin_lbl_order_detail_t_start;
static lv_obj_t * g_admin_lbl_order_detail_t_end;
static int g_admin_payment_order_sel = 0;
static lv_obj_t * g_admin_payment_tp_wrap;
static lv_obj_t * g_admin_payment_tp_lbl_prefix;
static lv_obj_t * g_admin_payment_tp_lbl_suffix;
static lv_obj_t * g_admin_payment_tp_digit_imgs[4];
static lv_obj_t * g_admin_payment_tp_digit_rollers[4];
static lv_obj_t * g_admin_payment_tp_lbl_colon;
static lv_obj_t * g_admin_payment_tp_btn_ok;
static lv_obj_t * g_admin_payment_tp_btn_cancel;
static uint32_t g_admin_payment_tp_minutes;   /* 0~59 */
static uint32_t g_admin_payment_tp_seconds;   /* 0~59 */
static bool g_admin_payment_tp_active = false;
static admin_payment_page_t g_admin_payment_page = ADMIN_PAYMENT_PAGE_LIST;
static bool g_admin_payment_ui_loading = false;
typedef struct {
    const char * title;
    bool running;
    const char * id_tail;
    const char * prog_name;
    const char * summary_sub;
    const char * prog_params;
    int items;
    const char * price;
    const char * time_start;
    const char * time_end;
} admin_payment_order_demo_t;

static const admin_payment_order_demo_t g_admin_payment_order_demo[ADMIN_PAYMENT_ORDER_CNT] = {
    { "Store A (NO.12345)", true,  "****************8423", "Low",
      "10min/WeChat", "Low temp / cool 2min", 10, "¥8.00",
      "2026-06-24 16:20", "2026-06-24 16:28" },
    { "Store A (NO.12384)", false, "****************2384", "Med",
      "10min/WeChat", "Med temp / cool 2min", 8, "¥8.00",
      "2026-06-23 10:12", "2026-06-23 10:40" },
    { "Store B (NO.12345)", false, "****************1245", "High",
      "8min/Alipay", "High temp / cool 2min", 5, "¥5.00",
      "2026-06-22 19:05", "2026-06-22 19:25" },
    { "Store C (NO.12345)", false, "****************3345", "Cool",
      "25min/WeChat", "Cool air", 12, "¥12.00",
      "2026-06-21 14:30", "2026-06-21 15:10" },
    { "Store D (NO.12345)", false, "****************5545", "Clean",
      "10min/WeChat", "Air clean", 10, "¥8.00",
      "2026-06-20 08:40", "2026-06-20 09:08" },
};



/* 数据设置子页控件 */
static lv_obj_t * g_admin_panel_data;
static lv_obj_t * g_admin_lbl_data_title;
static lv_obj_t * g_admin_img_data_title_box;
static lv_obj_t * g_admin_data_set_box_wrap;
static lv_obj_t * g_admin_data_list_view;
static lv_obj_t * g_admin_data_strategy_view;
static lv_obj_t * g_admin_cb_data_upload[7];
static lv_obj_t * g_admin_lbl_data_upload[7];
static lv_obj_t * g_admin_data_row_upload[7];
static lv_obj_t * g_admin_cb_data_strategy[5];
static lv_obj_t * g_admin_lbl_data_strategy[5];
static lv_obj_t * g_admin_data_row_strategy[5];
static admin_data_page_t g_admin_data_page = ADMIN_DATA_PAGE_LIST;
static bool g_admin_data_strategy_ui_loading = false;
static bool g_admin_data_upload_ui_loading = false;

/* 网络设置子页控件 */
static lv_obj_t * g_admin_panel_network;
static lv_obj_t * g_admin_lbl_network_title;
static lv_obj_t * g_admin_btn_network_wifi;
static lv_obj_t * g_admin_btn_network_4g;

/* WIFI 设置子页控件 */
static lv_obj_t * g_admin_panel_wifi;
static lv_obj_t * g_admin_lbl_wifi_title;
static lv_obj_t * g_admin_sw_wifi;
static lv_obj_t * g_admin_lbl_wifi_prompt;
static lv_obj_t * g_admin_lbl_wifi_status;
static lv_obj_t * g_admin_wifi_done_center;
static lv_obj_t * g_admin_wifi_set_box_wrap;
static lv_obj_t * g_admin_img_wifi_title_box;
static lv_obj_t * g_admin_img_wifi_done;
static lv_obj_t * g_admin_lbl_wifi_done;
static admin_wifi_phase_t g_admin_wifi_phase = ADMIN_WIFI_PHASE_PROMPT;
static lv_timer_t * g_admin_wifi_timer;
static bool g_admin_wifi_ui_loading;

/* 4G 设置子页控件 */
static lv_obj_t * g_admin_panel_4g;
static lv_obj_t * g_admin_lbl_4g_title;
static lv_obj_t * g_admin_sw_4g;
static lv_obj_t * g_admin_lbl_4g_prompt;
static lv_obj_t * g_admin_lbl_4g_status;
static lv_obj_t * g_admin_4g_done_center;
static lv_obj_t * g_admin_4g_set_box_wrap;
static lv_obj_t * g_admin_img_4g_title_box;
static lv_obj_t * g_admin_img_4g_done;
static lv_obj_t * g_admin_lbl_4g_done;
static admin_4g_phase_t g_admin_4g_phase = ADMIN_4G_PHASE_PROMPT;
static lv_timer_t * g_admin_4g_timer;
static bool g_admin_4g_ui_loading;

/* 密码修改子页控件（title_box 同待机时间；无 set_box；结果页同配网成功/失败） */
static lv_obj_t * g_admin_panel_pwd_chg_old;
static lv_obj_t * g_admin_img_pwd_chg_old_title_box;
static lv_obj_t * g_admin_lbl_pwd_chg_old_title;
static lv_obj_t * g_admin_pwd_chg_old_content;
static lv_obj_t * g_admin_lbl_pwd_chg_old;
static lv_obj_t * g_admin_ta_pwd_chg_old;
static lv_obj_t * g_admin_lbl_pwd_chg_new1;
static lv_obj_t * g_admin_ta_pwd_chg_new1;
static lv_obj_t * g_admin_panel_pwd_chg_new;
static lv_obj_t * g_admin_img_pwd_chg_new_title_box;
static lv_obj_t * g_admin_lbl_pwd_chg_new_title;
static lv_obj_t * g_admin_pwd_chg_new_content;
static lv_obj_t * g_admin_lbl_pwd_chg_new2;
static lv_obj_t * g_admin_ta_pwd_chg_new2;
static lv_obj_t * g_admin_pwd_chg_result;       /* 成功/失败全屏结果层 */
static lv_obj_t * g_admin_img_pwd_chg_result;
static lv_obj_t * g_admin_lbl_pwd_chg_result;
static char g_admin_pwd_chg_pending[ADMIN_PWD_LEN]; /* 页1确认后的新密码，供页2比对 */
static uint8_t g_admin_pwd_chg_page1_step;  /* 0=原密码框 1=新密码框 */
static bool g_admin_pwd_chg_result_ok;      /* 结果页：true=成功→menu2，false=失败→页1 */
static admin_factory_phase_t g_admin_factory_phase = ADMIN_FACTORY_PHASE_PROMPT;
static lv_timer_t * g_admin_factory_timer;
static admin_system_upgrade_phase_t g_admin_system_upgrade_phase = ADMIN_SYSTEM_UPGRADE_PHASE_PROMPT;
static lv_timer_t * g_admin_system_upgrade_timer;
static lv_obj_t * g_admin_prog_btns[TOTAL_PROGRAMS];
static lv_obj_t * g_admin_prog_btn_lbls[TOTAL_PROGRAMS];
static lv_obj_t * g_admin_prog_list;
static lv_obj_t * g_admin_prog_detail;
static lv_obj_t * g_admin_img_prog_title_box;
static lv_obj_t * g_admin_lbl_prog_title;
static bool g_admin_prog_is_detail;
static lv_obj_t * g_admin_prog_ta[4];           /* 初始烘干/追加次数/程序金额/追加金额 */
static lv_obj_t * g_admin_prog_roller[2];       /* 烘干温度/追加时间 */
static lv_obj_t * g_admin_prog_field_box[7];    /* 7 列统一白底外框 */
static lv_obj_t * g_admin_prog_cool_lbl;        /* 冷却时间只读 */
static lv_obj_t * g_admin_prog_dash[7];         /* 不可用参数显示 -- */
static lv_obj_t * g_admin_btn_prog_reset;
static lv_obj_t * g_admin_btn_prog_confirm;
static int32_t g_admin_prog_sel;
static lv_obj_t * g_admin_prog_field_lbl[7];    /* 程序设置 7 参数字段名 */
static lv_obj_t * g_admin_prog_total_val_lbl;     /* 上栏左侧：当前程序总时长（同主页底部） */
static lv_style_t s_admin_prog_btn_style;
static lv_style_t s_admin_prog_btn_sel_style;
static bool s_admin_prog_btn_style_inited;
static lv_obj_t * g_admin_ta_prog_active;       /* 程序设置页当前绑定的键盘输入框 */
static bool g_admin_prog_ui_loading;            /* load_fields 期间禁止 save，避免覆盖各程序默认值 */
static admin_view_t g_admin_view = PASSWORD;
static bool g_admin_unlocked;
static uint32_t g_machine_id = 1u;             /* 默认机器 ID，界面显示 000001 */
static char g_admin_pwd[ADMIN_PWD_LEN] = ADMIN_PWD_DEFAULT;  /* 管理员 6 位密码，可修改 */
static lv_style_t s_admin_menu_frame_style;   /* 外环：顶蓝底黑渐变，模拟渐变描边 */
static lv_style_t s_admin_menu_inner_style;   /* 内层：按钮深蓝渐变底 */
static bool s_admin_menu_style_inited;
static lv_style_t s_brightness_slider_frame_style;  /* 亮度滑条外框：左黑右蓝水平渐变描边 */
static lv_style_t s_brightness_slider_inner_style;  /* 亮度滑条内层：纯黑底 */
static bool s_brightness_slider_style_inited;
#define ADMIN_MENU_BORDER_W 2
#define ADMIN_MENU_BTN_W 210
#define ADMIN_MENU_BTN_H 168
#define ADMIN_MENU_BTN_GAP_X 175
#define ADMIN_MENU_BTN_GAP_Y 52
#define ADMIN_MENU_BTN_GRID_Y0 96
#define BRIGHT_PAGE_TRACK_BG    0x3A3A3A
#define BRIGHT_PAGE_FILL_0      0x000000
#define BRIGHT_PAGE_FILL_43     0x2F63F9
#define BRIGHT_PAGE_FILL_77     0x4FBEFF
#define BRIGHT_PAGE_FILL_100    0x4FBEFF
#define BRIGHT_PAGE_TRACK_H     35
#define BRIGHT_PAGE_KNOB_PAD    0
#define BRIGHT_PAGE_SIDE_PAD    10
#define BRIGHT_GRIP_LINE_W      20
#define BRIGHT_GRIP_LINE_H      2
#define BRIGHT_GRIP_LINE_GAP    3
static lv_grad_dsc_t s_bright_page_fill_grad;
#define BRIGHTNESS_SLIDER_BLUE  0x2a7fff  /* 渐变右端蓝色 */
#define BRIGHTNESS_SLIDER_BLACK 0x000000  /* 渐变左端黑色 */
static lv_font_t s_font_admin_kb;              /* SC_30 副本 + Montserrat 回退（图标键）键盘使用图标 */
static const lv_font_t * s_font_admin_kb_ptr;
static bool s_admin_kb_encoder_inited;

static void admin_kb_font_init(void);  //初始化管理员数字键盘字体（中文 + Montserrat 回退）
static bool admin_kb_is_visible(void);  //判断管理员数字键盘是否处于显示状态
static void admin_kb_encoder_style_init(void);  //为键盘内部按键设置编码器焦点描边并注册事件
static void admin_kb_encoder_enter(void);  //编码器进入键盘按键编辑：聚焦并选中首键
static void cb_admin_kb_encoder(lv_event_t * e);  //管理员键盘编码器：旋转逐步切换按键
static void admin_kb_close(void);  //收起管理员数字键盘并恢复焦点
static void cb_admin_kb_cancel(lv_event_t * e);  //键盘 CANCEL：小键盘图标关闭键盘
static void admin_ta_begin_edit(lv_obj_t * ta);  //弹出数字键盘并进入编码器按键编辑
static void cb_admin_ta_key_enter(lv_event_t * e);  //编码器 Enter：进入编辑，避免误触发 READY 校验
static void cb_admin_ta_kb_focus(lv_event_t * e);  //密码/机器 ID 输入框：弹出键盘并进入逐键选择
static void admin_encoder_group_add_kb(lv_group_t * group);  //将管理员键盘加入编码器 group（无外层描边）
static void admin_panel_show(admin_view_t view);  //切换管理员子面板显示并重建编码器焦点
static void admin_session_reset(void);  //退出管理员：清除解锁状态并回到密码页
static void admin_encoder_rebuild(void);  //按当前管理员子页重建编码器 focus 顺序
static void admin_group_edge_cb(lv_group_t * group, bool forward);  //menu1 第8钮右转进入 menu2
static void admin_group_focus_cb(lv_group_t * group);  //menu2 第1钮左转回到 menu1 第8钮
static void admin_machine_id_label_update(void);  //刷新机器 ID「当前 ID」标签

static void admin_dormancy_show_time_picker(void);
static void admin_dormancy_hide_time_picker(void);
static void admin_dormancy_sync_switches(void);
static void admin_payment_set_page(admin_payment_page_t page);
static void admin_payment_sync_list_ui(void);
static void admin_payment_sync_method_ui(void);
static void admin_payment_hide_time_picker(void);
static void admin_data_set_page(admin_data_page_t page);
static void admin_data_sync_upload_items_ui(void);
static void admin_data_sync_strategy_ui(void);
static void admin_wifi_ui_enter(void);
static void admin_wifi_set_phase(admin_wifi_phase_t phase);
static void admin_4g_ui_enter(void);
static void admin_4g_back_to_network(void);
static void admin_pwd_chg_hide_result(void);
static void admin_pwd_chg_page1_on_ready(void);
static void admin_pwd_chg_page2_on_ready(void);
static void cb_admin_machine_cancel(lv_event_t * e);
static void cb_admin_mid_success_click(lv_event_t * e);
static void admin_password_try(void);  //校验管理员密码（888888）并进入菜单
static bool admin_machine_id_apply(void);  //校验并保存 6 位机器 ID
static void admin_machine_id_back_to_menu1(void);  //离开机器 ID 页：清空输入并回 menu1
static void cb_admin_back(lv_event_t * e);  //管理员顶栏返回：子页回退或退出到主页
static void cb_load_admin(lv_event_t * e);  //主页管理员入口：进入管理员页
static void cb_admin_ta_ready(lv_event_t * e);  //输入框 READY：密码校验或保存机器 ID
static void cb_admin_open_machine_id(lv_event_t * e);  //菜单「机器 ID 设置」入口
static void cb_admin_machine_confirm(lv_event_t * e);  //机器 ID 确认：保存并回菜单
static void cb_admin_open_program_settings(lv_event_t * e);  //菜单「程序设置」入口
static void cb_admin_open_brightness(lv_event_t * e);  //菜单「屏幕亮度」入口
static void admin_brightness_back_to_menu1(void);  //离开屏幕亮度页：回 menu1
static void admin_brightness_sync_ui(void);  //屏幕亮度页：刷新开关与滑条
static void admin_brightness_apply_switch_layout(void);  //屏幕亮度页：按宏重新设开关宽高/样式/位置
static void admin_panel_style_switch(lv_obj_t * sw);  //管理员子页开关 OFF 白 / ON 橙样式
static int32_t admin_brightness_snap_slider(int32_t v);  //亮度滑条：对齐到步进（当前 1）
static void cb_admin_brightness_switch_changed(lv_event_t * e);  //常亮开关 VALUE_CHANGED
static void admin_brightness_slider_sync_bar(int32_t v);  //同步下层 lv_bar 动条显示
static void admin_brightness_slider_sync_focus_frame(void);  //同步上层焦点框与滑条焦点态
static void cb_admin_brightness_slider_focus_frame(lv_event_t * e);  //滑条获焦/失焦时刷新焦点框
static void cb_admin_brightness_slider_changed(lv_event_t * e);  //亮度滑条 VALUE_CHANGED
static void cb_admin_brightness_slider_encoder(lv_event_t * e);  //亮度滑条编码器短按切换编辑/导航
static void cb_admin_brightness_slider_key(lv_event_t * e);  //亮度滑条编码器编辑态：左右旋转按步进调值
static void admin_brightness_slider_exit_edit(lv_obj_t * slider);  //亮度滑条退出编码器编辑态
static void cb_admin_open_sound(lv_event_t * e);  //菜单「声音控制」入口
static void admin_sound_back_to_menu2(void);  //离开声音控制页：回 menu2
static void admin_sound_sync_ui(void);  //声音控制页：刷新开关与滑条
static void admin_sound_apply_switch_layout(lv_obj_t * sw, lv_obj_t * anchor_lbl);  //声音页：开关样式与对齐
static int32_t admin_sound_snap_volume_slider(int32_t v);  //音量滑条：对齐步进
static int32_t admin_sound_snap_touch_sound_volume_slider(int32_t v);  //触控声音滑条：对齐步进
static void cb_admin_sound_touch_switch_changed(lv_event_t * e);  //触控声音开关 VALUE_CHANGED
static void cb_admin_sound_voice_broadcast_switch_changed(lv_event_t * e);  //声音播报开关 VALUE_CHANGED
static void admin_sound_volume_slider_sync_bar(int32_t v);  //同步音量下层 bar
static void admin_sound_touch_sound_volume_slider_sync_bar(int32_t v);  //同步触控声音下层 bar
static void admin_sound_volume_slider_sync_focus_frame(void);  //同步音量滑条编码器白色焦点框
static void admin_sound_touch_sound_volume_slider_sync_focus_frame(void);  //同步触控声音滑条焦点框
static void cb_admin_sound_volume_slider_focus_frame(lv_event_t * e);  //音量滑条获焦/失焦时刷新焦点框
static void cb_admin_sound_touch_sound_volume_slider_focus_frame(lv_event_t * e);  //触控声音滑条获焦/失焦
static void cb_admin_sound_volume_slider_changed(lv_event_t * e);  //音量滑条 VALUE_CHANGED
static void cb_admin_sound_touch_sound_volume_slider_changed(lv_event_t * e);  //触控声音滑条 VALUE_CHANGED
static void cb_admin_sound_volume_slider_encoder(lv_event_t * e);  //音量滑条编码器短按切换编辑/导航
static void cb_admin_sound_touch_sound_volume_slider_encoder(lv_event_t * e);  //触控声音滑条编码器短按
static void cb_admin_sound_volume_slider_key(lv_event_t * e);  //音量滑条编码器编辑态左右旋转步进调值
static void cb_admin_sound_touch_sound_volume_slider_key(lv_event_t * e);  //触控声音滑条编码器编辑态调值
static void admin_sound_volume_slider_exit_edit(lv_obj_t * slider);  //音量滑条退出编码器编辑态
static void admin_sound_touch_sound_volume_slider_exit_edit(lv_obj_t * slider);  //触控声音滑条退出编辑态
static void admin_gradient_slider_fill_inner(lv_obj_t * frame, lv_obj_t ** out_bar, lv_obj_t ** out_slider,
                                             lv_obj_t ** out_focus);  //渐变滑条内层 bar/slider/focus
static void cb_admin_open_dormancy_standby(lv_event_t * e);  //菜单「待机时间」入口
static void cb_admin_dormancy_confirm(lv_event_t * e);  //待机时间确认：写入超时并回菜单
static void admin_lang_btn_set_selected(lv_obj_t * btn, bool selected);  //语言页按钮：填充/描边
static void admin_lang_sync_btn_ui(void);  //语言页按钮与 g_ui_lang 对齐
static void admin_lang_back_to_menu2(void);  //离开语言设置页：回 menu2
static void cb_admin_open_language_settings(lv_event_t * e);  //菜单「语言设置」入口
static void cb_admin_lang_zh(lv_event_t * e);  //语言页「中文」
static void cb_admin_lang_en(lv_event_t * e);  //语言页「英文」
static void admin_factory_set_phase(admin_factory_phase_t phase);  //恢复默认子页：切换确认/恢复中/完成 UI
static void admin_factory_reset_ui_enter(void);  //进入恢复默认页：复位为确认前态
static void admin_factory_back_to_menu2(void);  //离开恢复默认页：回 menu2
static void admin_factory_run_restore(void);  //执行出厂恢复：程序/待机/机器 ID 等
static void admin_factory_timer_stop(void);  //停止恢复默认页 2s 完成态定时器
static void cb_admin_factory_cancel(lv_event_t * e);  //恢复默认「取消」：回菜单
static void cb_admin_factory_ok(lv_event_t * e);  //恢复默认「确定」：写回出厂数据
static void cb_admin_factory_timer(lv_timer_t * t);  //2s 定时器：恢复中 → 完成态
static void cb_admin_open_factory_reset(lv_event_t * e);  //菜单「恢复默认」入口
static void admin_contact_back_to_menu2(void);  //离开联系我们页：回 menu2
static void cb_admin_open_contact_us(lv_event_t * e);  //菜单「联系我们」入口
static void admin_auto_dispense_back_to_menu1(void);  //离开防缠绕功能页：回 menu1
static void cb_admin_open_auto_dispense(lv_event_t * e);  //菜单「防缠绕功能」入口
static void cb_admin_auto_dispense_on(lv_event_t * e);  //防缠绕功能「开启」：ui_auto_dispense_set(true)
static void cb_admin_auto_dispense_off(lv_event_t * e);  //防缠绕功能「关闭」：ui_auto_dispense_set(false)
static void admin_fresh_air_care_sync_btn_ui(void);  //新风护理页：刷新开启/关闭按钮选中样式
static void admin_fresh_air_care_back_to_menu1(void);  //离开新风护理页：回 menu1
static void cb_admin_open_fresh_air_care(lv_event_t * e);  //菜单「新风护理」入口
static void cb_admin_fresh_air_care_on(lv_event_t * e);  //新风护理「开启」：ui_fresh_air_care_set(true)
static void cb_admin_fresh_air_care_off(lv_event_t * e);  //新风护理「关闭」：ui_fresh_air_care_set(false)
static void admin_system_upgrade_set_phase(admin_system_upgrade_phase_t phase);  //系统升级子页：切换确认/升级中/完成 UI
static void admin_system_upgrade_ui_enter(void);  //进入系统升级页：复位为确认前态
static void admin_system_upgrade_back_to_menu2(void);  //离开系统升级页：回 menu2
static void admin_system_upgrade_timer_stop(void);  //停止系统升级页 2s 完成态定时器
static void cb_admin_system_upgrade_ok(lv_event_t * e);  //系统升级「确认」：进入升级中 UI
static void cb_admin_system_upgrade_timer(lv_timer_t * t);  //2s 定时器：升级中 → 完成态
static void cb_admin_open_system_upgrade(lv_event_t * e);  //菜单「系统升级」入口
static void admin_4g_set_phase(admin_4g_phase_t phase);  //4G 设置子页：切换说明/配网中/成功 UI
static void admin_4g_ui_enter(void);  //进入 4G 设置页：复位为说明态并同步开关
static void admin_4g_back_to_menu1(void);  //离开 4G 设置页：回 menu1
static void admin_4g_timer_stop(void);  //停止 4G 配网 2s 完成态定时器
static void admin_4g_start_provisioning(void);  //4G 开关 OFF→ON：进入配网中并启动 2s 定时器
static void cb_admin_4g_switch_changed(lv_event_t * e);  //4G 开关切换：更新状态或触发配网流程
static void cb_admin_4g_timer(lv_timer_t * t);  //2s 定时器：配网中 → 成功态
static void cb_admin_open_network_settings(lv_event_t * e);  //menu1「网络设置」入口
static void cb_admin_open_wifi_settings(lv_event_t * e);  //网络设置「WIFI设置」入口
static void cb_admin_open_4g_settings(lv_event_t * e);  //网络设置「4G设置」入口
static void admin_network_back_to_menu1(void);  //离开网络设置页：回 menu1
static void admin_wifi_back_to_network(void);  //离开 WIFI 设置页：回网络设置
static void admin_pwd_chg_enter(void);  //密码修改入口：进入原密码校验页
static void admin_pwd_chg_old_try(void);  //密码修改步骤一：键盘 OK 校验原密码
static void admin_pwd_chg_show_new(void);  //密码修改步骤二：显示双新密码输入页
static void admin_pwd_chg_new_on_ready(void);  //密码修改步骤二：键盘 OK 切换框或保存
static void admin_pwd_chg_back_to_menu2(void);  //离开密码修改：回 menu2
static void cb_admin_open_password_change(lv_event_t * e);  //menu2「密码修改」入口
static void admin_payment_style_checkbox(lv_obj_t * cb);  //支付设置页：未选空心橙框，已选橙色对号
static uint32_t ui_payment_timeout_roller_index_from_sec(uint16_t sec);  //秒数 → roller 下标
static void admin_payment_timeout_roller_exit_edit(lv_obj_t * roller);  //支付超时 roller 退出编码器编辑态
static void cb_admin_payment_timeout_roller_encoder(lv_event_t * e);  //支付超时 roller 编码器短按切换编辑/导航
static void admin_payment_back_to_menu2(void);  //离开支付设置页：回 menu2
static void cb_admin_open_payment_settings(lv_event_t * e);  //menu2「支付设置」入口
static void cb_admin_payment_alipay_changed(lv_event_t * e);  //支付设置：支付宝复选框变更
static void cb_admin_payment_wechat_changed(lv_event_t * e);  //支付设置：微信复选框变更
static void admin_dormancy_sync_roller(void);
static void admin_dormancy_back_to_menu1(void);
static void cb_admin_dormancy_roller_changed(lv_event_t * e);
static void cb_admin_dormancy_roller_encoder(lv_event_t * e);
static void admin_data_back_to_menu1(void);
static void cb_admin_open_data_settings(lv_event_t * e);
static void admin_payment_add_divider(lv_obj_t * parent, lv_coord_t x);
static void brightness_slider_style_init(void);
static void admin_4g_apply_switch_layout(void);
static void admin_data_style_checkbox(lv_obj_t * cb);
static const ui_str_id_t g_data_upload_item_str_ids[7];
static const ui_str_id_t g_data_strategy_str_ids[5];
static void cb_admin_data_strategy_pressed(lv_event_t * e);
static void cb_admin_data_strategy_changed(lv_event_t * e);
static void ui_idle_apply_dormancy_period(void);  //按 g_ui_dormancy_timeout_ms 刷新空闲定时器周期
static const char * ui_dormancy_timeout_label(uint32_t ms);  //毫秒待机时长 → 界面显示文案（与待机 roller 选项一致）
static void cb_admin_prog_roller_encoder(lv_event_t * e);  //程序设置 roller 编码器短按切换编辑/导航
static lv_obj_t * make_admin_menu_btn(lv_obj_t * parent, const char * txt, const lv_image_dsc_t * icon);
static void cb_admin_menu_gesture(lv_event_t * e);
static void cb_admin_err_blocker(lv_event_t * e);
static void cb_admin_kb_btn(lv_event_t * e);
static lv_obj_t * admin_panel_apply_shell(lv_obj_t * panel, lv_obj_t * title_lbl);
static lv_obj_t * admin_menu_btn_get_label(lv_obj_t * btn);  //管理员菜单按钮内文字 label
static void admin_menu_btn_bind_i18n(lv_obj_t * btn, ui_str_id_t id);  //菜单按钮 label 绑定 i18n

static lv_coord_t s_mode_card_sz;
static lv_coord_t s_mode_step_x;       /* 拖动灵敏度基准（平均槽距） */
static lv_coord_t s_mode_gap_outer;    /* 第1-2、第4-5 之间间距（较窄） */
static lv_coord_t s_mode_gap_inner;    /* 第2-3、第3-4 之间间距（较宽） */
static lv_coord_t s_mode_row_shift_up;
static lv_coord_t s_mode_drag_snap_px;

static lv_obj_t * g_scr_off;			//待机页
static lv_obj_t * g_scr_home;			//主页
static lv_obj_t * g_scr_running;		//运行页
static lv_obj_t * g_scr_end;			//运行结束页
static lv_obj_t * g_scr_pay;			//支付页
static lv_obj_t * g_scr_pay_done;		//支付完成页
static lv_obj_t * g_scr_add_time;		//追加时间页
static lv_obj_t * g_lbl_clock;
static lv_obj_t * g_lbl_clock_running;
static lv_obj_t * g_lbl_clock_end;
static lv_obj_t * g_lbl_clock_pay;
static lv_obj_t * g_lbl_clock_pay_done;
static lv_obj_t * g_pay_btn_back;                /* 支付页返回，进支付后默认焦点避免误触启停 */
static bool g_pay_focus_back_on_enter;
static lv_obj_t * g_lbl_clock_alarm;
static lv_obj_t * g_lbl_clock_off;
static lv_obj_t * g_lbl_clock_off_center;          /* 待机页居中大字时钟 */
static lv_obj_t * g_off_btn_power;
static lv_obj_t * g_scr_before_off;                /* 空闲进待机前所在界面；NULL 表示开机首屏待机 */
static lv_timer_t * g_idle_timer;
static uint32_t g_ui_dormancy_timeout_ms = UI_DORMANCY_TIMEOUT_DEFAULT_MS;
static bool g_ui_dormancy_no_sleep = false;
static lv_coord_t g_idle_last_ptr_x = -1;
static lv_coord_t g_idle_last_ptr_y = -1;
static lv_obj_t * g_mode_carousel;
static lv_obj_t * g_mode_cards[CAROUSEL_VISIBLE_SLOTS];
static lv_obj_t * g_mode_card_imgs[CAROUSEL_VISIBLE_SLOTS];
static lv_obj_t * g_mode_card_labels[CAROUSEL_VISIBLE_SLOTS];
static lv_obj_t * g_mode_dots[TOTAL_PROGRAMS];
static lv_obj_t * g_lbl_home_time;
static lv_obj_t * g_lbl_home_temp;
static lv_obj_t * g_lbl_home_pay;
static lv_obj_t * g_lbl_home_lang;
static lv_obj_t * g_lbl_pay_price;
static lv_obj_t * g_lbl_pay_price_alipay;
static lv_obj_t * g_lbl_pay_price_wechat;
static lv_obj_t * g_lbl_pay_hint;
static lv_obj_t * g_lbl_pay_hint_alipay_l1;
static lv_obj_t * g_lbl_pay_hint_alipay_l2;
static lv_obj_t * g_lbl_pay_hint_wechat_l1;
static lv_obj_t * g_lbl_pay_hint_wechat_l2;
static lv_obj_t * g_lbl_pay_done;
static lv_obj_t * g_lbl_end_title;
static lv_obj_t * g_lbl_end_hint;
static lv_obj_t * g_alarm_overlay;          // lv_layer_top() 全屏报警弹层根
static lv_obj_t * g_alarm_panel_fluid;      // 缺液子面板
static lv_obj_t * g_alarm_img_warn;         // 缺液提示图
static lv_obj_t * g_alarm_lbl_hint;         // 缺液提示文字
static lv_obj_t * g_alarm_img_bar;          // 报警底栏 bar_01
static lv_obj_t * g_alarm_btn_back;         // 报警顶栏返回
static lv_obj_t * g_alarm_btn_runpause;     // 报警顶栏启停
static lv_obj_t * g_alarm_btn_power;        // 报警顶栏电源
static lv_obj_t * g_pay_single_col;
static lv_obj_t * g_pay_dual_row;
static lv_obj_t * g_pay_qr_img;
static lv_obj_t * g_pay_qr_alipay;
static lv_obj_t * g_pay_qr_wechat;

/* 双码支付页布局参数：调节左右两列间距与提示文字宽度 */
#define PAY_DUAL_COL_GAP   140   /* 支付宝列与微信列之间的水平间距 */
#define PAY_DUAL_HINT_W    450   /* 每列提示文字区域宽度，单行显示避免挤压换行 */
static lv_obj_t * g_kuaixi_run_lbl_status;
static lv_obj_t * g_kuaixi_run_lbl_stages;
static lv_obj_t * g_running_mode_label;   /* 运行页中间程序名，与 g_wheel_sel / 轮播一致 */
static lv_obj_t * g_running_time_label; /* 运行页倒计时标签 */
static lv_obj_t * g_running_lbl_temp_live;  /* 运行页实时温度 label */
static int16_t g_running_live_temp_c = 25;  /* 运行页实时温度 25为初始默认占位值*/
static lv_timer_t * g_running_countdown_timer;
static lv_timer_t * g_running_blink_timer;
static lv_timer_t * g_pay_done_timer;
static uint32_t g_running_remain_sec;
static bool g_running_countdown_paused;
static bool g_running_blink_visible;
static int32_t g_wheel_sel = MODE_IDX_DEFAULT; /* 0..TOTAL_PROGRAMS-1 */
int32_t* get_g_wheel_sel(void) { return &g_wheel_sel; }
static float g_wheel_turn = 0.f; /* fractional slot offset while dragging / animating */
static lv_coord_t g_wheel_press_x;
static bool g_wheel_dragging;
static bool g_wheel_anim_active;
static int32_t g_wheel_anim_commit_delta;
#define WHEEL_TURN_ANIM_SCALE 1000
#define WHEEL_ENC_ANIM_MS_PER_STEP 200u
#define WHEEL_SNAP_ANIM_MS 260u
/* 运行页童锁 */
static lv_obj_t * g_running_mid;                 /* 中间栏：童锁按钮对齐参考 */
static lv_obj_t * g_running_btn_back;
static lv_obj_t * g_running_btn_runpause;
static lv_obj_t * g_running_btn_power;
static lv_obj_t * g_running_child_lock_btn;
static lv_obj_t * g_running_child_lock_img;
static lv_obj_t * g_running_lock_blocker;      /* 全屏遮罩：童锁时拦截触摸 */
static bool g_ui_child_lock;                     /* 童锁激活时拦截主页滑动及除童锁外的界面跳转 */
static lv_obj_t * g_home_btn_runpause;           /* 主页顶栏启停（触摸） */
static lv_obj_t * g_home_btn_power;              /* 主页顶栏电源 */
static lv_obj_t * g_home_btn_admin;              /* 主页底栏管理员入口 */
static lv_obj_t * g_home_carousel_enc;           /* 轮播编码器：旋转选程序，短按等同启停 */

LV_IMAGE_DECLARE(img_01_Low_Temp);
LV_IMAGE_DECLARE(img_02_Med_Temp);
LV_IMAGE_DECLARE(img_03_High_Temp);
LV_IMAGE_DECLARE(img_04_Cool_Air);
LV_IMAGE_DECLARE(img_05_Air_Clean);

LV_IMAGE_DECLARE(QR_Alipay_01_Low_Temp);
LV_IMAGE_DECLARE(QR_Alipay_02_Med_Temp);
LV_IMAGE_DECLARE(QR_Alipay_03_High_Temp);
LV_IMAGE_DECLARE(QR_Alipay_04_Cool_Air);
LV_IMAGE_DECLARE(QR_Alipay_05_Air_Clean);
LV_IMAGE_DECLARE(QR_WeChat_01_Low_Temp);
LV_IMAGE_DECLARE(QR_WeChat_02_Med_Temp);
LV_IMAGE_DECLARE(QR_WeChat_03_High_Temp);
LV_IMAGE_DECLARE(QR_WeChat_04_Cool_Air);
LV_IMAGE_DECLARE(QR_WeChat_05_Air_Clean);
LV_IMAGE_DECLARE(QR_code_xiaoya);

LV_IMAGE_DECLARE(bar_01);
LV_IMAGE_DECLARE(lack_of_softener);
LV_IMAGE_DECLARE(lack_of_detergent);
LV_IMAGE_DECLARE(fault_logo);
LV_IMAGE_DECLARE(wifi_logo);
LV_IMAGE_DECLARE(child_lock_logo);
LV_IMAGE_DECLARE(title_bar);
LV_IMAGE_DECLARE(input_box);
LV_IMAGE_DECLARE(delete);
LV_IMAGE_DECLARE(enter);
LV_IMAGE_DECLARE(admin_button_box);
LV_IMAGE_DECLARE(id_set);
LV_IMAGE_DECLARE(net_set);
LV_IMAGE_DECLARE(data_set);
LV_IMAGE_DECLARE(program_set);
LV_IMAGE_DECLARE(standby_time_set);
LV_IMAGE_DECLARE(auto_put_set);
LV_IMAGE_DECLARE(fresh_air_set);
LV_IMAGE_DECLARE(bright_set);
LV_IMAGE_DECLARE(sound_set);
LV_IMAGE_DECLARE(language_set);
LV_IMAGE_DECLARE(upgrade_set);
LV_IMAGE_DECLARE(default_set);
LV_IMAGE_DECLARE(service_set);
LV_IMAGE_DECLARE(pay_set);
LV_IMAGE_DECLARE(password_set);
LV_IMAGE_DECLARE(title_box);
LV_IMAGE_DECLARE(set_box);
LV_IMAGE_DECLARE(bright_logo);
LV_IMAGE_DECLARE(touch_sound_logo);
LV_IMAGE_DECLARE(sound_logo);
LV_IMAGE_DECLARE(time_set_box);
LV_IMAGE_DECLARE(wechat_logo);
LV_IMAGE_DECLARE(alipay_logo);
LV_IMAGE_DECLARE(success);
LV_IMAGE_DECLARE(failure);
LV_IMAGE_DECLARE(washing_machine);

static const lv_image_dsc_t * const g_program_imgs[TOTAL_PROGRAMS] = {
    &img_01_Low_Temp, &img_02_Med_Temp, &img_03_High_Temp, &img_04_Cool_Air, &img_05_Air_Clean
};

static const lv_image_dsc_t * const g_alipay_qr_imgs[TOTAL_PROGRAMS] = {
    &QR_Alipay_01_Low_Temp, &QR_Alipay_02_Med_Temp, &QR_Alipay_03_High_Temp,
    &QR_Alipay_04_Cool_Air, &QR_Alipay_05_Air_Clean
};
static const lv_image_dsc_t * const g_wechat_qr_imgs[TOTAL_PROGRAMS] = {
    &QR_WeChat_01_Low_Temp, &QR_WeChat_02_Med_Temp, &QR_WeChat_03_High_Temp,
    &QR_WeChat_04_Cool_Air, &QR_WeChat_05_Air_Clean
};

/* 程序参数：与 g_mode_name_ids[] / g_program_imgs[] 下标 0=低温 … 4=风自洁 */
typedef struct {
    uint32_t wash_sec;       /* 主页底部：程序总运行时间（秒）= 烘干+冷却+追加 */
    const char * temp;
    int32_t price;           /* 默认金额；-1 表示无 */
} ui_program_profile_t;

typedef struct {
    uint32_t wash_sec;       /* 烘干阶段时长（秒）= 初始烘干 + 追加×次数；0 表示无烘干 */
    uint32_t rinse_sec;      /* 保留未用 */
    uint32_t spin_sec;       /* 打冷风/冷却阶段时长（秒） */
    ui_str_id_t stages_bar_id;
} ui_program_run_stages_t;

#define PROG_ADMIN_FIELD_CNT  7
#define PROG_ADMIN_LBL_BOX_GAP  12  /* 程序设置字段名与白框间距 */

/* 管理员「程序设置」扩展参数（表3.1 初值；时长按分钟存储/展示，运行倒计时 ×60 换算为秒） */
#define PROG_ADMIN_DEMO_SEC   0   /* 1=PC 演示用秒；0=按分钟×60 换算总时长 */
#define PROG_CAP_PRICE      0x0001u
#define PROG_CAP_ADD_PRICE  0x0002u
#define PROG_CAP_INIT_DRY   0x0004u
#define PROG_CAP_ADD_COUNT  0x0008u
#define PROG_CAP_ADD_TIME   0x0010u
#define PROG_CAP_TEMP       0x0020u
#define PROG_CAP_COOL       0x0040u

#define PROG_CAP_DRY_FULL   (PROG_CAP_PRICE | PROG_CAP_ADD_PRICE | PROG_CAP_INIT_DRY | \
                             PROG_CAP_TEMP | PROG_CAP_COOL | PROG_CAP_ADD_COUNT | PROG_CAP_ADD_TIME)
#define PROG_CAP_COLD_AIR   (PROG_CAP_PRICE | PROG_CAP_ADD_PRICE | PROG_CAP_COOL | \
                             PROG_CAP_ADD_COUNT | PROG_CAP_ADD_TIME)
#define PROG_CAP_AIR_CLEAN  (PROG_CAP_PRICE | PROG_CAP_COOL)  /* 风自洁无追加，追加金额显示 -- */

typedef struct {
    int32_t  price;          /* 程序金额（元）0-999 */
    int32_t  add_price;      /* 追加时间金额（元）0-999 */
    uint16_t init_dry_min;   /* 初始烘干时间（分钟 0-90） */
    uint8_t  add_count;      /* 追加次数 0-20 */
    uint16_t add_time_min;   /* 单次追加时间（分钟步进10） */
    int8_t   temp_idx;       /* 0/1/2；-1 表示无烘干温度 */
    uint8_t  cool_min;       /* 冷却时间（分钟，只读工厂值） */
    uint16_t cap;            /* 各参数是否可用（见 PROG_CAP_*） */
} ui_program_admin_t;

static ui_program_profile_t g_program_profiles[TOTAL_PROGRAMS];
static ui_program_run_stages_t g_program_run_stages[TOTAL_PROGRAMS];
static ui_program_admin_t g_prog_cfg[TOTAL_PROGRAMS];
static ui_program_admin_t g_prog_cfg_factory[TOTAL_PROGRAMS];

/* 程序设置 UI 上下文（管理员 / 循环程序共用一套 save/load 逻辑） */
typedef struct {
	ui_program_admin_t * cfg_tbl;
	int32_t sel;
	bool ui_loading;
	lv_obj_t * btns[TOTAL_PROGRAMS];
	lv_obj_t * ta[4];
	lv_obj_t * roller[2];
	lv_obj_t * field_box[7];
	lv_obj_t * cool_lbl;
	lv_obj_t * dash[7];
	lv_obj_t * field_lbl[7];
	lv_obj_t * total_val_lbl;
} prog_ui_ctx_t;

static prog_ui_ctx_t g_pui_admin;
static prog_ui_ctx_t g_pui_cycle;

typedef struct {
	int32_t prog_idx;
	ui_program_admin_t cfg;
	uint8_t add_sel;
	uint32_t run_count;
	uint32_t fault_at_count; // 故障发生时的已完成轮次
	uint8_t fault_code;
} cycle_session_t;

static cycle_session_t g_cycle_session;
static ui_program_admin_t g_cycle_cfg[TOTAL_PROGRAMS];

/* 追加时间页：当前选择值与会话（确定后用于支付/运行） */
static uint8_t g_add_time_sel;//追加次数
static uint8_t g_session_add_count;//会话追加次数
static int32_t g_session_total_price;//会话总金额
static uint32_t g_session_total_sec;//会话总时长
static ui_program_run_stages_t g_session_run_stages;//会话运行分段
static bool g_session_active;//会话是否激活

static lv_obj_t * g_add_time_btn_back;//追加页返回按钮
static lv_obj_t * g_add_time_btn_runpause;//追加页启停按钮
static lv_obj_t * g_add_time_btn_power;//追加页电源按钮
static lv_obj_t * g_lbl_clock_add_time;//追加页顶部时钟标签
static lv_obj_t * g_add_time_mid;//追加页中间栏
static lv_obj_t * g_add_time_lbl_title;//追加页标题标签
static lv_obj_t * g_add_time_lbl_prog_name;//追加页程序名称标签
static lv_obj_t * g_add_time_lbl_total_price;//追加页总金额标签
static lv_obj_t * g_add_time_lbl_total_time;//追加页总时长标签
static lv_obj_t * g_add_time_lbl_count_val;//追加页次数标签
static lv_obj_t * g_add_time_btn_up;//追加页次数+1按钮
static lv_obj_t * g_add_time_btn_down;//追加页次数-1按钮

static void program_admin_init_factory(void);  //写入表3.1 程序初值并同步到主页/运行页
static void program_admin_apply_one(int32_t idx);  //将单程序 cfg 同步到主页/运行页参数表
static void program_admin_apply_all(void);  //将全部程序 cfg 同步到主页/运行页
static int16_t program_admin_temp_celsius(int32_t prog_idx, int8_t temp_idx);  //按程序与档位取烘干温度 ℃
static void program_admin_ui_save_fields(void);  //从程序设置 UI 控件写回 g_prog_cfg
static void program_admin_ui_load_fields(void);  //将 g_prog_cfg 加载到程序设置 UI 控件
static void program_admin_ui_apply_caps(void);  //按 cap 显示/隐藏程序设置各参数字段
static void prog_ui_apply_caps(prog_ui_ctx_t * pui);
static void prog_ui_sync_prog_pick_ui(prog_ui_ctx_t * pui);
static void prog_ui_save_fields(prog_ui_ctx_t * pui);
static void prog_ui_load_fields(prog_ui_ctx_t * pui);
static void prog_ui_build_cycle_program_panel(lv_obj_t * panel, lv_coord_t panel_h);
static void program_admin_build_panel(lv_obj_t * root, lv_coord_t body_y, lv_coord_t body_h);  //构建程序设置子面板
static void program_admin_back_to_menu1(void);  //离开程序设置：保存并回 menu1
static void program_admin_sync_prog_pick_ui(void);  //刷新程序选择上栏按钮选中样式
static void program_admin_update_total_display(void);  //刷新程序设置上栏总时长显示
static lv_obj_t * make_admin_prog_btn(lv_obj_t * parent, const char * txt);  //创建程序设置上栏程序按钮

/* ========== ui.c 功能函数前向声明 ========== */
static void ui_send_beep_seq(int32_t seq);  //向蜂鸣器任务发送音效序号（RTOS 队列；PC 仿真为空实现）
static void ui_apply_chinese_font(void);  //绑定中文字体并应用到 LVGL 默认主题
static lv_obj_t * make_text_btn(lv_obj_t * parent, const char * txt, lv_coord_t w, lv_coord_t h);  //创建透明背景文本按钮
static lv_obj_t * make_orange_fill_btn(lv_obj_t * parent, const char * txt, lv_coord_t w, lv_coord_t h);  //创建橙色实心圆角按钮
static lv_obj_t * make_orange_outline_btn(lv_obj_t * parent, const char * txt, lv_coord_t w, lv_coord_t h);  //创建橙色描边圆角按钮
static void cb_clock(lv_timer_t * t);  //定时器回调：每秒更新各界面顶部时钟标签
static void create_top_status_bar(lv_obj_t * top, lv_obj_t ** clock_lbl_out);  //创建顶部右侧状态栏（4G / WiFi / 时间）
static lv_obj_t * create_top_bar(lv_obj_t * parent, lv_obj_t ** clock_lbl_out,
    lv_obj_t * back_target, lv_obj_t ** back_btn_out);  //创建顶部栏：可选返回键 + 状态栏
static lv_obj_t * add_top_text_btn(lv_obj_t * top, const char * txt, lv_coord_t x);  //在顶部栏添加启停/电源类文本按钮
static lv_obj_t * make_top_back_btn(lv_obj_t * top, lv_event_cb_t cb);  //顶栏透明返回钮（自定义回调）
static void carousel_size_cb(lv_event_t * e);  //轮播区尺寸变化时重新布局
static void carousel_update_card_images(void);  //按当前选中程序刷新 5 张可见卡片图与名称标签
static lv_coord_t carousel_rel_offset_x(float rel);
static lv_coord_t carousel_slot_center_x(int slot);  //计算槽位相对轮播中心的水平偏移
static void carousel_wrap_relayout(void);  //轮播区整体布局：卡片位置/缩放/透明度与指示点
static int32_t carousel_dot_preview_sel(void);
static int32_t wheel_clamp_sel(int32_t v);
static int32_t wheel_mod_total(int32_t v);  //程序索引在 0..TOTAL_PROGRAMS-1 内循环取模
static void carousel_anim_stop(bool commit);
static void carousel_anim_to_turn(float target_turn, int32_t commit_delta, uint32_t duration_ms);
static void home_sync_encoder_focus_after_carousel_drag(void);  //触摸滑动改程序后对齐编码器焦点
static void home_encoder_group_build(void);  //主页编码器：仅轮播（常驻 editing）
static void cb_home_carousel_encoder(lv_event_t * e);  //轮播编码器：旋转选程序，短按进支付/追加
static void cb_home_runpause(lv_event_t * e);  //主页启停触摸点击
static void cb_home_prog_dot_focus(lv_event_t * e);  //轮播指示点：触摸点击同步选中程序
static void wheel_pointer_cb(lv_event_t * e);  //轮播区按下/拖动/释放：滑动切换程序
static void home_enter_pay_screen_request(void);  //延迟进入支付页（含二维码），避免误进支付完成
static void cb_load_screen(lv_event_t * e);  //通用界面加载事件回调，根据 user_data 切换屏幕
static void ui_layout_init(void);  //初始化轮播区布局参数（卡片尺寸、间距、拖动灵敏度）
static void style_screen_base(lv_obj_t * scr);  //设置屏幕基础样式（黑底、无边框）
static void style_obj_encoder_focus_outline_color(lv_obj_t * obj, uint32_t color_hex, lv_opa_t opa);  //为控件设置指定颜色的编码器焦点描边
static void style_obj_encoder_focus_outline(lv_obj_t * obj);  //为控件设置编码器焦点描边（默认白 50%）
static void ui_encoder_group_add(lv_group_t * group, lv_obj_t * obj);  //控件创建后立即加入编码器 group
static void style_prog_field_encoder_focus_inner(lv_obj_t * obj);  //程序设置：内贴橙框
static void ui_encoder_group_add_prog_field(lv_group_t * group, lv_obj_t * obj);  //程序设置字段加入 group（内贴橙框）
static void ui_encoder_group_add_no_outline(lv_group_t * group, lv_obj_t * obj);  //加入 group 但不显示焦点框
static void style_brightness_slider_encoder_focus_inner(lv_obj_t * obj);
static void ui_encoder_group_add_brightness_slider(lv_group_t * group, lv_obj_t * obj);

static void create_screens(void);  //创建全部屏幕对象并设固定分辨率
static void ui_set_encoder_group(lv_group_t * group);  //将编码器/键盘输入设备绑定到指定 focus group
static void ui_screen_load(lv_obj_t * scr);  //屏幕加载包装：清童锁/倒计时，切换 group 与页面逻辑
static void ui_idle_reset(void);  //重置空闲计时（有输入时调用）
static void ui_idle_on_screen_changed(lv_obj_t * scr);  //进入/离开待机页时暂停或恢复空闲计时
static void ui_idle_poll_pointer(void);  //检测鼠标移动并重置空闲计时
static void cb_idle_timeout(lv_timer_t * t);  //空闲超时：进入待机页
static void cb_indev_activity(lv_event_t * e);  //输入设备活动：重置空闲计时
static void ui_idle_indev_hook(void);  //为鼠标/编码器注册活动监听
static void ui_idle_init(void);  //创建空闲计时器并注册输入监听
static void cb_off_wake(lv_event_t * e);  //待机页触摸/点击：恢复待机前界面（首屏待机则进主页）
static void ui_apply_indev_long_press_ms(uint16_t ms);  //统一设置指针/编码器长按判定时间
static void running_child_lock_on_enter(void);  //童锁激活时的进入钩子（预留扩展）
static void running_child_lock_on_exit(void);  //童锁解除时的退出钩子（预留扩展）
static void running_child_lock_align_btn(void);  //将童锁按钮对齐到运行页中间栏右侧
static void running_child_lock_apply_locked(bool locked, bool silent_unlock);  //应用童锁锁定/解锁 UI（不操作编码器 group）
static void running_child_lock_remote_clear_if_leaving_running(lv_obj_t * target_scr);  //离开运行页时静默解除童锁
static void cb_running_child_lock_long(lv_event_t * e);  //童锁按钮长按：切换童锁开/关
static void cb_running_child_lock_released(lv_event_t * e);  //童锁松开：焦点回到童锁按钮
static void build_home(void);  //构建主页：顶部栏、程序轮播、底部参数栏
static const ui_program_profile_t * program_profile_get(int32_t idx);  //获取指定程序索引的参数表项
static void program_format_time_label(uint32_t sec, char * buf, size_t buf_sz);  //格式化为首页时间标签文本
static void program_format_price_home(int32_t price, char * buf, size_t buf_sz);  //格式化为首页价格标签
static void program_format_price_pay(int32_t price, char * buf, size_t buf_sz);  //格式化为支付页价格标签
static void home_sync_program_labels(void);  //按当前选中程序刷新主页底部时间/温度/价格标签
static void pay_sync_price_label(void);  //按当前选中程序刷新支付页金额标签
static void pay_sync_pay_ui(void);  //按支付方式与程序刷新支付页二维码与提示布局
static uint8_t add_time_max_count(int32_t idx);  //追加时间页可选次数上限（0~程序设置 add_count）
static int32_t add_time_calc_price(const ui_program_admin_t * c, uint8_t sel);  //总金额=程序金额+追加金额×次数
static uint32_t add_time_calc_total_sec(const ui_program_admin_t * c, uint8_t sel);  //总时间=初始烘干+次数×追加时间+冷却
static void add_time_calc_run_stages(const ui_program_admin_t * c, uint8_t sel, ui_program_run_stages_t * out);  //会话运行分段
static void add_time_session_commit(uint8_t sel);  //锁定本次订单追加次数与金额/时长
static void add_time_session_clear(void);  //清除支付/运行会话
static void add_time_sel_step(int8_t delta);  //追加次数步进（仅上下按钮）
static void add_time_page_sync_labels(void);  //刷新追加时间页全部显示
static void add_time_encoder_group_build(void);  //重建追加时间页编码器焦点顺序
static void build_add_time(void);  //构建追加时间页
static void cb_add_time_back(lv_event_t * e);  //追加页返回：清会话并回主页（原取消逻辑）
static void cb_add_time_btn_up(lv_event_t * e);  //追加次数 +1
static void cb_add_time_btn_down(lv_event_t * e);  //追加次数 -1
static void cb_add_time_runpause(lv_event_t * e);  //追加页启停：提交会话并进支付（原确定逻辑）
static void cb_add_time_power_stub(lv_event_t * e);  //追加页电源预留空回调
static const ui_program_run_stages_t * running_program_stages_get(int32_t idx);  //运行页分段（会话优先）
static void running_screen_sync_mode_name(void);  //运行页程序名标签与 g_wheel_sel 保持一致
static uint32_t running_program_total_sec(int32_t idx);  //运行总时长（洗涤+漂洗×次数+脱水）
static void running_status_sync_labels(void);  //按程序与剩余时间刷新运行页底部左右文案
static void running_countdown_format(uint32_t sec, char * buf, size_t buf_sz);  //将剩余秒数格式化为 0:MM
static void running_countdown_update_label(void);  //刷新运行页倒计时标签显示
static void pay_done_timer_stop(void);  //停止支付完成页 2 秒自动跳转定时器
static void cb_pay_done_timer(lv_timer_t * t);  //支付完成页定时器回调：2 秒后进入运行页
static void pay_done_timer_start(void);  //启动支付完成页 2 秒自动跳转定时器
static void running_blink_stop(void);  //停止暂停时倒计时闪烁定时器并恢复标签可见
static void cb_running_blink(lv_timer_t * t);  //暂停时倒计时标签 500ms 闪烁定时器回调
static void running_countdown_stop(void);  //停止运行页倒计时定时器
static void running_countdown_reset_all(void);  //重置倒计时状态：清除暂停、闪烁与定时器
static void running_countdown_pause(void);  //暂停倒计时并启动时间标签闪烁
static void running_countdown_resume(void);  //恢复倒计时并停止闪烁
static void cb_running_runpause(lv_event_t * e);  //运行页启停按钮：暂停/恢复倒计时
static void cb_running_countdown(lv_timer_t * t);  //运行页每分钟倒计时回调，归零后跳转结束页
static void running_countdown_arm(void);  //若剩余时间>0 则启动分钟倒计时定时器
static void running_countdown_start(void);  //进入运行页时按程序时长初始化并开始倒计时
static void running_live_params_sync(void);  //刷新运行页实时温度 label
static void build_running(void);  //构建运行页：背景、顶部栏、程序名/倒计时、童锁
static void build_end(void);  //构建洗涤结束页：返回、图标与提示文字
static void admin_dormancy_sync_switches(void)
{
    if(g_admin_dormancy_sw_time == NULL || g_admin_dormancy_sw_no_sleep == NULL) return;
    g_admin_dormancy_ui_loading = true;
    bool time_on = (g_ui_dormancy_timeout_ms != UI_DORMANCY_DISABLED_MS);
    if(time_on) {
        lv_obj_add_state(g_admin_dormancy_sw_time, LV_STATE_CHECKED);
    }
    else {
        lv_obj_remove_state(g_admin_dormancy_sw_time, LV_STATE_CHECKED);
    }
    if(g_ui_dormancy_no_sleep) {
        lv_obj_add_state(g_admin_dormancy_sw_no_sleep, LV_STATE_CHECKED);
    }
    else {
        lv_obj_remove_state(g_admin_dormancy_sw_no_sleep, LV_STATE_CHECKED);
    }
    g_admin_dormancy_ui_loading = false;
}

static void cb_admin_dormancy_sw_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(g_admin_view != DORMANCY_STANDBY) return;
    if(g_admin_dormancy_ui_loading) return;
    lv_obj_t * sw = lv_event_get_target_obj(e);
    if(sw == NULL) return;
    bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);

    g_admin_dormancy_ui_loading = true;

    if(sw == g_admin_dormancy_sw_time) {
        if(on) {
            /* 情况1：打开时间设置 → 关不息屏，进时间选择子页 */
            g_ui_dormancy_timeout_ms = UI_DORMANCY_TIMEOUT_DEFAULT_MS;
            g_ui_dormancy_no_sleep   = false;
            lv_obj_remove_state(g_admin_dormancy_sw_no_sleep, LV_STATE_CHECKED);
            admin_dormancy_show_time_picker();
        } else {
            /* 情况2：关闭时间设置 → 开不息屏 */
            g_ui_dormancy_timeout_ms = UI_DORMANCY_DISABLED_MS;
            g_ui_dormancy_no_sleep   = true;
            lv_obj_add_state(g_admin_dormancy_sw_no_sleep, LV_STATE_CHECKED);
        }
    }
    else if(sw == g_admin_dormancy_sw_no_sleep) {
        if(on) {
            /* 情况3：打开不息屏 → 关时间设置 */
            g_ui_dormancy_no_sleep   = true;
            g_ui_dormancy_timeout_ms = UI_DORMANCY_DISABLED_MS;
            lv_obj_remove_state(g_admin_dormancy_sw_time, LV_STATE_CHECKED);
        } else {
            /* 情况4：关闭不息屏 → 开时间设置（默认5分钟，不跳转子页） */
            g_ui_dormancy_no_sleep   = false;
            g_ui_dormancy_timeout_ms = UI_DORMANCY_TIMEOUT_DEFAULT_MS;
            lv_obj_add_state(g_admin_dormancy_sw_time, LV_STATE_CHECKED);
        }
    }

    g_admin_dormancy_ui_loading = false;
    ui_idle_apply_dormancy_period();
}

/* 将分钟/秒刷新到 4 个数字滚动条（MM:SS，均 ≤59） */
static void admin_dormancy_tp_sync_digits(void)
{
    uint32_t m = g_admin_dormancy_tp_minutes;
    uint32_t s = g_admin_dormancy_tp_seconds;
    if(m > TP_MMSS_MAX_MIN) m = TP_MMSS_MAX_MIN;
    if(s > TP_MMSS_MAX_MIN) s = TP_MMSS_MAX_MIN;
    if(g_admin_dormancy_tp_digit_rollers[0] != NULL)
        lv_roller_set_selected(g_admin_dormancy_tp_digit_rollers[0], m / 10, LV_ANIM_OFF);
    if(g_admin_dormancy_tp_digit_rollers[1] != NULL)
        lv_roller_set_selected(g_admin_dormancy_tp_digit_rollers[1], m % 10, LV_ANIM_OFF);
    if(g_admin_dormancy_tp_digit_rollers[2] != NULL)
        lv_roller_set_selected(g_admin_dormancy_tp_digit_rollers[2], s / 10, LV_ANIM_OFF);
    if(g_admin_dormancy_tp_digit_rollers[3] != NULL)
        lv_roller_set_selected(g_admin_dormancy_tp_digit_rollers[3], s % 10, LV_ANIM_OFF);
}

/* 显示时间选择子页 */
static void admin_dormancy_show_time_picker(void)
{
    if(g_admin_dormancy_tp_wrap == NULL) return;

    uint32_t ms = g_ui_dormancy_timeout_ms;
    if(ms == UI_DORMANCY_DISABLED_MS) ms = UI_DORMANCY_TIMEOUT_DEFAULT_MS;
    uint32_t total_sec = ms / 1000u;
    if(total_sec < 1) total_sec = 1;
    if(total_sec > PAYMENT_TIMEOUT_SEC_MAX) total_sec = PAYMENT_TIMEOUT_SEC_MAX;
    g_admin_dormancy_tp_minutes = total_sec / 60u;
    g_admin_dormancy_tp_seconds = total_sec % 60u;

    admin_dormancy_tp_sync_digits();

    if(g_admin_dormancy_set_box_wrap != NULL)
        lv_obj_add_flag(g_admin_dormancy_set_box_wrap, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_img_dormancy_title_box != NULL)
        lv_obj_add_flag(g_admin_img_dormancy_title_box, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_lbl_dormancy_title != NULL)
        lv_obj_add_flag(g_admin_lbl_dormancy_title, LV_OBJ_FLAG_HIDDEN);

    lv_obj_remove_flag(g_admin_dormancy_tp_wrap, LV_OBJ_FLAG_HIDDEN);
    g_admin_dormancy_tp_active = true;

}

/* 隐藏时间选择子页，回到开关页 */
static void admin_dormancy_hide_time_picker(void)
{
    if(g_admin_dormancy_tp_wrap == NULL) return;

    lv_obj_add_flag(g_admin_dormancy_tp_wrap, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_dormancy_set_box_wrap != NULL)
        lv_obj_remove_flag(g_admin_dormancy_set_box_wrap, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_img_dormancy_title_box != NULL)
        lv_obj_remove_flag(g_admin_img_dormancy_title_box, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_lbl_dormancy_title != NULL)
        lv_obj_remove_flag(g_admin_lbl_dormancy_title, LV_OBJ_FLAG_HIDDEN);

    g_admin_dormancy_tp_active = false;

}

/* 确定：应用选中时间（MM:SS） */
static void cb_admin_dormancy_tp_confirm(lv_event_t * e)
{
    (void)e;
    if(!g_admin_dormancy_tp_active) return;
    uint32_t total_sec = g_admin_dormancy_tp_minutes * 60u + g_admin_dormancy_tp_seconds;
    if(total_sec < 1) total_sec = 1;
    if(total_sec > PAYMENT_TIMEOUT_SEC_MAX) total_sec = PAYMENT_TIMEOUT_SEC_MAX;
    g_ui_dormancy_timeout_ms = total_sec * 1000u;
    ui_idle_apply_dormancy_period();
    admin_dormancy_hide_time_picker();
}

/* 取消：不保存，回到开关页 */
static void cb_admin_dormancy_tp_cancel(lv_event_t * e)
{
    (void)e;
    if(!g_admin_dormancy_tp_active) return;
    admin_dormancy_hide_time_picker();
}

/* 任意数字滚动条变化 → 重新计算 MM:SS（最大各 59） */
static void cb_admin_dormancy_tp_digit_roller_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(!g_admin_dormancy_tp_active) return;
    uint32_t m = 0;
    uint32_t s = 0;
    if(g_admin_dormancy_tp_digit_rollers[0] != NULL)
        m += lv_roller_get_selected(g_admin_dormancy_tp_digit_rollers[0]) * 10;
    if(g_admin_dormancy_tp_digit_rollers[1] != NULL)
        m += lv_roller_get_selected(g_admin_dormancy_tp_digit_rollers[1]);
    if(g_admin_dormancy_tp_digit_rollers[2] != NULL)
        s += lv_roller_get_selected(g_admin_dormancy_tp_digit_rollers[2]) * 10;
    if(g_admin_dormancy_tp_digit_rollers[3] != NULL)
        s += lv_roller_get_selected(g_admin_dormancy_tp_digit_rollers[3]);
    if(m > TP_MMSS_MAX_MIN) m = TP_MMSS_MAX_MIN;
    if(s > TP_MMSS_MAX_MIN) s = TP_MMSS_MAX_MIN;
    g_admin_dormancy_tp_minutes = m;
    g_admin_dormancy_tp_seconds = s;
}

/*
 * 亮度样式滑条：裁剪窗宽度跟随数值；内层渐变按整轨固定铺色。
 * fill / fill_grad / slider 成套使用（亮度页、声音页共用）。
 */
static void admin_bright_style_fill_sync(lv_obj_t * fill, lv_obj_t * fill_grad, lv_obj_t * slider)
{
    if(fill == NULL || slider == NULL) return;

    lv_obj_update_layout(slider);
    const int32_t track_w = lv_obj_get_width(slider);
    if(track_w <= 0) {
        lv_obj_set_width(fill, 0);
        return;
    }

    if(fill_grad != NULL) {
        lv_obj_set_width(fill_grad, track_w);
        lv_obj_align(fill_grad, LV_ALIGN_LEFT_MID, 0, 0);
    }

    const int32_t min_v = lv_slider_get_min_value(slider);
    const int32_t max_v = lv_slider_get_max_value(slider);
    const int32_t v = lv_slider_get_value(slider);
    if(v <= min_v) {
        lv_obj_set_width(fill, 0);
        return;
    }

    const int32_t half = BRIGHT_PAGE_TRACK_H / 2;
    const int32_t x_min = half;
    const int32_t x_max = track_w - half;
    int32_t range = max_v - min_v;
    if(range <= 0) range = 1;

    const int32_t knob_cx = x_min + (int32_t)(((int64_t)(x_max - x_min) * (v - min_v)) / range);
    int32_t fill_w = knob_cx + half;
    if(fill_w > track_w) fill_w = track_w;
    if(fill_w < 0) fill_w = 0;
    lv_obj_set_width(fill, fill_w);
}

static void admin_brightness_fill_sync(void)
{
    admin_bright_style_fill_sync(g_admin_brightness_fill, g_admin_brightness_fill_grad,
                                 g_admin_slider_brightness);
}

/* 在已布局好的 area 内创建亮度样式滑条（灰轨 + 渐变裁剪 + 透明轨旋钮） */
static void admin_bright_style_slider_build(lv_obj_t * area,
                                           lv_obj_t ** out_fill,
                                           lv_obj_t ** out_fill_grad,
                                           lv_obj_t ** out_slider,
                                           lv_event_cb_t value_changed_cb)
{
    if(area == NULL || out_fill == NULL || out_fill_grad == NULL || out_slider == NULL) return;

    lv_obj_t * track = lv_obj_create(area);
    lv_obj_set_size(track, LV_PCT(100), BRIGHT_PAGE_TRACK_H);
    lv_obj_align(track, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(track, lv_color_hex(BRIGHT_PAGE_TRACK_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(track, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(track, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(track, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(track, 0, LV_PART_MAIN);
    lv_obj_clear_flag(track, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    *out_fill = lv_obj_create(area);
    lv_obj_set_size(*out_fill, 0, BRIGHT_PAGE_TRACK_H);
    lv_obj_align(*out_fill, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_opa(*out_fill, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_radius(*out_fill, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_clip_corner(*out_fill, true, LV_PART_MAIN);
    lv_obj_set_style_border_width(*out_fill, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(*out_fill, 0, LV_PART_MAIN);
    lv_obj_clear_flag(*out_fill, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    *out_fill_grad = lv_obj_create(*out_fill);
    lv_obj_set_size(*out_fill_grad, LV_PCT(100), BRIGHT_PAGE_TRACK_H);
    lv_obj_align(*out_fill_grad, LV_ALIGN_LEFT_MID, 0, 0);
    {
        static bool s_grad_ready;
        if(!s_grad_ready) {
            const lv_color_t stops_c[] = {
                lv_color_hex(BRIGHT_PAGE_FILL_0),
                lv_color_hex(BRIGHT_PAGE_FILL_43),
                lv_color_hex(BRIGHT_PAGE_FILL_77),
                lv_color_hex(BRIGHT_PAGE_FILL_100),
            };
            const uint8_t stops_frac[] = {
                0,
                (uint8_t)(255 * 43 / 100),
                (uint8_t)(255 * 77 / 100),
                255,
            };
            lv_grad_init_stops(&s_bright_page_fill_grad, stops_c, NULL, stops_frac, 4);
            lv_grad_horizontal_init(&s_bright_page_fill_grad);
            s_grad_ready = true;
        }
        lv_obj_set_style_bg_grad(*out_fill_grad, &s_bright_page_fill_grad, LV_PART_MAIN);
    }
    lv_obj_set_style_bg_opa(*out_fill_grad, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(*out_fill_grad, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(*out_fill_grad, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(*out_fill_grad, 0, LV_PART_MAIN);
    lv_obj_clear_flag(*out_fill_grad, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    *out_slider = lv_slider_create(area);
    lv_obj_set_size(*out_slider, LV_PCT(100), BRIGHT_PAGE_TRACK_H);
    lv_obj_align(*out_slider, LV_ALIGN_LEFT_MID, 0, 0);
    lv_slider_set_range(*out_slider, 0, 100);
    lv_obj_set_style_pad_hor(*out_slider, BRIGHT_PAGE_TRACK_H / 2, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(*out_slider, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(*out_slider, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_radius(*out_slider, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(*out_slider, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(*out_slider, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_pad_all(*out_slider, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(*out_slider, lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    lv_obj_set_style_bg_opa(*out_slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_width(*out_slider, 0, LV_PART_KNOB);
    lv_obj_set_style_radius(*out_slider, LV_RADIUS_CIRCLE, LV_PART_KNOB);
    lv_obj_set_style_pad_all(*out_slider, BRIGHT_PAGE_KNOB_PAD, LV_PART_KNOB);
    lv_obj_set_style_shadow_width(*out_slider, 0, LV_PART_KNOB);

    lv_obj_add_flag(*out_slider, LV_OBJ_FLAG_SEND_DRAW_TASK_EVENTS | LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_obj_add_event_cb(*out_slider, cb_admin_brightness_slider_draw_grip, LV_EVENT_DRAW_TASK_ADDED, NULL);
    lv_obj_add_event_cb(*out_slider, cb_admin_brightness_slider_ext_draw_size, LV_EVENT_REFR_EXT_DRAW_SIZE, NULL);
    if(value_changed_cb != NULL) {
        lv_obj_add_event_cb(*out_slider, value_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    }
    lv_obj_refresh_ext_draw_size(*out_slider);
}

/*
 * 亮度滑条：在旋钮 FILL 上叠加三道握纹（与旋钮同次绘制，位置一致）。
 */
static void cb_admin_brightness_slider_draw_grip(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_DRAW_TASK_ADDED) return;

    static bool s_bright_draw_reenter;
    if(s_bright_draw_reenter) return;

    lv_draw_task_t * task = lv_event_get_draw_task(e);
    if(task == NULL || lv_draw_task_get_type(task) != LV_DRAW_TASK_TYPE_FILL) return;

    lv_draw_dsc_base_t * base = (lv_draw_dsc_base_t *)lv_draw_task_get_draw_dsc(task);
    if(base == NULL || base->part != LV_PART_KNOB || base->layer == NULL) return;

    lv_area_t area;
    lv_draw_task_get_area(task, &area);
    const int32_t cx = (area.x1 + area.x2) / 2;
    const int32_t cy = (area.y1 + area.y2) / 2;
    const int32_t line_w = BRIGHT_GRIP_LINE_W;
    const int32_t line_h = BRIGHT_GRIP_LINE_H;
    const int32_t line_gap = BRIGHT_GRIP_LINE_GAP;
    const int32_t total_h = line_h * 3 + line_gap * 2;
    const int32_t y0 = cy - total_h / 2;

    lv_draw_rect_dsc_t line_dsc;
    lv_draw_rect_dsc_init(&line_dsc);
    line_dsc.bg_color = lv_color_hex(COL_DIM);
    line_dsc.bg_opa = LV_OPA_COVER;
    line_dsc.radius = 1;
    line_dsc.border_width = 0;

    s_bright_draw_reenter = true;
    for(int i = 0; i < 3; i++) {
        lv_area_t line_area;
        line_area.x1 = cx - line_w / 2;
        line_area.x2 = line_area.x1 + line_w - 1;
        line_area.y1 = y0 + i * (line_h + line_gap);
        line_area.y2 = line_area.y1 + line_h - 1;
        lv_draw_rect(base->layer, &line_dsc, &line_area);
    }
    s_bright_draw_reenter = false;
}

/* 亮度滑条：扩大扩展绘制区，保证两端旋钮半圆完整刷新 */
static void cb_admin_brightness_slider_ext_draw_size(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_REFR_EXT_DRAW_SIZE) return;
    int32_t * s = (int32_t *)lv_event_get_param(e);
    if(s == NULL) return;
    *s = LV_MAX(*s, BRIGHT_PAGE_SIDE_PAD + 2);
}

static void cb_admin_brightness_slider_size_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_SIZE_CHANGED) return;
    admin_brightness_fill_sync();
}

/* 支付设置：首页 / 支付方式 / 超时选择 / 订单列表 / 摘要 / 详情 */
static void admin_payment_set_page(admin_payment_page_t page)
{
    g_admin_payment_page = page;
    bool list = (page == ADMIN_PAYMENT_PAGE_LIST);
    bool method = (page == ADMIN_PAYMENT_PAGE_METHOD);
    bool timeout = (page == ADMIN_PAYMENT_PAGE_TIMEOUT);
    bool orders = (page == ADMIN_PAYMENT_PAGE_ORDERS);
    bool summary = (page == ADMIN_PAYMENT_PAGE_ORDER_SUMMARY);
    bool detail = (page == ADMIN_PAYMENT_PAGE_ORDER_DETAIL);
    bool show_shell = (list || method || orders || summary || detail);

    if(g_admin_img_payment_title_box != NULL) {
        if(show_shell) lv_obj_remove_flag(g_admin_img_payment_title_box, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_img_payment_title_box, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_lbl_payment_title != NULL) {
        if(show_shell) lv_obj_remove_flag(g_admin_lbl_payment_title, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_lbl_payment_title, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_payment_set_box_wrap != NULL) {
        if(show_shell) lv_obj_remove_flag(g_admin_payment_set_box_wrap, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_payment_set_box_wrap, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_payment_list_view != NULL) {
        if(list) lv_obj_remove_flag(g_admin_payment_list_view, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_payment_list_view, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_payment_method_view != NULL) {
        if(method) lv_obj_remove_flag(g_admin_payment_method_view, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_payment_method_view, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_payment_orders_view != NULL) {
        if(orders) lv_obj_remove_flag(g_admin_payment_orders_view, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_payment_orders_view, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_payment_order_summary_view != NULL) {
        if(summary) lv_obj_remove_flag(g_admin_payment_order_summary_view, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_payment_order_summary_view, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_payment_order_detail_view != NULL) {
        if(detail) lv_obj_remove_flag(g_admin_payment_order_detail_view, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_payment_order_detail_view, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_payment_tp_wrap != NULL) {
        if(timeout) lv_obj_remove_flag(g_admin_payment_tp_wrap, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_payment_tp_wrap, LV_OBJ_FLAG_HIDDEN);
    }
    if(!timeout) g_admin_payment_tp_active = false;
}

static void admin_payment_sync_list_ui(void)
{
    g_admin_payment_ui_loading = true;
    if(g_admin_lbl_payment_method != NULL)
        lv_label_set_text(g_admin_lbl_payment_method, ui_translation(STR_PAYMENT_METHOD));
    if(g_admin_lbl_payment_timeout != NULL)
        lv_label_set_text(g_admin_lbl_payment_timeout, ui_translation(STR_PAYMENT_TIMEOUT));
    if(g_admin_lbl_payment_order != NULL)
        lv_label_set_text(g_admin_lbl_payment_order, ui_translation(STR_PAYMENT_ORDER));

    /* 支付方式/超时/订单查询为入口开关：保持用户当前态 */
    g_admin_payment_ui_loading = false;
}

static void admin_payment_tp_sync_digits(void)
{
    uint32_t m = g_admin_payment_tp_minutes;
    uint32_t s = g_admin_payment_tp_seconds;
    if(m > TP_MMSS_MAX_MIN) m = TP_MMSS_MAX_MIN;
    if(s > TP_MMSS_MAX_MIN) s = TP_MMSS_MAX_MIN;
    if(g_admin_payment_tp_digit_rollers[0] != NULL)
        lv_roller_set_selected(g_admin_payment_tp_digit_rollers[0], m / 10, LV_ANIM_OFF);
    if(g_admin_payment_tp_digit_rollers[1] != NULL)
        lv_roller_set_selected(g_admin_payment_tp_digit_rollers[1], m % 10, LV_ANIM_OFF);
    if(g_admin_payment_tp_digit_rollers[2] != NULL)
        lv_roller_set_selected(g_admin_payment_tp_digit_rollers[2], s / 10, LV_ANIM_OFF);
    if(g_admin_payment_tp_digit_rollers[3] != NULL)
        lv_roller_set_selected(g_admin_payment_tp_digit_rollers[3], s % 10, LV_ANIM_OFF);
}

static void admin_payment_show_time_picker(void)
{
    if(g_admin_payment_tp_wrap == NULL) return;

    uint32_t sec = g_ui_payment_timeout_sec;
    if(sec < PAYMENT_TIMEOUT_SEC_MIN) sec = PAYMENT_TIMEOUT_SEC_DEFAULT;
    if(sec > PAYMENT_TIMEOUT_SEC_MAX) sec = PAYMENT_TIMEOUT_SEC_MAX;
    g_admin_payment_tp_minutes = sec / 60u;
    g_admin_payment_tp_seconds = sec % 60u;

    admin_payment_tp_sync_digits();
    admin_payment_set_page(ADMIN_PAYMENT_PAGE_TIMEOUT);
    g_admin_payment_tp_active = true;

}

static void admin_payment_hide_time_picker(void)
{
    if(g_admin_payment_tp_wrap == NULL) return;
    admin_payment_set_page(ADMIN_PAYMENT_PAGE_LIST);
    admin_payment_sync_list_ui();
}

/* 摘要页「开发票 / 订单详情」宽度随文字变化，故每次刷新后重新右对齐排布 */
static void admin_payment_order_summary_btns_layout(void)
{
    if(g_admin_btn_order_invoice == NULL || g_admin_btn_order_detail == NULL) return;

    const lv_coord_t sum_w = 960;
    const lv_coord_t sum_x = (1117 - sum_w) / 2;
    const lv_coord_t y = 325 + 20;
    const lv_coord_t gap = 16;

    lv_obj_update_layout(g_admin_btn_order_invoice);
    lv_obj_update_layout(g_admin_btn_order_detail);

    lv_coord_t dw = lv_obj_get_width(g_admin_btn_order_detail);
    lv_coord_t iw = lv_obj_get_width(g_admin_btn_order_invoice);
    lv_obj_set_pos(g_admin_btn_order_detail, sum_x + sum_w - dw, y);
    lv_obj_set_pos(g_admin_btn_order_invoice, sum_x + sum_w - dw - gap - iw, y);
}

static void admin_payment_sync_order_summary_ui(int idx)
{
    char buf[64];
    if(idx < 0 || idx >= ADMIN_PAYMENT_ORDER_CNT) idx = 0;
    const admin_payment_order_demo_t * o = &g_admin_payment_order_demo[idx];

    if(g_admin_lbl_order_sum_title != NULL)
        lv_label_set_text(g_admin_lbl_order_sum_title, o->title);
    if(g_admin_lbl_order_sum_status != NULL) {
        lv_label_set_text(g_admin_lbl_order_sum_status,
            ui_translation(o->running ? STR_ORDER_STATUS_RUNNING : STR_ORDER_STATUS_DONE));
    }
    if(g_admin_order_sum_status_badge != NULL) {
        if(o->running) {
            lv_obj_set_style_bg_opa(g_admin_order_sum_status_badge, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_bg_color(g_admin_order_sum_status_badge,
                lv_color_hex(COL_ORANGE), LV_PART_MAIN);
            if(g_admin_lbl_order_sum_status != NULL)
                lv_obj_set_style_text_color(g_admin_lbl_order_sum_status,
                    lv_color_hex(COL_TEXT), LV_PART_MAIN);
        }
        else {
            lv_obj_set_style_bg_opa(g_admin_order_sum_status_badge, LV_OPA_TRANSP, LV_PART_MAIN);
            if(g_admin_lbl_order_sum_status != NULL)
                lv_obj_set_style_text_color(g_admin_lbl_order_sum_status,
                    lv_color_hex(COL_TEXT), LV_PART_MAIN);
        }
    }
    if(g_admin_lbl_order_sum_prog != NULL)
        lv_label_set_text(g_admin_lbl_order_sum_prog, o->prog_name);
    if(g_admin_lbl_order_sum_sub != NULL)
        lv_label_set_text(g_admin_lbl_order_sum_sub, o->summary_sub);
    if(g_admin_lbl_order_sum_time != NULL)
        lv_label_set_text(g_admin_lbl_order_sum_time, o->time_start);
    if(g_admin_lbl_order_sum_total != NULL) {
        lv_snprintf(buf, sizeof(buf), ui_translation(STR_ORDER_TOTAL_FMT), o->price);
        lv_label_set_text(g_admin_lbl_order_sum_total, buf);
    }
    admin_payment_order_summary_btns_layout();
}

static void admin_payment_sync_order_detail_ui(int idx)
{
    char buf[96];
    if(idx < 0 || idx >= ADMIN_PAYMENT_ORDER_CNT) idx = 0;
    const admin_payment_order_demo_t * o = &g_admin_payment_order_demo[idx];

    if(g_admin_lbl_order_detail_id != NULL) {
        lv_snprintf(buf, sizeof(buf), "%s %s",
            ui_translation(STR_ORDER_DETAIL_HDR), o->id_tail);
        lv_label_set_text(g_admin_lbl_order_detail_id, buf);
    }
    if(g_admin_lbl_order_detail_status != NULL) {
        lv_label_set_text(g_admin_lbl_order_detail_status,
            ui_translation(o->running ? STR_ORDER_STATUS_RUNNING : STR_ORDER_STATUS_DONE));
    }
    if(g_admin_order_detail_status_badge != NULL) {
        if(o->running) {
            lv_obj_set_style_bg_opa(g_admin_order_detail_status_badge, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_bg_color(g_admin_order_detail_status_badge,
                lv_color_hex(COL_ORANGE), LV_PART_MAIN);
            if(g_admin_lbl_order_detail_status != NULL)
                lv_obj_set_style_text_color(g_admin_lbl_order_detail_status,
                    lv_color_hex(COL_TEXT), LV_PART_MAIN);
        }
        else {
            lv_obj_set_style_bg_opa(g_admin_order_detail_status_badge, LV_OPA_TRANSP, LV_PART_MAIN);
            if(g_admin_lbl_order_detail_status != NULL)
                lv_obj_set_style_text_color(g_admin_lbl_order_detail_status,
                    lv_color_hex(COL_TEXT), LV_PART_MAIN);
        }
    }
    if(g_admin_lbl_order_detail_prog != NULL)
        lv_label_set_text(g_admin_lbl_order_detail_prog, o->prog_name);
    if(g_admin_lbl_order_detail_params != NULL)
        lv_label_set_text(g_admin_lbl_order_detail_params, o->prog_params);
    if(g_admin_lbl_order_detail_items != NULL) {
        lv_snprintf(buf, sizeof(buf), ui_translation(STR_ORDER_ITEMS_FMT), o->items);
        lv_label_set_text(g_admin_lbl_order_detail_items, buf);
    }
    if(g_admin_lbl_order_detail_paid != NULL) {
        lv_snprintf(buf, sizeof(buf), ui_translation(STR_ORDER_PAID_FMT), o->price);
        lv_label_set_text(g_admin_lbl_order_detail_paid, buf);
    }
    if(g_admin_lbl_order_detail_t_start != NULL) {
        lv_snprintf(buf, sizeof(buf), "%s%s",
            ui_translation(STR_ORDER_TIME_START), o->time_start);
        lv_label_set_text(g_admin_lbl_order_detail_t_start, buf);
    }
    if(g_admin_lbl_order_detail_t_end != NULL) {
        lv_snprintf(buf, sizeof(buf), "%s%s",
            ui_translation(STR_ORDER_TIME_END), o->time_end);
        lv_label_set_text(g_admin_lbl_order_detail_t_end, buf);
    }
}

static void admin_payment_show_order_summary(int idx)
{
    if(idx < 0 || idx >= ADMIN_PAYMENT_ORDER_CNT) idx = 0;
    g_admin_payment_order_sel = idx;
    admin_payment_sync_order_summary_ui(idx);
    admin_payment_set_page(ADMIN_PAYMENT_PAGE_ORDER_SUMMARY);
}

static void admin_payment_show_order_detail(int idx)
{
    if(idx < 0 || idx >= ADMIN_PAYMENT_ORDER_CNT) idx = 0;
    g_admin_payment_order_sel = idx;
    admin_payment_sync_order_detail_ui(idx);
    admin_payment_set_page(ADMIN_PAYMENT_PAGE_ORDER_DETAIL);
}

static void admin_data_style_switch(lv_obj_t * sw)
{
    if(sw == NULL) return;
    admin_panel_style_switch(sw);
    lv_obj_set_size(sw, ADMIN_SW_SIZE_W, ADMIN_SW_SIZE_H);
    lv_obj_refr_size(sw);
    lv_obj_set_style_radius(sw, lv_obj_get_height(sw) / 2, LV_PART_MAIN);
    lv_obj_update_layout(sw);
    lv_obj_set_style_radius(sw, lv_obj_get_content_height(sw) / 2, LV_PART_INDICATOR);
    admin_sw_apply_knob_pad(sw, ADMIN_SW_KNOB_SIZE, LV_PART_KNOB);
}

static void admin_data_set_page(admin_data_page_t page)
{
    g_admin_data_page = page;
    if(g_admin_panel_data == NULL) return;

    if(g_admin_data_list_view != NULL) {
        if(page == ADMIN_DATA_PAGE_LIST) lv_obj_remove_flag(g_admin_data_list_view, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_data_list_view, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_data_strategy_view != NULL) {
        if(page == ADMIN_DATA_PAGE_STRATEGY) lv_obj_remove_flag(g_admin_data_strategy_view, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_data_strategy_view, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_lbl_data_title != NULL) {
        if(page == ADMIN_DATA_PAGE_LIST) lv_obj_remove_flag(g_admin_lbl_data_title, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_lbl_data_title, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_img_data_title_box != NULL) {
        if(page == ADMIN_DATA_PAGE_LIST) lv_obj_remove_flag(g_admin_img_data_title_box, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_img_data_title_box, LV_OBJ_FLAG_HIDDEN);
    }
}

/* 数据设置页：打开指定上传项的策略页，并恢复该单项已保存的开关状态 */
static void admin_data_open_strategy(int item_idx)
{
    if(item_idx < 0 || item_idx >= ADMIN_DATA_UPLOAD_COUNT) return;
    g_admin_data_strategy_item = item_idx;
    admin_data_set_page(ADMIN_DATA_PAGE_STRATEGY);
    admin_data_sync_strategy_ui();
}

static void admin_pwd_chg_hide_result(void)
{
    if(g_admin_pwd_chg_result != NULL) {
        lv_obj_add_flag(g_admin_pwd_chg_result, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_view == PASSWORD_CHANGE_OLD) {
        if(g_admin_img_pwd_chg_old_title_box != NULL)
            lv_obj_remove_flag(g_admin_img_pwd_chg_old_title_box, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_lbl_pwd_chg_old_title != NULL)
            lv_obj_remove_flag(g_admin_lbl_pwd_chg_old_title, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_pwd_chg_old_content != NULL)
            lv_obj_remove_flag(g_admin_pwd_chg_old_content, LV_OBJ_FLAG_HIDDEN);
    }
    else if(g_admin_view == PASSWORD_CHANGE_NEW) {
        if(g_admin_img_pwd_chg_new_title_box != NULL)
            lv_obj_remove_flag(g_admin_img_pwd_chg_new_title_box, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_lbl_pwd_chg_new_title != NULL)
            lv_obj_remove_flag(g_admin_lbl_pwd_chg_new_title, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_pwd_chg_new_content != NULL)
            lv_obj_remove_flag(g_admin_pwd_chg_new_content, LV_OBJ_FLAG_HIDDEN);
    }
}

static void admin_pwd_chg_show_result(bool ok, ui_str_id_t msg_id)
{
    g_admin_pwd_chg_result_ok = ok;
    g_admin_pwd_chg_new_msg_id = msg_id;

    if(g_admin_kb != NULL) {
        g_admin_kb_ta = NULL;
        lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_view == PASSWORD_CHANGE_OLD) {
        if(g_admin_img_pwd_chg_old_title_box != NULL)
            lv_obj_add_flag(g_admin_img_pwd_chg_old_title_box, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_lbl_pwd_chg_old_title != NULL)
            lv_obj_add_flag(g_admin_lbl_pwd_chg_old_title, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_pwd_chg_old_content != NULL)
            lv_obj_add_flag(g_admin_pwd_chg_old_content, LV_OBJ_FLAG_HIDDEN);
    }
    else if(g_admin_view == PASSWORD_CHANGE_NEW) {
        if(g_admin_img_pwd_chg_new_title_box != NULL)
            lv_obj_add_flag(g_admin_img_pwd_chg_new_title_box, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_lbl_pwd_chg_new_title != NULL)
            lv_obj_add_flag(g_admin_lbl_pwd_chg_new_title, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_pwd_chg_new_content != NULL)
            lv_obj_add_flag(g_admin_pwd_chg_new_content, LV_OBJ_FLAG_HIDDEN);
    }

    if(g_admin_pwd_chg_result == NULL) return;
    if(g_admin_img_pwd_chg_result != NULL) {
        lv_image_set_src(g_admin_img_pwd_chg_result, ok ? &success : &failure);
    }
    if(g_admin_lbl_pwd_chg_result != NULL) {
        lv_label_set_text(g_admin_lbl_pwd_chg_result, ui_translation(msg_id));
    }
    lv_obj_remove_flag(g_admin_pwd_chg_result, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(g_admin_pwd_chg_result);
}

static void admin_pwd_chg_page1_on_ready(void)
{
    if(g_admin_ta_pwd_chg_old == NULL || g_admin_ta_pwd_chg_new1 == NULL) return;

    lv_obj_t * ta = g_admin_kb_ta;
    if(ta != g_admin_ta_pwd_chg_old && ta != g_admin_ta_pwd_chg_new1) {
        ta = (g_admin_pwd_chg_page1_step == 1) ? g_admin_ta_pwd_chg_new1 : g_admin_ta_pwd_chg_old;
    }

    if(ta == g_admin_ta_pwd_chg_old) {
        const char * told = lv_textarea_get_text(g_admin_ta_pwd_chg_old);
        if(told == NULL || lv_strlen(told) != 6) return;
        g_admin_pwd_chg_page1_step = 1;
        if(g_admin_kb != NULL) g_admin_kb_ta = g_admin_ta_pwd_chg_new1;
        return;
    }

    const char * told = lv_textarea_get_text(g_admin_ta_pwd_chg_old);
    const char * tnew = lv_textarea_get_text(g_admin_ta_pwd_chg_new1);
    if(told == NULL || tnew == NULL || lv_strlen(told) != 6 || lv_strlen(tnew) != 6) return;

    if(lv_strcmp(told, g_admin_pwd) != 0) {
        g_admin_pwd_chg_old_err_id = STR_PWD_WRONG_REENTER;
        admin_pwd_chg_show_result(false, STR_PWD_WRONG_REENTER);
        return;
    }
    if(lv_strcmp(tnew, g_admin_pwd) == 0) {
        /* 新密码须与原密码不同 */
        return;
    }

    lv_strncpy(g_admin_pwd_chg_pending, tnew, ADMIN_PWD_LEN - 1);
    g_admin_pwd_chg_pending[ADMIN_PWD_LEN - 1] = '\0';
    admin_panel_show(PASSWORD_CHANGE_NEW);
}

static void admin_pwd_chg_page2_on_ready(void)
{
    if(g_admin_ta_pwd_chg_new2 == NULL) return;
    const char * t2 = lv_textarea_get_text(g_admin_ta_pwd_chg_new2);
    if(t2 == NULL || lv_strlen(t2) != 6) return;

    if(lv_strcmp(t2, g_admin_pwd_chg_pending) != 0) {
        g_admin_pwd_chg_new_msg_id = STR_PWD_MISMATCH;
        admin_pwd_chg_show_result(false, STR_PWD_MISMATCH);
        return;
    }

    lv_strcpy(g_admin_pwd, g_admin_pwd_chg_pending);
    g_admin_pwd_chg_new_msg_id = STR_PWD_CHANGE_OK;
    admin_pwd_chg_show_result(true, STR_PWD_CHANGE_OK);
}

static void cb_admin_machine_cancel(lv_event_t * e)
{
	(void)e;
	admin_machine_id_back_to_menu1();
}

static void cb_admin_mid_success_click(lv_event_t * e)
{
	if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
	if(g_admin_mid_success_overlay != NULL) {
		lv_obj_add_flag(g_admin_mid_success_overlay, LV_OBJ_FLAG_HIDDEN);
	}
	admin_machine_id_back_to_menu1();
}

static void cb_admin_pwd_chg_result_click(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_pwd_chg_result_ok) {
        admin_pwd_chg_back_to_menu2();
    }
    else {
        admin_pwd_chg_enter();
    }
}

/* 布局尺寸变化时重算声音滑条填充宽度 */
static void cb_admin_sound_slider_size_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_SIZE_CHANGED) return;
    lv_obj_t * slider = lv_event_get_target_obj(e);
    if(slider == g_admin_slider_sound_volume) {
        admin_sound_volume_slider_sync_bar(0);
    }
    else if(slider == g_admin_slider_touch_sound_volume) {
        admin_sound_touch_sound_volume_slider_sync_bar(0);
    }
}

static void admin_wifi_ui_enter(void)
{
    admin_wifi_timer_stop();
    admin_wifi_apply_switch_layout();
    admin_wifi_sync_switch_ui();
    admin_wifi_set_phase(ADMIN_WIFI_PHASE_PROMPT);
}

static void admin_wifi_set_phase(admin_wifi_phase_t phase)
{
    g_admin_wifi_phase = phase;
    if(g_admin_view != WIFI_SETTINGS) return;

    const bool prompt = (phase == ADMIN_WIFI_PHASE_PROMPT);
    const bool provisioning = (phase == ADMIN_WIFI_PHASE_PROVISIONING);
    const bool result_page = (phase == ADMIN_WIFI_PHASE_SUCCESS || phase == ADMIN_WIFI_PHASE_FAILURE);

    if(g_admin_lbl_wifi_title != NULL) {
        if(result_page) lv_obj_add_flag(g_admin_lbl_wifi_title, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_remove_flag(g_admin_lbl_wifi_title, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_img_wifi_title_box != NULL) {
        if(result_page) lv_obj_add_flag(g_admin_img_wifi_title_box, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_remove_flag(g_admin_img_wifi_title_box, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_sw_wifi != NULL) {
        if(result_page) lv_obj_add_flag(g_admin_sw_wifi, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_remove_flag(g_admin_sw_wifi, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_lbl_wifi_prompt != NULL) {
        if(prompt) lv_obj_remove_flag(g_admin_lbl_wifi_prompt, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_lbl_wifi_prompt, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_lbl_wifi_status != NULL) {
        if(provisioning) {
            lv_label_set_text(g_admin_lbl_wifi_status, ui_translation(STR_WIFI_PROVISIONING));
            lv_obj_remove_flag(g_admin_lbl_wifi_status, LV_OBJ_FLAG_HIDDEN);
        }
        else {
            lv_obj_add_flag(g_admin_lbl_wifi_status, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if(g_admin_wifi_done_center != NULL) {
        if(result_page) {
            lv_obj_remove_flag(g_admin_wifi_done_center, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(g_admin_wifi_done_center);
            if(g_admin_img_wifi_done != NULL) {
                lv_image_set_src(g_admin_img_wifi_done,
                    (phase == ADMIN_WIFI_PHASE_SUCCESS) ? &success : &failure);
            }
            if(g_admin_lbl_wifi_done != NULL) {
                lv_label_set_text(g_admin_lbl_wifi_done,
                    ui_translation((phase == ADMIN_WIFI_PHASE_SUCCESS) ? STR_WIFI_SUCCESS : STR_WIFI_FAIL));
            }
        } else {
            lv_obj_add_flag(g_admin_wifi_done_center, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if(g_admin_wifi_set_box_wrap != NULL) {
        if(result_page) lv_obj_add_flag(g_admin_wifi_set_box_wrap, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_remove_flag(g_admin_wifi_set_box_wrap, LV_OBJ_FLAG_HIDDEN);
    }

}

/* 离开 4G 设置页：回网络设置 */
static void admin_4g_back_to_network(void)
{
    admin_4g_timer_stop();
    admin_panel_show(NETWORK_SETTINGS);
}

static void admin_wifi_sync_switch_ui(void)
{
    if(g_admin_sw_wifi == NULL) return;
    g_admin_wifi_ui_loading = true;
    if(ui_wifi_get()) {
        lv_obj_add_state(g_admin_sw_wifi, LV_STATE_CHECKED);
    }
    else {
        lv_obj_remove_state(g_admin_sw_wifi, LV_STATE_CHECKED);
    }
    g_admin_wifi_ui_loading = false;
}

/* 4G 结果页点击：成功回网络设置；失败回说明页并将开关关闭 */
static void cb_admin_4g_result_click(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != SETTINGS_4G) return;

    if(g_admin_4g_phase == ADMIN_4G_PHASE_SUCCESS) {
        admin_4g_back_to_network();
        return;
    }
    if(g_admin_4g_phase == ADMIN_4G_PHASE_FAILURE) {
        ui_4g_set(false);
        admin_4g_ui_enter();
    }
}

static void admin_wifi_timer_stop(void)
{
    if(g_admin_wifi_timer != NULL) {
        lv_timer_delete(g_admin_wifi_timer);
        g_admin_wifi_timer = NULL;
    }
}

static void admin_wifi_start_provisioning(void)
{
    admin_wifi_timer_stop();
    g_ui_wifi_connect_result = 1;//PC默认值，2秒后读取硬件反馈的值
    admin_wifi_set_phase(ADMIN_WIFI_PHASE_PROVISIONING);
    g_admin_wifi_timer = lv_timer_create(cb_admin_wifi_timer, 2000, NULL);
    lv_timer_set_repeat_count(g_admin_wifi_timer, 1);
}

static void cb_admin_wifi_switch_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(g_admin_view != WIFI_SETTINGS) return;
    if(g_admin_wifi_ui_loading) return;

    lv_obj_t * sw = lv_event_get_target_obj(e);
    bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);

    if(on) {
        if(g_admin_wifi_phase == ADMIN_WIFI_PHASE_PROMPT) {
            ui_wifi_set(true);
            admin_wifi_start_provisioning();
        }
    }
    else {
        ui_wifi_set(false);
        if(g_admin_wifi_phase != ADMIN_WIFI_PHASE_PROMPT) {
            admin_wifi_timer_stop();
            admin_wifi_set_phase(ADMIN_WIFI_PHASE_PROMPT);
        }
    }
}

static void cb_admin_wifi_timer(lv_timer_t * t)
{
    (void)t;
    g_admin_wifi_timer = NULL;
    if(g_admin_view == WIFI_SETTINGS) {
        if(g_ui_wifi_connect_result == 1) {
            admin_wifi_set_phase(ADMIN_WIFI_PHASE_SUCCESS);
        } else {
            admin_wifi_set_phase(ADMIN_WIFI_PHASE_FAILURE);
        }
    }
}

static void cb_admin_wifi_result_click(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != WIFI_SETTINGS) return;

    if(g_admin_wifi_phase == ADMIN_WIFI_PHASE_SUCCESS) {
        admin_wifi_back_to_network();
        return;
    }
    if(g_admin_wifi_phase == ADMIN_WIFI_PHASE_FAILURE) {
        ui_wifi_set(false);
        admin_wifi_ui_enter();
    }
}

static void admin_wifi_apply_switch_layout(void)
{
    if(g_admin_sw_wifi == NULL) return;

    admin_panel_style_switch(g_admin_sw_wifi);
    lv_obj_set_size(g_admin_sw_wifi, ADMIN_SW_SIZE_W, ADMIN_SW_SIZE_H);
    lv_obj_refr_size(g_admin_sw_wifi);

    const lv_coord_t radius_cap = lv_obj_get_height(g_admin_sw_wifi) / 2;
    lv_obj_set_style_radius(g_admin_sw_wifi, radius_cap, LV_PART_MAIN);
    lv_obj_update_layout(g_admin_sw_wifi);
    lv_coord_t radius_ind = lv_obj_get_content_height(g_admin_sw_wifi) / 2;
    if(radius_ind < 0) {
        radius_ind = 0;
    }
    lv_obj_set_style_radius(g_admin_sw_wifi, radius_ind, LV_PART_INDICATOR);

    admin_sw_apply_knob_pad(g_admin_sw_wifi, ADMIN_SW_KNOB_SIZE, LV_PART_KNOB);

    lv_obj_set_pos(g_admin_sw_wifi, 1200, 150);
    lv_obj_invalidate(g_admin_sw_wifi);
}

static void cb_admin_payment_list_sw_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(g_admin_view != PAYMENT_SETTINGS) return;
    if(g_admin_payment_ui_loading) return;
    if(g_admin_payment_page != ADMIN_PAYMENT_PAGE_LIST) return;

    lv_obj_t * sw = lv_event_get_target_obj(e);
    if(sw == NULL) return;
    bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);

    if(sw == g_admin_sw_payment_method) {
        if(on) {
            admin_payment_set_page(ADMIN_PAYMENT_PAGE_METHOD);
            admin_payment_sync_list_ui();
    admin_payment_sync_method_ui();
        }
    }
    else if(sw == g_admin_sw_payment_timeout) {
        if(on) {
            admin_payment_show_time_picker();
        }
    }
    else if(sw == g_admin_sw_payment_order) {
        g_ui_payment_order_enabled = on;
        if(on) {
            admin_payment_set_page(ADMIN_PAYMENT_PAGE_ORDERS);
        }
    }
}

static void cb_admin_payment_order_row_clicked(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != PAYMENT_SETTINGS) return;
    if(g_admin_payment_page != ADMIN_PAYMENT_PAGE_ORDERS) return;
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    admin_payment_show_order_summary(idx);
}

static void cb_admin_payment_order_invoice(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_payment_page != ADMIN_PAYMENT_PAGE_ORDER_SUMMARY) return;
    /* 开发票：本期仅占位，无跳转 */
    (void)e;
}

static void cb_admin_payment_order_detail_btn(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_payment_page != ADMIN_PAYMENT_PAGE_ORDER_SUMMARY) return;
    admin_payment_show_order_detail(g_admin_payment_order_sel);
}

static void cb_admin_payment_tp_confirm(lv_event_t * e)
{
    (void)e;
    if(!g_admin_payment_tp_active) return;
    uint32_t total_sec = g_admin_payment_tp_minutes * 60u + g_admin_payment_tp_seconds;
    ui_payment_timeout_sec_set((uint16_t)total_sec);
    admin_payment_hide_time_picker();
}

static void cb_admin_payment_tp_cancel(lv_event_t * e)
{
    (void)e;
    if(!g_admin_payment_tp_active) return;
    admin_payment_hide_time_picker();
}

static void cb_admin_payment_tp_digit_roller_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(!g_admin_payment_tp_active) return;
    uint32_t m = 0;
    uint32_t s = 0;
    if(g_admin_payment_tp_digit_rollers[0] != NULL)
        m += lv_roller_get_selected(g_admin_payment_tp_digit_rollers[0]) * 10;
    if(g_admin_payment_tp_digit_rollers[1] != NULL)
        m += lv_roller_get_selected(g_admin_payment_tp_digit_rollers[1]);
    if(g_admin_payment_tp_digit_rollers[2] != NULL)
        s += lv_roller_get_selected(g_admin_payment_tp_digit_rollers[2]) * 10;
    if(g_admin_payment_tp_digit_rollers[3] != NULL)
        s += lv_roller_get_selected(g_admin_payment_tp_digit_rollers[3]);
    if(m > TP_MMSS_MAX_MIN) m = TP_MMSS_MAX_MIN;
    if(s > TP_MMSS_MAX_MIN) s = TP_MMSS_MAX_MIN;
    g_admin_payment_tp_minutes = m;
    g_admin_payment_tp_seconds = s;
}

static void cb_admin_data_upload_switch_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(g_admin_view != DATA_SETTINGS) return;
    if(g_admin_data_page != ADMIN_DATA_PAGE_LIST) return;
    if(g_admin_data_upload_ui_loading) return;
    lv_obj_t * sw = lv_event_get_target_obj(e);
    if(sw == NULL) return;
    int idx = admin_data_upload_index_from_cb(sw);
    if(idx < 0) return;
    bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);
    switch(idx) {
    case 0: ui_data_upload_basic_set(on); break;
    case 1: ui_data_upload_sensor_set(on); break;
    case 2: ui_data_upload_fault_set(on); break;
    case 3: ui_data_upload_auto_dispense_set(on); break;
    case 4: ui_data_upload_payment_order_set(on); break;
    case 5: ui_data_upload_user_op_set(on); break;
    case 6: ui_data_upload_device_set(on); break;
    default: break;
    }
    /* 开关打开后跳转到该单项的策略页，并恢复该项已保存策略 */
    if(on) {
        admin_data_open_strategy(idx);
    }
}

static void build_admin(void);  //构建管理员页：密码/设置网格/机器ID子面板
static void build_selfcheck(void);  //构建有水自检页
static void build_cycle(void);  //构建循环程序（寿命试验）页
static void cycle_ui_reset(void);
static void cycle_abort_run(void);
static void cycle_finish_current_run(void);
static void cycle_run_count_label_sync(void);
static void cycle_fault_overlay_create(lv_obj_t * parent, lv_obj_t ** panel_out, lv_obj_t ** lbl_out);
static void cycle_fault_blink_stop(void);
static void cycle_fault_blink_start(void);
static void cycle_encoder_group_build(void);
static void cycle_kb_close(void);
static void cycle_kb_encoder_style_init(void);
static void cb_cycle_kb_cancel(lv_event_t * e);
static void cb_cycle_kb_encoder(lv_event_t * e);
static void cb_cycle_prog_ta_focus(lv_event_t * e);
static void cb_cycle_prog_roller_encoder(lv_event_t * e);
void ui_cycle_mode_enter(void);
void ui_cycle_mode_exit(void);
static void cb_admin_open_cycle(lv_event_t * e);
static void selfcheck_timer_stop_all(void);
static void selfcheck_indicator_blink_stop(void);
static void selfcheck_apply_step_indicators(selfcheck_ui_state_t step);
static void selfcheck_temp_label_sync(void);
static void selfcheck_ui_reset(void);
static void selfcheck_exit_to_vendor_menu(void);
static void ui_admin_resume_unlocked(void);
static void cb_admin_open_vendor_maint(lv_event_t * e);
static void cb_admin_open_selfcheck(lv_event_t * e);
static void admin_vendor_serial_back_to_menu1(void);
static void admin_vendor_menu_back_to_menu1(void);
static void admin_vendor_serial_try(void);
static void cb_selfcheck_back(lv_event_t * e);
static void cb_selfcheck_runpause(lv_event_t * e);
static void build_alarm_overlay(void);  //构建 lv_layer_top 报警弹层：E1–E14 + 缺液
static void alarm_content_update(void);  //刷新轮播当前子页内容（缺液图/文案）
static void alarm_fault_panel_relayout_content(uint8_t fault_idx);  //说明2 显隐 + 页脚电话/售后重排
static void alarm_fault_panels_relayout_content(void);  //全部故障子页重排内容
static void ui_alarm_poll(void);  //每帧检测报警并控制弹层显隐与轮播
static void ui_encoder_group_restore_for_active_screen(void);  //按当前 screen 恢复编码器组
static void build_pay(void);  //构建支付页：二维码、金额与支付提示
static void build_pay_done(void);  //构建支付完成页：图标与 2 秒后自动进运行页
static void build_off(void);  //构建关机/待机页：仅顶部栏与启停/电源

// 可移植字体绑定（通过字体适配层统一选择自定义/回退字体）
static const lv_font_t * s_font_sc_20;
static const lv_font_t * s_font_sc_27;
static const lv_font_t * s_font_sc_30;
static const lv_font_t * s_font_sc_35;
static const lv_font_t * s_font_sc_40;
static const lv_font_t * s_font_sc_50;
static const lv_font_t * s_font_sc_70;
static const lv_font_t * s_font_sc_125;

static void ui_fsm_poll_running_pause_sync(void);
static void ui_fsm_runpause_apply(bool long_press);
static void ui_fsm_runpause_apply_running_page(void);
static void cb_power_long(lv_event_t * event);
static void cb_runpause(lv_event_t * event);
static void cb_runpause_long(lv_event_t * event);
#if UI_DEBUG_PAY_SKIP_TO_DONE
static void cb_pay_debug_skip_to_done(lv_event_t * e);
#endif

static void cb_power_long(lv_event_t* event)
{
	(void)event;

	if(fsm.state != FSM_OFF) {
		if(g_cycle_active) {
			cycle_ui_reset();
			ui_cycle_mode_exit();
		}
		fsm_state_change(FSM_OFF);
	} else {
		fsm_state_change(FSM_STANDBY);
	}

	return;
}

/*
 * 外部暂停/继续（方案 B）：在 LVGL 任务外可写 get_fsm_state()；
 * task_lvgl 每圈调用 ui_fsm_poll_running_pause_sync()，在运行页同步倒计时。
 */
static void cb_runpause(lv_event_t* event)
{
	(void)event;
	/* 循环模式运行页启停：仅中断回设定页（cb_running_runpause 已处理，此处防双路径） */
	if(g_cycle_active && lv_scr_act() == g_scr_running) {
		return;
	}
	ui_fsm_runpause_apply(false);
}

static void cb_runpause_long(lv_event_t* event)
{
	(void)event;
	ui_fsm_runpause_apply(true);
}

#if UI_DEBUG_PAY_SKIP_TO_DONE
static void cb_pay_debug_skip_to_done(lv_event_t * e)
{
	(void)e;
	SETFLAG(FSM_FLAG_NEED_PAYMENT);
	SETFLAG(FSM_FLAG_PAYMENT_SUCCESS);
	ui_scr_load_async();
}
#endif

/* 写 FSM 状态；运行页 UI 由 ui_fsm_poll_running_pause_sync 同步 */
static void ui_fsm_runpause_apply(bool long_press)
{
	(void)long_press;
	if(g_cycle_active) return; // 5.2.1：循环模式不走主页支付/追加路径
	if(fsm.state == FSM_STANDBY && lv_screen_active() == g_scr_home) {
		// SETFLAG(FSM_FLAG_NEED_PAYMENT);
		// ui_scr_load_async();
		/*
		 * 有追加能力：同步进入追加时间页（FSM 保持 STANDBY，不置 NEED_PAYMENT）。
		 * 无追加能力（如风自洁）：commit(0) 后走原 FSM 异步支付路径，与改前直达支付一致。
		 */
		if(add_time_max_count(g_wheel_sel) == 0u) {
			g_add_time_sel = 0;
			add_time_session_commit(0);
			SETFLAG(FSM_FLAG_NEED_PAYMENT);
			ui_scr_load_async();
		} else {
			g_add_time_sel = 0;
			add_time_page_sync_labels();
			ui_screen_load(g_scr_add_time);
		}
	} else if(fsm.state == FSM_RUNNING) {
		fsm_state_change(FSM_PAUSED);
	} else if(fsm.state == FSM_PAUSED) {
		fsm_state_change(FSM_RUNNING);
	} else if(fsm.state == FSM_OFF) {
		fsm_state_change(FSM_STANDBY);
	}

	ui_fsm_poll_running_pause_sync();//运行页同步倒计时
}

/* 为单个控件设置正文字体（label / button 等） */
static void ui_set_obj_font(lv_obj_t * obj, const lv_font_t * font)
{
	if(obj != NULL && font != NULL) {                  /* 空指针保护 */
		lv_obj_set_style_text_font(obj, font, LV_PART_MAIN); /* 应用到 MAIN 部件 */
	}
}

//绑定中文字体并应用到 LVGL 默认主题
static void ui_apply_chinese_font(void)  //绑定中文字体并应用到 LVGL 默认主题
{
	s_font_sc_20 = ui_font_get_sc_20();
	s_font_sc_27 = ui_font_get_sc_27();
	s_font_sc_30 = ui_font_get_sc_30();
	s_font_sc_35 = ui_font_get_sc_35();
	s_font_sc_40 = ui_font_get_sc_40();
	s_font_sc_50 = ui_font_get_sc_50();
	s_font_sc_70 = ui_font_get_sc_70();
	s_font_sc_125 = ui_font_get_sc_125();

	lv_display_t * d = lv_display_get_default();
	if(d != NULL && s_font_sc_20 != NULL) {
		lv_theme_t * th = lv_theme_default_init(d, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
																						LV_THEME_DEFAULT_DARK, s_font_sc_20);
		lv_display_set_theme(d, th);
	}
	admin_kb_font_init();
}

/* 管理员数字键盘：数字键 ui_font_SC_30，缺字（图标等）回退 lv_font_montserrat_30 */
static void admin_kb_font_init(void)
{
	if(s_font_admin_kb_ptr != NULL) return;
	if(s_font_sc_30 == NULL) return;
	s_font_admin_kb = *s_font_sc_30;
#if LV_FONT_MONTSERRAT_30
	s_font_admin_kb.fallback = &lv_font_montserrat_30;
#else
	s_font_admin_kb.fallback = NULL;
#endif
	s_font_admin_kb_ptr = &s_font_admin_kb;
}




//以下是按钮函数的定义
//创建文本按钮
static lv_obj_t * make_text_btn(lv_obj_t * parent, const char * txt, lv_coord_t w, lv_coord_t h)  //创建透明背景文本按钮
{
	lv_obj_t * b = lv_button_create(parent);
	lv_obj_set_size(b, w, h);
	lv_obj_set_style_pad_all(b, 0, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(b, LV_OPA_TRANSP, LV_PART_MAIN);//背景透明
	lv_obj_set_style_border_width(b, 0, LV_PART_MAIN);//边框宽度为 0
	lv_obj_set_style_shadow_width(b, 0, LV_PART_MAIN);//阴影宽度为 0
	lv_obj_t * l = lv_label_create(b);
	lv_label_set_text(l, txt);
	lv_obj_set_style_text_color(l, lv_color_hex(COL_TEXT), LV_PART_MAIN);//文本颜色 COL_TEXT
	ui_set_obj_font(l, s_font_sc_20);
	lv_obj_center(l);//文本居中
	return b;
}//创建文本按钮

//创建橙色填充按钮
static lv_obj_t * make_orange_fill_btn(lv_obj_t * parent, const char * txt, lv_coord_t w, lv_coord_t h)  //创建橙色实心圆角按钮
{
	lv_obj_t * b = lv_button_create(parent);
	lv_obj_set_size(b, w, h);
	lv_obj_set_style_radius(b, 25, LV_PART_MAIN);                              //圆角半径
	lv_obj_set_style_shadow_width(b, 0, LV_PART_MAIN);                        //取消阴影
	lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_PART_MAIN);                   //背景不透明
	lv_obj_set_style_bg_color(b, lv_color_hex(COL_ORANGE), LV_PART_MAIN);     //背景颜色 COL_ORANGE
	lv_obj_t * l = lv_label_create(b);
	lv_label_set_text(l, txt);
	lv_obj_set_style_text_color(l, lv_color_hex(COL_TEXT), LV_PART_MAIN);     //文本颜色 COL_TEXT
	lv_obj_center(l);
	return b;
}//创建橙色填充按钮

//创建橙色描边按钮
static lv_obj_t * make_orange_outline_btn(lv_obj_t * parent, const char * txt, lv_coord_t w, lv_coord_t h)  //创建橙色描边圆角按钮
{
	lv_obj_t * b = lv_button_create(parent);
	lv_obj_set_size(b, w, h);
	lv_obj_set_style_radius(b, 25, LV_PART_MAIN);                              //圆角半径
	lv_obj_set_style_shadow_width(b, 0, LV_PART_MAIN);                        //取消阴影
	lv_obj_set_style_bg_opa(b, LV_OPA_TRANSP, LV_PART_MAIN);                  //背景透明
	lv_obj_set_style_border_width(b, 3, LV_PART_MAIN);                        //边框宽度为 3
	lv_obj_set_style_border_color(b, lv_color_hex(COL_ORANGE), LV_PART_MAIN); //边框颜色 COL_ORANGE
	lv_obj_t * l = lv_label_create(b);
	lv_label_set_text(l, txt);
	lv_obj_set_style_text_color(l, lv_color_hex(COL_ORANGE), LV_PART_MAIN);   //文本颜色 COL_ORANGE
	lv_obj_center(l);
	return b;
}//创建橙色描边按钮

/* 状态栏 时间同步 */
/* 定时器回调：每秒更新各页顶部时钟 label（HH:MM，非 i18n） */
static void cb_clock(lv_timer_t * t)
{
	(void)t;                                             /* 未使用定时器指针 */
	static uint32_t s_clock_seconds = 10u * 3600u + 8u * 60u; /* 仿真时钟起点 16:30:00 */
	uint32_t hour = (s_clock_seconds / 3600u) % 24u;   /* 小时 0..23 */
	uint32_t minute = (s_clock_seconds / 60u) % 60u;   /* 分钟 0..59 */
	uint32_t second = s_clock_seconds % 60u;           /* 秒 0..59 */
	char buf[8];                                       /* 状态栏 HH:MM */
	char buf_sec[12];                                  /* 待机居中 HH:MM:SS */
	snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)hour, (unsigned)minute);
	snprintf(buf_sec, sizeof(buf_sec), "%02u:%02u:%02u",
		(unsigned)hour, (unsigned)minute, (unsigned)second);
	if(g_lbl_clock != NULL) {                            /* 主页顶部时钟 label */
		lv_label_set_text(g_lbl_clock, buf);
	}
	if(g_lbl_clock_running != NULL) {                    /* 运行页时钟 label */
		lv_label_set_text(g_lbl_clock_running, buf);
	}
	if(g_lbl_clock_end != NULL) {                        /* 结束页时钟 label */
		lv_label_set_text(g_lbl_clock_end, buf);
	}
	if(g_lbl_clock_pay != NULL) {                        /* 支付页时钟 label */
		lv_label_set_text(g_lbl_clock_pay, buf);
	}
	if(g_lbl_clock_pay_done != NULL) {                   /* 支付完成页时钟 label */
		lv_label_set_text(g_lbl_clock_pay_done, buf);
	}
	if(g_lbl_clock_alarm != NULL) {                      /* 报警页时钟 label */
		lv_label_set_text(g_lbl_clock_alarm, buf);
	}
	if(g_lbl_clock_off != NULL) {                        /* 待机页右上角时钟 label */
		lv_label_set_text(g_lbl_clock_off, buf);
	}
	if(g_lbl_clock_off_center != NULL) {                 /* 待机页居中大字时钟（含秒） */
		lv_label_set_text(g_lbl_clock_off_center, buf_sec);
	}
	if(g_lbl_clock_admin != NULL) {                    /* 管理员页状态栏时钟 */
		lv_label_set_text(g_lbl_clock_admin, buf);
	}
	if(g_lbl_clock_add_time != NULL) {                 /* 追加时间页状态栏时钟 */
		lv_label_set_text(g_lbl_clock_add_time, buf);
	}
	if(g_lbl_clock_selfcheck != NULL) {                /* 自检页状态栏时钟 */
		lv_label_set_text(g_lbl_clock_selfcheck, buf);
	}
	if(g_lbl_clock_cycle != NULL) {                    /* 循环程序页状态栏时钟 */
		lv_label_set_text(g_lbl_clock_cycle, buf);
	}
	ui_idle_poll_pointer();                              /* 鼠标移动则重置空闲计时 */
	s_clock_seconds = (s_clock_seconds + 1u) % (24u * 3600u); /* 秒计数 +1，24h 回绕 */
}

/* 顶部栏右侧：4G / WiFi / 时间（主页与运行页共用） */
static void create_top_status_bar(lv_obj_t * top, lv_obj_t ** clock_lbl_out)  //创建顶部右侧状态栏：4G / WiFi / 时间
{
	lv_obj_t * status = lv_obj_create(top);
	lv_obj_set_size(status, LV_PCT(14), LV_PCT(100));
	lv_obj_set_pos(status, LV_PCT(84), 0);
	lv_obj_set_style_bg_opa(status, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(status, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(status, 0, LV_PART_MAIN);
	lv_obj_set_style_radius(status, 0, LV_PART_MAIN);

	lv_obj_t * t4g = lv_label_create(status);
	lv_label_set_text(t4g, "4G");
	lv_obj_align(t4g, LV_ALIGN_BOTTOM_LEFT, LV_PCT(25), -3);
	lv_obj_set_style_text_color(t4g, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(t4g, s_font_sc_27);

	lv_obj_t * tw = lv_image_create(status);
	lv_image_set_src(tw, &wifi_logo);
	lv_obj_align(tw, LV_ALIGN_BOTTOM_LEFT, LV_PCT(47), -10);

	lv_obj_t * clock = lv_label_create(status);
	lv_label_set_text(clock, "10:08");
	lv_obj_align(clock, LV_ALIGN_BOTTOM_RIGHT, 0, -3);
	lv_obj_set_style_text_color(clock, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(clock, s_font_sc_27);
	if(clock_lbl_out != NULL) {
		*clock_lbl_out = clock;
	}

	static bool s_clock_timer_started = false;
	if(!s_clock_timer_started) {
		lv_timer_create(cb_clock, 1000, NULL);
		s_clock_timer_started = true;
	} else if(clock_lbl_out != NULL && *clock_lbl_out != NULL) {
		cb_clock(NULL);
	}
}

/* 顶部栏：可选返回 + 4G / WiFi / 时间（启停、电源由各界面单独添加） */
static lv_obj_t * create_top_bar(lv_obj_t * parent, lv_obj_t ** clock_lbl_out,
	lv_obj_t * back_target, lv_obj_t ** back_btn_out)
{
	lv_obj_t * top = lv_obj_create(parent);
	lv_obj_set_size(top, LV_PCT(100), LV_PCT(10));
	lv_obj_set_pos(top, 0, 0);
	lv_obj_set_style_bg_opa(top, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(top, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(top, 0, LV_PART_MAIN);
	lv_obj_set_style_radius(top, 0, LV_PART_MAIN);

	if(back_target != NULL) {
		LV_IMAGE_DECLARE(back);
		lv_obj_t * imgbtn_back = lv_button_create(top);
		lv_obj_set_size(imgbtn_back, 90, 50);
		lv_obj_set_style_bg_opa(imgbtn_back, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(imgbtn_back, 0, LV_PART_MAIN);
		lv_obj_set_style_shadow_width(imgbtn_back, 0, LV_PART_MAIN);
		lv_obj_align(imgbtn_back, LV_ALIGN_LEFT_MID, 0, 0);
		lv_obj_t * img = lv_image_create(imgbtn_back);
		lv_image_set_src(img, &back);
		lv_obj_center(img);
		lv_obj_add_event_cb(imgbtn_back, cb_load_screen, LV_EVENT_CLICKED, back_target);
		if(back_btn_out != NULL) {
			*back_btn_out = imgbtn_back;
		}
	}

	create_top_status_bar(top, clock_lbl_out);
	return top;
}

static lv_obj_t * make_top_back_btn(lv_obj_t * top, lv_event_cb_t cb)
{
	LV_IMAGE_DECLARE(back);
	lv_obj_t * imgbtn_back = lv_button_create(top);
	lv_obj_set_size(imgbtn_back, 90, 50);
	lv_obj_set_style_bg_opa(imgbtn_back, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(imgbtn_back, 0, LV_PART_MAIN);
	lv_obj_set_style_shadow_width(imgbtn_back, 0, LV_PART_MAIN);
	lv_obj_align(imgbtn_back, LV_ALIGN_LEFT_MID, 0, 0);
	lv_obj_t * img = lv_image_create(imgbtn_back);
	lv_image_set_src(img, &back);
	lv_obj_center(img);
	if(cb != NULL) {
		lv_obj_add_event_cb(imgbtn_back, cb, LV_EVENT_CLICKED, NULL);
	}
	return imgbtn_back;
}

static lv_obj_t * add_top_text_btn(lv_obj_t * top, const char * txt, lv_coord_t x)
{
	lv_obj_t * btn = make_text_btn(top, txt, 80, 32);
	lv_obj_align(btn, LV_ALIGN_LEFT_MID, x, 0);
	lv_obj_remove_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
	return btn;
}

//轮播区函数定义
//轮播区尺寸变化事件回调函数
static void carousel_size_cb(lv_event_t * e)  //轮播区尺寸变化时重新布局
{
	(void)e;
	carousel_wrap_relayout();  //重排轮播布局
}

/* 卡片固定绑定程序下标：slot i ↔ 程序 i（线性条带，两端不循环、不隐藏） */
static void carousel_update_card_images(void)
{
	for(int i = 0; i < CAROUSEL_VISIBLE_SLOTS; i++) {
		if(g_mode_cards[i] == NULL) continue;
		if(i >= TOTAL_PROGRAMS) {
			lv_obj_add_flag(g_mode_cards[i], LV_OBJ_FLAG_HIDDEN);
			continue;
		}
		lv_obj_remove_flag(g_mode_cards[i], LV_OBJ_FLAG_HIDDEN);
		if(g_mode_card_imgs[i] != NULL) {
			lv_image_set_src(g_mode_card_imgs[i], g_program_imgs[i]);
		}
		if(g_mode_card_labels[i] != NULL) {
			lv_label_set_text(g_mode_card_labels[i], ui_program_name_get(i));
		}
	}
}

/* 相对焦点的水平偏移（选中在 0；|rel| 越大越靠边）。两端选中时一侧可到 ±4 */
static lv_coord_t carousel_rel_offset_x(float rel)
{
	/* 与原 5 槽间距接近：|rel|<=1 用 inner，1~2 再加 outer 比例，更远按 step 延续 */
	float a = rel < 0.f ? -rel : rel;
	float sign = rel < 0.f ? -1.f : 1.f;
	float dist;
	if(a <= 1.f) {
		dist = a * (float)s_mode_gap_inner;
	}
	else if(a <= 2.f) {
		dist = (float)s_mode_gap_inner + (a - 1.f) * (float)s_mode_gap_outer;
	}
	else {
		dist = (float)(s_mode_gap_inner + s_mode_gap_outer) + (a - 2.f) * (float)s_mode_step_x;
	}
	return (lv_coord_t)(sign * dist);
}

/* 5 张卡片中心相对轮播区中心的 X 偏移（兼容旧槽位公式，供其它逻辑参考） */
static lv_coord_t carousel_slot_center_x(int slot)  //计算槽位相对轮播中心的水平偏移
{
	return carousel_rel_offset_x((float)(slot - CAROUSEL_CENTER_SLOT));
}

//轮播区布局：程序 i 固定在卡片 i；焦点 focus=sel-turn 居中，端点时一侧排出全部其余程序
static void carousel_wrap_relayout(void)  //重排轮播布局
{
	if(g_mode_carousel == NULL) return;
	lv_coord_t mw = lv_obj_get_width(g_mode_carousel);
	lv_coord_t mh = lv_obj_get_height(g_mode_carousel);
	if(mw < 16 || mh < 16) return;

	carousel_update_card_images();

	lv_coord_t cx = mw / 2;
	lv_coord_t cy = mh / 2;
	const lv_coord_t y = cy - s_mode_card_sz / 2 - s_mode_row_shift_up;

	/* 焦点：turn 负（左滑/下一程序）→ focus 增大 */
	float focus = (float)g_wheel_sel - g_wheel_turn;

	/* 端点一侧排多张时略收间距，尽量留在屏内 */
	float max_abs_rel = 0.f;
	for(int i = 0; i < TOTAL_PROGRAMS && i < CAROUSEL_VISIBLE_SLOTS; i++) {
		float ar = (float)i - focus;
		if(ar < 0.f) ar = -ar;
		if(ar > max_abs_rel) max_abs_rel = ar;
	}
	float space_scale = 1.f;
	if(max_abs_rel > 2.05f) {
		lv_coord_t budget = mw / 2 - s_mode_card_sz / 4;
		if(budget < 80) budget = 80;
		lv_coord_t need = carousel_rel_offset_x(max_abs_rel);
		if(need < 0) need = -need;
		if(need > budget && need > 0) {
			space_scale = (float)budget / (float)need;
			if(space_scale < 0.55f) space_scale = 0.55f;
		}
	}

	int fg_i = -1;
	float fg_af = 99.f;

	for(int i = 0; i < CAROUSEL_VISIBLE_SLOTS; i++) {
		if(g_mode_cards[i] == NULL) continue;
		if(i >= TOTAL_PROGRAMS) continue;
		if(lv_obj_has_flag(g_mode_cards[i], LV_OBJ_FLAG_HIDDEN)) continue;

		float rel = (float)i - focus;
		float af = rel < 0.f ? -rel : rel;
		lv_coord_t x_off = (lv_coord_t)((float)carousel_rel_offset_x(rel) * space_scale);
		lv_coord_t x = cx + x_off - s_mode_card_sz / 2;

		lv_obj_set_pos(g_mode_cards[i], x, y);

		float af_vis = af;
		if(af_vis > 2.5f) af_vis = 2.5f;
		float t = 1.0f - ((af_vis > 2.0f) ? 2.0f : af_vis) / 2.0f;
		int32_t scale = MODE_SCALE_MIN + (int32_t)((float)(MODE_SCALE_MAX - MODE_SCALE_MIN) * t);
		lv_obj_set_style_transform_pivot_x(g_mode_cards[i], s_mode_card_sz / 2, LV_PART_MAIN);
		lv_obj_set_style_transform_pivot_y(g_mode_cards[i], s_mode_card_sz / 2, LV_PART_MAIN);
		lv_obj_set_style_transform_scale_x(g_mode_cards[i], scale, LV_PART_MAIN);
		lv_obj_set_style_transform_scale_y(g_mode_cards[i], scale, LV_PART_MAIN);

		lv_opa_t opa;
		if(af <= 0.1f) opa = LV_OPA_COVER;
		else if(af >= 2.5f) opa = (lv_opa_t)(LV_OPA_30);
		else if(af >= 2.0f) opa = (lv_opa_t)(LV_OPA_30);
		else {
			opa = (lv_opa_t)(255 - (int)(af * (255 - 76) / 2.0f));
		}
		lv_obj_set_style_opa(g_mode_cards[i], opa, LV_PART_MAIN);

		if(af < fg_af) {
			fg_af = af;
			fg_i = i;
		}
	}

	if(fg_i >= 0 && g_mode_cards[fg_i] != NULL) {
		lv_obj_move_foreground(g_mode_cards[fg_i]);
	}

	/* 指示点随拖动预览高亮；正式选中仍是 g_wheel_sel（松手/编码器才改） */
	const int32_t dot_sel = carousel_dot_preview_sel();
	lv_coord_t dot_w[TOTAL_PROGRAMS];
	lv_coord_t total_w = 0;
	const lv_coord_t dot_gap = 8;
	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		dot_w[i] = (i == dot_sel) ? 13 : 10;
		total_w += dot_w[i];
		if(i < TOTAL_PROGRAMS - 1) total_w += dot_gap;
	}

	lv_coord_t x_cursor = -total_w / 2;
	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		if(g_mode_dots[i] == NULL) {
			x_cursor += dot_w[i] + dot_gap;
			continue;
		}

		lv_obj_set_size(g_mode_dots[i], dot_w[i], dot_w[i]);
		lv_obj_align(g_mode_dots[i], LV_ALIGN_CENTER, x_cursor + dot_w[i] / 2, 0);
		lv_obj_set_style_bg_color(g_mode_dots[i], lv_color_hex((i == dot_sel) ? COL_TEXT : COL_DIM), LV_PART_MAIN);
		x_cursor += dot_w[i] + dot_gap;
	}
}

/* 指示点预览选中：线性夹紧，两端不循环 */
static int32_t carousel_dot_preview_sel(void)
{
	int step = (int)(g_wheel_turn >= 0.f ? (g_wheel_turn + 0.5f) : (g_wheel_turn - 0.5f));
	/* turn 负（左滑）→ 下一程序 → 索引增大：preview = sel - step */
	int32_t idx = g_wheel_sel - step;
	if(idx < 0) idx = 0;
	if(idx >= TOTAL_PROGRAMS) idx = TOTAL_PROGRAMS - 1;
	return idx;
}

/* 程序索引夹紧到 0..TOTAL_PROGRAMS-1（主页轮播线性、不循环） */
static int32_t wheel_clamp_sel(int32_t v)
{
	if(v < 0) return 0;
	if(v >= TOTAL_PROGRAMS) return TOTAL_PROGRAMS - 1;
	return v;
}

//程序索引在 0..TOTAL_PROGRAMS-1 范围内循环（非轮播场景仍可用）
static int32_t wheel_mod_total(int32_t v)  //程序索引取模
{
	v %= TOTAL_PROGRAMS;
	if(v < 0) v += TOTAL_PROGRAMS;
	return v;
}

/* ---------- 轮播 turn 动画：编码器连续过渡 / 触摸松手吸附 ---------- */

static void carousel_anim_exec_cb(void * var, int32_t v)
{
	(void)var;
	g_wheel_turn = (float)v / (float)WHEEL_TURN_ANIM_SCALE;
	carousel_wrap_relayout();
}

static void carousel_anim_completed_cb(lv_anim_t * a)
{
	(void)a;
	g_wheel_anim_active = false;
	if(g_wheel_anim_commit_delta != 0) {
		g_wheel_sel = wheel_clamp_sel(g_wheel_sel + g_wheel_anim_commit_delta);
		g_wheel_anim_commit_delta = 0;
	}
	g_wheel_turn = 0.f;
	carousel_wrap_relayout();
	home_sync_program_labels();
}

static void carousel_anim_stop(bool commit)
{
	lv_anim_delete(&g_wheel_turn, carousel_anim_exec_cb);
	g_wheel_anim_active = false;
	if(commit && g_wheel_anim_commit_delta != 0) {
		g_wheel_sel = wheel_clamp_sel(g_wheel_sel + g_wheel_anim_commit_delta);
		g_wheel_anim_commit_delta = 0;
		g_wheel_turn = 0.f;
		carousel_wrap_relayout();
		home_sync_program_labels();
	}
	else if(!commit) {
		g_wheel_anim_commit_delta = 0;
	}
}

/* target_turn：视觉偏移目标；commit_delta：到位后 g_wheel_sel 增量（下一程序为正） */
static void carousel_anim_to_turn(float target_turn, int32_t commit_delta, uint32_t duration_ms)
{
	lv_anim_delete(&g_wheel_turn, carousel_anim_exec_cb);

	g_wheel_anim_commit_delta = commit_delta;
	g_wheel_anim_active = true;

	int32_t start_v = (int32_t)(g_wheel_turn * (float)WHEEL_TURN_ANIM_SCALE);
	int32_t end_v = (int32_t)(target_turn * (float)WHEEL_TURN_ANIM_SCALE);
	if(start_v == end_v) {
		carousel_anim_completed_cb(NULL);
		return;
	}
	if(duration_ms < 80u) duration_ms = 80u;
	if(duration_ms > 400u) duration_ms = 400u;

	lv_anim_t a;
	lv_anim_init(&a);
	lv_anim_set_var(&a, &g_wheel_turn);
	lv_anim_set_values(&a, start_v, end_v);
	lv_anim_set_duration(&a, duration_ms);
	lv_anim_set_exec_cb(&a, carousel_anim_exec_cb);
	lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
	lv_anim_set_completed_cb(&a, carousel_anim_completed_cb);
	lv_anim_start(&a);
}

// 触摸滑动改程序后，将编码器焦点保持在轮播并进入 editing
static void home_sync_encoder_focus_after_carousel_drag(void)
{
	if(g_group_home == NULL) return;
	if(g_home_carousel_enc != NULL) {
		lv_group_focus_obj(g_home_carousel_enc);
		lv_group_set_editing(g_group_home, true);
	}
}

/* 主页编码器：仅轮播代理；常驻 editing，旋转直接切程序 */
static void home_encoder_group_build(void)
{
	if(g_group_home == NULL) return;
	lv_group_remove_all_objs(g_group_home);
	if(g_home_carousel_enc != NULL) {
		ui_encoder_group_add(g_group_home, g_home_carousel_enc);
		lv_group_set_wrap(g_group_home, false);
		lv_group_focus_obj(g_home_carousel_enc);
		lv_group_set_editing(g_group_home, true);
	}
}

/* 编码器步进：动画 turn → -delta，到位后提交 sel；到头不再循环 */
static void home_carousel_encoder_step(int32_t delta)
{
	if(delta == 0) return;
	if(g_wheel_dragging) return;

	int32_t new_commit = g_wheel_anim_commit_delta + delta;
	/* 夹到合法选中范围，两端停住 */
	int32_t min_c = -g_wheel_sel;
	int32_t max_c = (TOTAL_PROGRAMS - 1) - g_wheel_sel;
	if(new_commit < min_c) new_commit = min_c;
	if(new_commit > max_c) new_commit = max_c;
	if(new_commit == g_wheel_anim_commit_delta && new_commit == 0 && delta != 0) {
		/* 已在端点且继续往外转：无动作 */
		return;
	}
	if(new_commit == g_wheel_anim_commit_delta && g_wheel_anim_active) {
		return;
	}

	float target = -(float)new_commit;
	float dist = target - g_wheel_turn;
	if(dist < 0.f) dist = -dist;
	uint32_t ms = (uint32_t)(dist * (float)WHEEL_ENC_ANIM_MS_PER_STEP);
	carousel_anim_to_turn(target, new_commit, ms);
}

/* 主页启停：触摸点击进支付（与编码器短按同一 FSM 路径） */
static void cb_home_runpause(lv_event_t * e)
{
	lv_event_code_t code = lv_event_get_code(e);
	if(code == LV_EVENT_CLICKED) {
		cb_runpause(e);
		return;
	}
	if(code == LV_EVENT_RELEASED && g_home_btn_runpause != NULL) {
		lv_obj_remove_state(g_home_btn_runpause, LV_STATE_PRESSED);
	}
}

/* 轮播编码器：旋转选程序；短按等同启停 → 进支付 */
static void cb_home_carousel_encoder(lv_event_t * e)
{
	if(g_ui_child_lock) return;
	if(g_home_carousel_enc == NULL || lv_event_get_target(e) != g_home_carousel_enc) return;

	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_FOCUSED) {
		if(g_group_home != NULL) {
			lv_group_set_editing(g_group_home, true);
		}
		return;
	}
	if(code == LV_EVENT_CLICKED) {
		/* 仅编码器短按进支付；指针点击空白区不应命中本对象（已去 CLICKABLE） */
		lv_indev_t * indev = lv_indev_get_act();
		if(indev != NULL && lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER) {
			return;
		}
		cb_runpause(e);
		lv_event_stop_processing(e);
		return;
	}
	if(code == LV_EVENT_KEY) {
		uint32_t key = lv_event_get_key(e);
		if(key == LV_KEY_LEFT) {
			home_carousel_encoder_step(-1);
			lv_event_stop_processing(e);
		}
		else if(key == LV_KEY_RIGHT) {
			home_carousel_encoder_step(1);
			lv_event_stop_processing(e);
		}
	}
}

/* 轮播指示点：触摸点击时带动画切到对应程序（线性，不绕环） */
static void cb_home_prog_dot_focus(lv_event_t * e)
{
	if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
	int32_t idx = (int32_t)(intptr_t)lv_event_get_user_data(e);
	if(idx < 0 || idx >= TOTAL_PROGRAMS) return;

	carousel_anim_stop(true);
	if(g_wheel_sel == idx) return;

	int32_t delta = idx - g_wheel_sel;
	float target = -(float)delta;
	float dist = target < 0.f ? -target : target;
	uint32_t ms = (uint32_t)(dist * (float)WHEEL_ENC_ANIM_MS_PER_STEP);
	carousel_anim_to_turn(target, delta, ms);
}

//轮播区点击事件，处理轮播区的按下、拖动、释放等事件，实现卡片的滑动切换和点击判定
static void wheel_pointer_cb(lv_event_t * e)  //轮播区按下/拖动/释放
{
	if(g_ui_child_lock) return;
	lv_event_code_t code = lv_event_get_code(e);
	lv_indev_t * indev = lv_indev_get_act();
	if(indev == NULL) return;
	lv_point_t p;
	lv_indev_get_point(indev, &p);

	if(code == LV_EVENT_PRESSED) {
		/* 打断动画并提交，保证拖动从整数槽开始 */
		carousel_anim_stop(true);
		g_wheel_press_x = p.x;
		g_wheel_dragging = true;
		g_wheel_turn = 0.f;
		carousel_wrap_relayout();
		return;
	}

	if(code == LV_EVENT_PRESSING && g_wheel_dragging) {
		lv_coord_t dx = p.x - g_wheel_press_x;
		float turn = (float)dx / (float)s_mode_step_x;
		/* 线性边界：不能拖出首/末程序 */
		float max_pos = (float)g_wheel_sel;                         /* 右滑 → 更小索引 */
		float max_neg = (float)((TOTAL_PROGRAMS - 1) - g_wheel_sel); /* 左滑 → 更大索引 */
		if(turn > max_pos) turn = max_pos;
		if(turn < -max_neg) turn = -max_neg;
		g_wheel_turn = turn;
		carousel_wrap_relayout();
		return;
	}

	if(code == LV_EVENT_RELEASED && g_wheel_dragging) {
		lv_coord_t dx = p.x - g_wheel_press_x;
		g_wheel_dragging = false;

		if(LV_ABS(dx) < 14) {
			carousel_anim_to_turn(0.f, 0, WHEEL_SNAP_ANIM_MS / 2u);
			return;
		}

		int k;
		if(dx < 0) k = (-dx) / s_mode_drag_snap_px;
		else k = dx / s_mode_drag_snap_px;
		if(k == 0) k = 1;
		if(k > 3) k = 3;

		int32_t delta = (dx < 0) ? k : -k;
		int32_t next = wheel_clamp_sel(g_wheel_sel + delta);
		delta = next - g_wheel_sel;
		if(delta == 0) {
			carousel_anim_to_turn(0.f, 0, WHEEL_SNAP_ANIM_MS / 2u);
			home_sync_encoder_focus_after_carousel_drag();
			return;
		}
		float target = -(float)delta;
		carousel_anim_to_turn(target, delta, WHEEL_SNAP_ANIM_MS);
		home_sync_encoder_focus_after_carousel_drag();
	}
}







static void cb_load_screen(lv_event_t * e)
{
	if(g_ui_child_lock) return;
	lv_obj_t * scr = (lv_obj_t *)lv_event_get_user_data(e);
	if(scr == NULL) return;
	/* 5.2.1：循环模式运行页返回 → 中断而非回主页 */
	if(g_cycle_active && lv_scr_act() == g_scr_running && scr == g_scr_home) {
		cycle_abort_run();
		return;
	}
	ui_screen_load(scr);  //加载目标屏幕
}

//通用界面加载事件回调，加载管理员界面
static void cb_load_admin(lv_event_t * e)
{
	(void)e;
	if(g_ui_child_lock) return;
	ui_screen_load(g_scr_admin);
}

//轮播区布局参数初始化
static void ui_layout_init(void)
{
	s_mode_card_sz = 380;             //轮播区卡片尺寸（px）
	s_mode_step_x = 215;              //拖动换算基准（平均槽距，px）
	s_mode_gap_outer = 200;           //第 1-2、第 4-5 间距（较窄）
	s_mode_gap_inner = 240;           //第 2-3、第 3-4 间距（较宽，离中心更远）
	s_mode_row_shift_up = 0;          //轮播区卡片相对水平中心向上偏移（px），使卡片位置更协调
	s_mode_drag_snap_px = 150;        //每满一格 s_mode_drag_snap_px 多切 1 个程序
}

//设置屏幕基础样式（黑底、无边框）
static void style_screen_base(lv_obj_t * scr)  //设置屏幕基础样式（黑底、无边框）
{
	lv_obj_set_style_bg_color(scr, lv_color_hex(COL_BG), LV_PART_MAIN);   //设置 背景颜色 COL_BG
	lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);             //设置 背景不透明
	lv_obj_set_style_border_width(scr, 0, LV_PART_MAIN);                  //设置 边框宽度为 0
	lv_obj_set_style_pad_all(scr, 0, LV_PART_MAIN);                       //设置 内边距为 0
	lv_obj_set_style_radius(scr,0,LV_PART_MAIN);                          //设置 圆角半径 0
}

/* 默认主题只为部分控件加了 LV_STATE_FOCUS_KEY 描边；imagebutton 等需自行设置。
 * 颜色与粗细对齐 lv_theme_default 的 outline_primary（theme 主色、LV_OPA_50、dpx 3 线宽与内边距） */
static void style_obj_encoder_focus_outline_color(lv_obj_t * obj, uint32_t color_hex, lv_opa_t opa)
{
	if(obj == NULL) return;
	lv_display_t * disp = lv_obj_get_display(obj);
	lv_coord_t ow = 3;
	if(disp != NULL) ow = (lv_coord_t)lv_display_dpx(disp, 3);

	lv_obj_set_style_outline_color(obj, lv_color_hex(color_hex), LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_width(obj, ow, LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_pad(obj, ow, LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_opa(obj, opa, LV_STATE_FOCUS_KEY);
}

//为控件设置编码器/按键焦点描边样式（默认白描边 50%）
static void style_obj_encoder_focus_outline(lv_obj_t * obj)
{
	style_obj_encoder_focus_outline_color(obj, COL_TEXT, LV_OPA_50);
}

/* 编码器 group：控件创建后立即按顺序加入 */
static void ui_encoder_group_add(lv_group_t * group, lv_obj_t * obj)
{
	if(group == NULL || obj == NULL) return;
	lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICK_FOCUSABLE);
	style_obj_encoder_focus_outline(obj);
	lv_group_add_obj(group, obj);
}

/* 内贴 border：FOCUS_KEY + FOCUSED 双态（rebuild 程序化 focus 可能仅有 FOCUSED） */
static void ui_encoder_style_inner_border_focus_states(lv_obj_t * obj, uint32_t color_hex)
{
	if(obj == NULL) return;
	lv_display_t * disp = lv_obj_get_display(obj);
	lv_coord_t bw = 6;
	if(disp != NULL) bw = (lv_coord_t)lv_display_dpx(disp, 6);
	static const lv_state_t focus_states[] = { LV_STATE_FOCUS_KEY, LV_STATE_FOCUSED };
	for(size_t i = 0; i < sizeof(focus_states) / sizeof(focus_states[0]); i++) {
		lv_state_t st = focus_states[i];
		lv_obj_set_style_border_color(obj, lv_color_hex(color_hex), LV_PART_MAIN | st);
		lv_obj_set_style_border_width(obj, bw, LV_PART_MAIN | st);
		lv_obj_set_style_border_opa(obj, LV_OPA_COVER, LV_PART_MAIN | st);
		lv_obj_set_style_radius(obj, 4, LV_PART_MAIN | st);
	}
}

/* 程序设置 textarea/roller：获焦时在白框内侧显示橙色 border（不用外扩 outline） */
static void style_prog_field_encoder_focus_inner(lv_obj_t * obj)
{
	if(obj == NULL) return;
	lv_obj_set_style_outline_width(obj, 0, LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_opa(obj, LV_OPA_TRANSP, LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_width(obj, 0, LV_STATE_FOCUSED);
	lv_obj_set_style_outline_opa(obj, LV_OPA_TRANSP, LV_STATE_FOCUSED);

	lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	ui_encoder_style_inner_border_focus_states(obj, COL_ORANGE);
}

//程序设置字段加入 group（内贴橙框）
static void ui_encoder_group_add_prog_field(lv_group_t * group, lv_obj_t * obj)
{
	if(group == NULL || obj == NULL) return;
	lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICK_FOCUSABLE);
	style_prog_field_encoder_focus_inner(obj);
	lv_group_add_obj(group, obj);
}

/* 屏幕亮度滑条：获焦时内侧白色 border（尺寸同程序设置字段，颜色 COL_TEXT） */
static void style_brightness_slider_encoder_focus_inner(lv_obj_t * obj)
{
	if(obj == NULL) return;
	lv_obj_set_style_outline_width(obj, 0, LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_opa(obj, LV_OPA_TRANSP, LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_width(obj, 0, LV_STATE_FOCUSED);
	lv_obj_set_style_outline_opa(obj, LV_OPA_TRANSP, LV_STATE_FOCUSED);

	lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	ui_encoder_style_inner_border_focus_states(obj, COL_TEXT);
}

//亮度滑条加入 group（不加描边）
static void ui_encoder_group_add_brightness_slider(lv_group_t * group, lv_obj_t * obj)
{
	if(group == NULL || obj == NULL) return;
	/* 焦点白框画在 g_admin_brightness_slider_focus 上，滑条本体不加描边 */
	lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICK_FOCUSABLE);
	lv_obj_set_style_outline_width(obj, 0, LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_opa(obj, LV_OPA_TRANSP, LV_STATE_FOCUS_KEY);
	lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_FOCUS_KEY);
	lv_group_add_obj(group, obj);
}

/* 亮度滑条：外框左黑右蓝渐变描边，内层纯黑底 */
static void brightness_slider_style_init(void)
{
	if(s_brightness_slider_style_inited) return;

	lv_style_init(&s_brightness_slider_frame_style);
	lv_style_set_radius(&s_brightness_slider_frame_style, 8);
	lv_style_set_bg_opa(&s_brightness_slider_frame_style, LV_OPA_COVER);
	lv_style_set_bg_color(&s_brightness_slider_frame_style, lv_color_hex(BRIGHTNESS_SLIDER_BLACK));
	lv_style_set_bg_grad_color(&s_brightness_slider_frame_style, lv_color_hex(BRIGHTNESS_SLIDER_BLUE));
	lv_style_set_bg_grad_dir(&s_brightness_slider_frame_style, LV_GRAD_DIR_HOR);
	lv_style_set_border_width(&s_brightness_slider_frame_style, 0);
	lv_style_set_pad_all(&s_brightness_slider_frame_style, ADMIN_MENU_BORDER_W);
	lv_style_set_shadow_width(&s_brightness_slider_frame_style, 0);

	lv_style_init(&s_brightness_slider_inner_style);
	lv_style_set_radius(&s_brightness_slider_inner_style, 6);
	lv_style_set_bg_opa(&s_brightness_slider_inner_style, LV_OPA_COVER);
	lv_style_set_bg_color(&s_brightness_slider_inner_style, lv_color_hex(BRIGHTNESS_SLIDER_BLACK));
	lv_style_set_bg_grad_color(&s_brightness_slider_inner_style, lv_color_hex(BRIGHTNESS_SLIDER_BLACK));
	lv_style_set_bg_grad_dir(&s_brightness_slider_inner_style, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&s_brightness_slider_inner_style, 0);
	lv_style_set_pad_all(&s_brightness_slider_inner_style, 0);

	s_brightness_slider_style_inited = true;
}

/* 在已创建的渐变外框 frame 内填充 bar / 透明 slider / 编码器焦点层 */
static void admin_gradient_slider_fill_inner(lv_obj_t * frame, lv_obj_t ** out_bar, lv_obj_t ** out_slider,
                                             lv_obj_t ** out_focus)
{
	if(frame == NULL) return;

	lv_obj_t * inner = lv_obj_create(frame);
	lv_obj_remove_style_all(inner);
	lv_obj_add_style(inner, &s_brightness_slider_inner_style, LV_PART_MAIN);
	lv_obj_set_size(inner, LV_PCT(100), LV_PCT(100));
	lv_obj_clear_flag(inner, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

	lv_obj_t * bar = lv_bar_create(inner);
	lv_obj_set_size(bar, LV_PCT(100), LV_PCT(100));
	lv_bar_set_range(bar, 0, 100);
	lv_obj_set_style_bg_opa(bar, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(bar, 0, LV_PART_MAIN);
	lv_obj_set_style_radius(bar, 4, LV_PART_MAIN);
	lv_obj_set_style_pad_all(bar, 2, LV_PART_MAIN);
	lv_obj_set_style_bg_color(bar, lv_color_hex(BRIGHTNESS_SLIDER_BLACK), LV_PART_INDICATOR);
	lv_obj_set_style_bg_grad_color(bar, lv_color_hex(BRIGHTNESS_SLIDER_BLUE), LV_PART_INDICATOR);
	lv_obj_set_style_bg_grad_dir(bar, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
	lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_INDICATOR);
	lv_obj_set_style_radius(bar, 2, LV_PART_INDICATOR);
	lv_obj_remove_flag(bar, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

	lv_obj_t * slider = lv_slider_create(inner);
	lv_obj_set_size(slider, LV_PCT(100), LV_PCT(100));
	lv_slider_set_range(slider, 0, 100);
	lv_obj_set_style_bg_opa(slider, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(slider, 0, LV_PART_MAIN);
	lv_obj_set_style_radius(slider, 4, LV_PART_MAIN);
	lv_obj_set_style_pad_all(slider, 2, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(slider, LV_OPA_TRANSP, LV_PART_INDICATOR);
	lv_obj_set_style_bg_opa(slider, LV_OPA_TRANSP, LV_PART_KNOB);
	lv_obj_set_style_border_opa(slider, LV_OPA_TRANSP, LV_PART_KNOB);
	lv_obj_set_style_shadow_opa(slider, LV_OPA_TRANSP, LV_PART_KNOB);
	lv_obj_set_style_width(slider, 0, LV_PART_KNOB);
	lv_obj_set_style_height(slider, 0, LV_PART_KNOB);
	lv_obj_set_style_pad_all(slider, 0, LV_PART_KNOB);

	lv_obj_t * focus = lv_obj_create(inner);
	lv_obj_remove_style_all(focus);
	lv_obj_set_size(focus, LV_PCT(100), LV_PCT(100));
	style_brightness_slider_encoder_focus_inner(focus);
	lv_obj_remove_flag(focus, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_add_flag(focus, LV_OBJ_FLAG_EVENT_BUBBLE);
	lv_obj_move_foreground(focus);

	if(out_bar != NULL) *out_bar = bar;
	if(out_slider != NULL) *out_slider = slider;
	if(out_focus != NULL) *out_focus = focus;
}

/* 轮播卡片：编码器可选程序，但不显示焦点描边 */
static void ui_encoder_group_add_no_outline(lv_group_t * group, lv_obj_t * obj)
{
	if(group == NULL || obj == NULL) return;
	lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICK_FOCUSABLE);
	lv_obj_set_style_outline_width(obj, 0, LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_opa(obj, LV_OPA_TRANSP, LV_STATE_FOCUS_KEY);
	lv_group_add_obj(group, obj);
}

// 将管理员键盘加入编码器 group：不加外层描边，焦点由内部按键呈现
static void admin_encoder_group_add_kb(lv_group_t * group)
{
	if(group == NULL || g_admin_kb == NULL) return;
	lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_CLICK_FOCUSABLE);
	lv_obj_set_style_outline_width(g_admin_kb, 0, LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_opa(g_admin_kb, LV_OPA_TRANSP, LV_STATE_FOCUS_KEY);
	lv_group_add_obj(group, g_admin_kb);
}

// 判断管理员数字键盘是否处于显示状态
static bool admin_kb_is_visible(void)
{
	if(g_admin_kb == NULL) return false;
	return !lv_obj_has_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
}

// 统计数字键盘 map 中的按钮总数（不含换行）
static uint32_t admin_kb_btn_count(const lv_obj_t * kb)
{
	uint32_t cnt = 0;
	const char * const * map = lv_buttonmatrix_get_map(kb);
	if(map == NULL) return 0;
	for(uint32_t i = 0; map[i] != NULL && map[i][0] != '\0'; i++) {
		if(lv_strcmp(map[i], "\n") != 0) cnt++;
	}
	return cnt;
}

// 编码器逐步切换键盘按键：1→2→3→… 顺序切换，到首尾停止不循环
static void admin_kb_encoder_step(lv_obj_t * kb, int32_t delta)
{
	uint32_t cnt = admin_kb_btn_count(kb);
	if(cnt == 0) return;

	uint32_t cur = lv_keyboard_get_selected_button(kb);
	if(cur == LV_BUTTONMATRIX_BUTTON_NONE) {
		cur = (delta >= 0) ? 0 : (cnt - 1);
	}
	else if(delta > 0) {
		if(cur + 1 >= cnt) return;
		cur++;
	}
	else {
		if(cur == 0) return;
		cur--;
	}

	lv_buttonmatrix_set_selected_button(kb, cur);
	lv_obj_invalidate(kb);
}

// 管理员键盘编码器选中首键「1」
static void admin_kb_encoder_select_first(lv_obj_t * kb)
{
	if(kb == NULL) return;
	lv_buttonmatrix_set_selected_button(kb, 0);
	lv_obj_invalidate(kb);
}

// 管理员键盘编码器事件：聚焦时补选按键，旋转时按 map 顺序逐步切换
static void cb_admin_kb_encoder(lv_event_t * e)
{
	lv_obj_t * kb = lv_event_get_target_obj(e);
	lv_event_code_t code = lv_event_get_code(e);
	if(kb == NULL) return;

	if(code == LV_EVENT_FOCUSED) {
		if(g_group_admin != NULL && (g_group_admin && lv_group_get_editing(g_group_admin)) &&
		   lv_keyboard_get_selected_button(kb) == LV_BUTTONMATRIX_BUTTON_NONE) {
			admin_kb_encoder_select_first(kb);
		}
	}
	else if(code == LV_EVENT_KEY) {
		if(g_group_admin != NULL && (g_group_admin && lv_group_get_editing(g_group_admin))) {
			uint32_t key = lv_event_get_key(e);
			if(key == LV_KEY_RIGHT) {
				admin_kb_encoder_step(kb, +1);
				lv_event_stop_processing(e);
			}
			else if(key == LV_KEY_LEFT) {
				admin_kb_encoder_step(kb, -1);
				lv_event_stop_processing(e);
			}
		}
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		lv_obj_invalidate(kb);
	}
}

// 为键盘内部按键设置编码器焦点描边，并注册编码器事件
static void admin_kb_encoder_style_init(void)
{
	if(g_admin_kb == NULL || s_admin_kb_encoder_inited) return;

	lv_display_t * disp = lv_obj_get_display(g_admin_kb);
	lv_coord_t ow = 3;
	if(disp != NULL) ow = (lv_coord_t)lv_display_dpx(disp, 3);

	lv_obj_set_style_outline_color(g_admin_kb, lv_color_hex(COL_TEXT), LV_PART_ITEMS | LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_width(g_admin_kb, ow, LV_PART_ITEMS | LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_pad(g_admin_kb, ow, LV_PART_ITEMS | LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_opa(g_admin_kb, LV_OPA_COVER, LV_PART_ITEMS | LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_color(g_admin_kb, lv_color_hex(COL_TEXT), LV_PART_ITEMS | LV_STATE_FOCUS_KEY | LV_STATE_CHECKED);
	lv_obj_set_style_outline_width(g_admin_kb, ow, LV_PART_ITEMS | LV_STATE_FOCUS_KEY | LV_STATE_CHECKED);
	lv_obj_set_style_outline_pad(g_admin_kb, ow, LV_PART_ITEMS | LV_STATE_FOCUS_KEY | LV_STATE_CHECKED);
	lv_obj_set_style_outline_opa(g_admin_kb, LV_OPA_COVER, LV_PART_ITEMS | LV_STATE_FOCUS_KEY | LV_STATE_CHECKED);
	lv_obj_set_style_outline_width(g_admin_kb, 0, LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_opa(g_admin_kb, LV_OPA_TRANSP, LV_STATE_FOCUS_KEY);

	lv_obj_add_event_cb(g_admin_kb, cb_admin_kb_encoder, LV_EVENT_FOCUSED, NULL);
	lv_obj_add_event_cb(g_admin_kb, cb_admin_kb_encoder, LV_EVENT_KEY | LV_EVENT_PREPROCESS, NULL);
	lv_obj_add_event_cb(g_admin_kb, cb_admin_kb_encoder, LV_EVENT_VALUE_CHANGED, NULL);
	lv_obj_add_event_cb(g_admin_kb, cb_admin_kb_cancel, LV_EVENT_CANCEL, NULL);

	s_admin_kb_encoder_inited = true;
}

// 编码器进入键盘按键编辑：聚焦键盘、开启编辑模式并选中首键
static void admin_kb_encoder_enter(void)
{
	if(g_admin_kb == NULL || g_group_admin == NULL) return;
	if(!admin_kb_is_visible()) return;
	lv_group_focus_obj(g_admin_kb);
	if(g_group_admin) lv_group_set_editing(g_group_admin, true);
	admin_kb_encoder_select_first(g_admin_kb);
}

// 收起管理员数字键盘（点击小键盘图标触发 LV_EVENT_CANCEL）
static void admin_kb_close(void)
{
	if(g_admin_kb == NULL || !admin_kb_is_visible()) return;

	if(g_admin_view == PROGRAM_SETTINGS) {
		program_admin_ui_save_fields();
		g_admin_ta_prog_active = NULL;
	}

	g_admin_kb_ta = NULL;
	lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);

	if(g_group_admin != NULL) {
		if(g_group_admin) lv_group_set_editing(g_group_admin, false);
		if(g_admin_view == PROGRAM_SETTINGS) {
			if(g_admin_prog_sel >= 0 && g_admin_prog_sel < TOTAL_PROGRAMS &&
			   g_admin_prog_btns[g_admin_prog_sel] != NULL) {
				lv_group_focus_obj(g_admin_prog_btns[g_admin_prog_sel]);
			}
		}
		else if(g_admin_view == PASSWORD && g_admin_ta_pwd != NULL) {
			lv_group_focus_obj(g_admin_ta_pwd);
		}
		else if(g_admin_view == MACHINE_ID && g_admin_ta_machine_id != NULL) {
			lv_group_focus_obj(g_admin_ta_machine_id);
		}
		else if(g_admin_view == VENDOR_SERIAL && g_admin_ta_vendor_serial != NULL) {
			lv_group_focus_obj(g_admin_ta_vendor_serial);
		}
	}
}

// 键盘 CANCEL 回调：小键盘图标按下后由 LVGL 派发
static void cb_admin_kb_cancel(lv_event_t * e)
{
	if(lv_event_get_code(e) != LV_EVENT_CANCEL) return;
	admin_kb_close();
}

/* 弹出数字键盘并进入编码器按键编辑（触摸/编码器共用） */
static void admin_ta_begin_edit(lv_obj_t * ta)
{
	if(g_admin_kb == NULL || ta == NULL) return;
	g_admin_kb_ta = ta;
	lv_obj_remove_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
}

/* 编码器在密码输入框上按 Enter：仅进入键盘编辑，勿触发单行 textarea 的 READY 校验 */
static void cb_admin_ta_key_enter(lv_event_t * e)
{
	if(lv_event_get_code(e) != LV_EVENT_KEY) return;
	if(lv_event_get_key(e) != LV_KEY_ENTER) return;

	lv_obj_t * ta = lv_event_get_target_obj(e);
	if(g_admin_view != PASSWORD && g_admin_view != PASSWORD_CHANGE_OLD &&
	   g_admin_view != PASSWORD_CHANGE_NEW && g_admin_view != VENDOR_SERIAL) return;

	if(g_group_admin != NULL && (g_group_admin && lv_group_get_editing(g_group_admin)) &&
	   admin_kb_is_visible() && g_admin_kb != NULL &&
	   (g_group_admin ? lv_group_get_focused(g_group_admin) : NULL) == g_admin_kb) {
		return;
	}

	if(g_admin_view == PASSWORD_CHANGE_NEW && ta != NULL) {
		if(ta == g_admin_ta_pwd_chg_new1) {
			g_admin_pwd_chg_page1_step = 0;
		}
		else if(ta == g_admin_ta_pwd_chg_new2) {
			g_admin_pwd_chg_page1_step = 1;
		}
	}

	admin_ta_begin_edit(ta);
	lv_event_stop_processing(e);
}

// 密码/机器 ID 输入框：弹出键盘并进入按键逐键选择（与程序设置页一致）
static void cb_admin_ta_kb_focus(lv_event_t * e)
{
	lv_obj_t * ta = lv_event_get_target_obj(e);
	lv_event_code_t code = lv_event_get_code(e);
	if(g_admin_view != PASSWORD && g_admin_view != MACHINE_ID &&
	   g_admin_view != PASSWORD_CHANGE_OLD && g_admin_view != PASSWORD_CHANGE_NEW &&
	   g_admin_view != VENDOR_SERIAL) return;

	if(g_admin_view == PASSWORD_CHANGE_NEW && code == LV_EVENT_CLICKED && ta != NULL) {
		if(ta == g_admin_ta_pwd_chg_new1) {
			g_admin_pwd_chg_page1_step = 0;
		}
		else if(ta == g_admin_ta_pwd_chg_new2) {
			g_admin_pwd_chg_page1_step = 1;
		}
	}

	if(code == LV_EVENT_FOCUSED) {
		if(g_group_admin != NULL && (g_group_admin && lv_group_get_editing(g_group_admin)) && admin_kb_is_visible()) {
			admin_kb_encoder_enter();
		}
	} else if(code == LV_EVENT_CLICKED) {
		admin_ta_begin_edit(ta);
	}
}

static void create_screens(void) //创建界面，并按当前显示分辨率设满屏宽高
{
	g_scr_off = lv_obj_create(NULL);
	g_scr_home = lv_obj_create(NULL);
	g_scr_running = lv_obj_create(NULL);
	g_scr_end = lv_obj_create(NULL);
	g_scr_pay = lv_obj_create(NULL);
	g_scr_pay_done = lv_obj_create(NULL);
	g_scr_add_time = lv_obj_create(NULL);
	g_scr_admin = lv_obj_create(NULL);
	g_scr_selfcheck = lv_obj_create(NULL);
	g_scr_cycle = lv_obj_create(NULL);
	style_screen_base(g_scr_off);  //设置屏幕基础样式（黑底、无边框）
	style_screen_base(g_scr_home);                 //主页
	style_screen_base(g_scr_running);              //运行
	style_screen_base(g_scr_end);                  //运行结束
	style_screen_base(g_scr_pay);                  //支付
	style_screen_base(g_scr_pay_done);             //支付完成
	style_screen_base(g_scr_add_time);             //追加时间
	style_screen_base(g_scr_admin);                //管理员
	style_screen_base(g_scr_selfcheck);            //有水自检
	style_screen_base(g_scr_cycle);                //循环程序（寿命试验）
	lv_obj_set_size(g_scr_off, UI_FIXED_W, UI_FIXED_H);
	lv_obj_set_size(g_scr_home, UI_FIXED_W, UI_FIXED_H);              //主页宽高
	lv_obj_set_size(g_scr_running, UI_FIXED_W, UI_FIXED_H);           //运行宽高
	lv_obj_set_size(g_scr_end, UI_FIXED_W, UI_FIXED_H);
	lv_obj_set_size(g_scr_pay, UI_FIXED_W, UI_FIXED_H);
	lv_obj_set_size(g_scr_pay_done, UI_FIXED_W, UI_FIXED_H);
	lv_obj_set_size(g_scr_add_time, UI_FIXED_W, UI_FIXED_H);
	lv_obj_set_size(g_scr_admin, UI_FIXED_W, UI_FIXED_H);
	lv_obj_set_size(g_scr_selfcheck, UI_FIXED_W, UI_FIXED_H);
	lv_obj_set_size(g_scr_cycle, UI_FIXED_W, UI_FIXED_H);
}

//将编码器/键盘输入设备绑定到指定 focus group
static void ui_set_encoder_group(lv_group_t * group)  //切换编码器 group（NULL 清除）
{
	lv_indev_t * indev = NULL;
	while((indev = lv_indev_get_next(indev)) != NULL) {
		lv_indev_type_t type = lv_indev_get_type(indev);
		if(type == LV_INDEV_TYPE_ENCODER || type == LV_INDEV_TYPE_KEYPAD) {
			lv_indev_set_group(indev, group);
		}
	}
}

/* 屏幕加载包装：切换 scr、编码器 group，并按目标页刷新相关 label */
static void ui_screen_load(lv_obj_t * scr)
{
	if(scr == NULL) return;                              /* 空目标直接返回 */

	running_child_lock_remote_clear_if_leaving_running(scr); /* 离运行页则关童锁 */

	if(scr != g_scr_running) {
		running_countdown_reset_all();                   /* 非运行页：停倒计时与闪烁 */
	}
	if(scr != g_scr_pay_done) {
		pay_done_timer_stop();                           /* 非支付完成页：停 2s 跳转定时器 */
	}

	lv_screen_load(scr);                                 /* LVGL 切换活动屏幕 */

	if(scr == g_scr_home) {
		home_encoder_group_build();
		ui_set_encoder_group(g_group_home);
	}
	else if(scr == g_scr_off) {
		ui_set_encoder_group(NULL);
	}
	else if(scr == g_scr_running) {
		ui_set_encoder_group(NULL);
		running_screen_sync_mode_name();
		running_live_params_sync();
		if(g_cycle_active && g_cycle_ui_state == CYCLE_UI_FAULT && g_cycle_fault_on_running) {
			cycle_fault_blink_start();
		} else {
			running_countdown_start();
		}
	}
	else if(scr == g_scr_end) {
		add_time_session_clear();
		ui_set_encoder_group(NULL);
	}
	else if(scr == g_scr_add_time) {
		g_add_time_sel = 0;
		add_time_page_sync_labels();
		ui_set_encoder_group(NULL);
	}
	else if(scr == g_scr_pay) {
		/* 支付页：将当前程序 cfg 写入 param_change[] 供 MCU/通信（时间为分钟） */
		param_change[0] = g_prog_cfg[g_wheel_sel].init_dry_min;
		param_change[1] = g_prog_cfg[g_wheel_sel].cool_min;
		param_change[2] = g_session_active ? (uint16_t)g_session_add_count :
			(g_prog_cfg[g_wheel_sel].add_count <= 0 ? 0 : g_prog_cfg[g_wheel_sel].add_count);
		param_change[3] = g_prog_cfg[g_wheel_sel].add_time_min <= 0 ? 0 : g_prog_cfg[g_wheel_sel].add_time_min;
		param_change[4] = g_prog_cfg[g_wheel_sel].temp_idx < 0 ? 0 : (uint16_t)program_admin_temp_celsius(g_wheel_sel, g_prog_cfg[g_wheel_sel].temp_idx);
		param_change[5] = g_prog_cfg[g_wheel_sel].price < 0 ? 0 : (uint16_t)g_prog_cfg[g_wheel_sel].price;
		param_change[6] = g_prog_cfg[g_wheel_sel].add_price < 0 ? 0 : (uint16_t)g_prog_cfg[g_wheel_sel].add_price;
		switch(g_wheel_sel) {
			case 0: { param_change[7] = 1; break; }
			case 1: { param_change[7] = 2; break; }
			case 2: { param_change[7] = 3; break; }
			case 3: { param_change[7] = 4; break; }
			case 4: { param_change[7] = 5; break; }
			default: { param_change[7] = 3; break; }
		}
		ui_set_encoder_group(NULL);
		pay_sync_price_label();
		pay_sync_pay_ui();
		g_pay_focus_back_on_enter = false;
	}
	else if(scr == g_scr_pay_done) {
		ui_set_encoder_group(NULL);
		pay_done_timer_start();
	}
	else if(scr == g_scr_admin) {
		admin_session_reset();
		ui_set_encoder_group(NULL);
	}
	else if(scr == g_scr_selfcheck) {
		selfcheck_ui_reset();
		ui_set_encoder_group(NULL);
	}
	else if(scr == g_scr_cycle) {
		cycle_fault_blink_stop();
		if(g_cycle_ui_state == CYCLE_UI_FAULT) {
			cycle_fault_blink_start();
		}
		cycle_run_count_label_sync();
		ui_set_encoder_group(NULL);
	}
	else {
		ui_set_encoder_group(NULL);
	}
	ui_idle_on_screen_changed(scr);                    /* 待机页暂停空闲计时 */
}

/* 当前屏幕不进入空闲待机（待机页、运行页、洗涤完成页、报警弹层可见、管理员页常亮） */
static bool ui_alarm_overlay_is_visible(void);  //前向声明：报警弹层是否正在显示

static bool ui_idle_screen_keeps_awake(lv_obj_t * scr)
{
	if(scr == NULL) return false;
	if(ui_alarm_overlay_is_visible()) return true;     //报警弹层可见时保持常亮
	return scr == g_scr_off || scr == g_scr_running || scr == g_scr_end || scr == g_scr_admin ||
	       scr == g_scr_selfcheck || scr == g_scr_cycle;
}

//重置空闲计时（有输入时调用）
static void ui_idle_reset(void)
{
    if(g_idle_timer == NULL) return;
    if(ui_idle_screen_keeps_awake(lv_scr_act())) return;
    if(g_ui_dormancy_no_sleep || g_ui_dormancy_timeout_ms == UI_DORMANCY_DISABLED_MS) return;
    lv_timer_reset(g_idle_timer);
    lv_timer_resume(g_idle_timer);
}

//进入/离开待机页时暂停或恢复空闲计时
static void ui_idle_on_screen_changed(lv_obj_t * scr)
{
    if(g_idle_timer == NULL) return;
    if(ui_idle_screen_keeps_awake(scr)) {
        lv_timer_pause(g_idle_timer);
        g_idle_last_ptr_x = -1;
        g_idle_last_ptr_y = -1;
    }
    else {
        ui_idle_reset();
    }
}

//检测鼠标移动，重置空闲计时
static void ui_idle_poll_pointer(void)
{
    if(ui_idle_screen_keeps_awake(lv_scr_act())) return;

    lv_indev_t * indev = NULL;
    while((indev = lv_indev_get_next(indev)) != NULL) {
        if(lv_indev_get_type(indev) != LV_INDEV_TYPE_POINTER) continue;

        lv_point_t p;
        lv_indev_get_point(indev, &p);
        if(g_idle_last_ptr_x >= 0 &&
           (p.x != g_idle_last_ptr_x || p.y != g_idle_last_ptr_y)) {
            ui_idle_reset();
        }
        g_idle_last_ptr_x = p.x;
        g_idle_last_ptr_y = p.y;
        break;
    }
}

//空闲超时，进入待机页
static void cb_idle_timeout(lv_timer_t * t)
{
    (void)t;
    if(g_scr_off == NULL) return;
    lv_obj_t * cur = lv_scr_act();
    if(ui_idle_screen_keeps_awake(cur)) return;
    g_scr_before_off = cur;                              /* 记录待机前界面 */
    ui_screen_load(g_scr_off);
}

//输入设备活动：重置空闲计时
static void cb_indev_activity(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_PRESSING || code == LV_EVENT_RELEASED ||
       code == LV_EVENT_CLICKED || code == LV_EVENT_LONG_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT ||
       code == LV_EVENT_KEY || code == LV_EVENT_ROTARY || code == LV_EVENT_GESTURE) {
        ui_idle_reset();
    }
}

//为鼠标/编码器/键盘注册活动监听
static void ui_idle_indev_hook(void)
{
    lv_indev_t * indev = NULL;
    while((indev = lv_indev_get_next(indev)) != NULL) {
        lv_indev_type_t type = lv_indev_get_type(indev);
        if(type == LV_INDEV_TYPE_POINTER || type == LV_INDEV_TYPE_ENCODER || type == LV_INDEV_TYPE_KEYPAD) {
            lv_indev_add_event_cb(indev, cb_indev_activity, LV_EVENT_ALL, NULL);
        }
    }
}

/* 按 g_ui_dormancy_timeout_ms 刷新空闲定时器（0=不熄屏则暂停） */
static void ui_idle_init(void)
{
    if(g_idle_timer == NULL) {
        g_idle_timer = lv_timer_create(cb_idle_timeout, g_ui_dormancy_timeout_ms, NULL);
    }
    ui_idle_apply_dormancy_period();
    ui_idle_indev_hook();
}

/* 待机页触摸/点击：回到待机前界面；开机首屏待机（g_scr_before_off==NULL）则进主页 */
static void cb_off_wake(lv_event_t * e)
{
	lv_event_code_t code = lv_event_get_code(e);
	if(code != LV_EVENT_CLICKED) return;
	if(lv_scr_act() != g_scr_off) return;

	if(fsm.state == FSM_OFF) {
		fsm.state = FSM_STANDBY;
		need_scr_load = false;
	}

	lv_obj_t * target = g_scr_before_off;
	if(target == NULL || target == g_scr_off) {
		target = g_scr_home;
	}
	ui_screen_load(target);
}

//统一设置指针/编码器长按判定时间
static void ui_apply_indev_long_press_ms(uint16_t ms)  //统一设置指针/编码器长按判定时间
{
	lv_indev_t * indev = NULL;
	while((indev = lv_indev_get_next(indev)) != NULL) {
		lv_indev_type_t t = lv_indev_get_type(indev);
		if(t == LV_INDEV_TYPE_POINTER || t == LV_INDEV_TYPE_ENCODER) {
			lv_indev_set_long_press_time(indev, ms);
		}
	}
}

//童锁激活时的进入钩子（预留扩展）
static void running_child_lock_on_enter(void)  //童锁激活时的进入钩子（预留扩展）
{
	FsmReq fr = {
		.fsm_req_type = FSM_EVT_CHILD_LOCK_ON,
		.next_state = fsm.state,
	};
	xQueueSend(r.q_fsm, &fr, portMAX_DELAY);
}

//童锁解除时的退出钩子（预留扩展）
static void running_child_lock_on_exit(void)  //童锁解除时的退出钩子（预留扩展）
{
	FsmReq fr = {
		.fsm_req_type = FSM_EVT_CHILD_LOCK_OFF,
		.next_state = fsm.state,
	};
	xQueueSend(r.q_fsm, &fr, portMAX_DELAY);
}

/* 中间栏（程序名+倒计时）尺寸变化时，童锁按钮重新对齐到其右侧 */
static void cb_running_mid_layout_changed(lv_event_t * e)
{
	(void)e;
	running_child_lock_align_btn();
}

//将童锁按钮对齐到运行页中间栏右侧
static void running_child_lock_align_btn(void)  //将童锁按钮对齐到运行页中间栏右侧
{
	if(g_running_child_lock_btn == NULL || g_running_mid == NULL) return;
	lv_obj_t * root = lv_obj_get_parent(g_running_mid);
	if(root != NULL) {
		lv_obj_update_layout(root);                      /* 先刷新布局，再读 mid 宽高 */
	}
	lv_obj_align_to(g_running_child_lock_btn, g_running_mid, LV_ALIGN_RIGHT_MID, 60, 13);
}

//应用童锁锁定/解锁 UI（不操作编码器 group）
static void running_child_lock_apply_locked(bool locked, bool silent_unlock)
{
	if(g_running_child_lock_btn == NULL) return;

	bool* need_child_lock = &g_ui_child_lock;
	if(locked) {
		*need_child_lock = true;
		running_child_lock_on_enter();
	}
	else {
		*need_child_lock = false;
		if(!silent_unlock) {
			running_child_lock_on_exit();
		}
	}

	g_ui_child_lock = locked;

	if(locked) {
		lv_obj_set_style_bg_opa(g_running_child_lock_btn, LV_OPA_COVER, LV_PART_MAIN);
		lv_obj_set_style_bg_color(g_running_child_lock_btn, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
		lv_obj_set_style_border_width(g_running_child_lock_btn, 0, LV_PART_MAIN);
		if(g_running_child_lock_img != NULL) {
			lv_obj_set_style_image_opa(g_running_child_lock_img, LV_OPA_COVER, LV_PART_MAIN);
		}
		if(g_running_lock_blocker != NULL) {
			lv_obj_remove_flag(g_running_lock_blocker, LV_OBJ_FLAG_HIDDEN);
			lv_obj_move_foreground(g_running_lock_blocker);
		}
		if(g_running_btn_power != NULL) {
			lv_obj_move_foreground(g_running_btn_power);
		}
		lv_obj_move_foreground(g_running_child_lock_btn);
	}
	else {
		lv_obj_set_style_bg_opa(g_running_child_lock_btn, LV_OPA_COVER, LV_PART_MAIN);
		lv_obj_set_style_bg_color(g_running_child_lock_btn, lv_color_hex(0x555555), LV_PART_MAIN);
		lv_obj_set_style_border_width(g_running_child_lock_btn, 0, LV_PART_MAIN);
		if(g_running_child_lock_img != NULL) {
			lv_obj_set_style_image_opa(g_running_child_lock_img, LV_OPA_40, LV_PART_MAIN);
		}
		if(g_running_lock_blocker != NULL) {
			lv_obj_add_flag(g_running_lock_blocker, LV_OBJ_FLAG_HIDDEN);
		}
	}
}

//离开运行页时静默解除童锁
static void running_child_lock_remote_clear_if_leaving_running(lv_obj_t * target_scr)  //离开运行页时静默解除童锁
{
	if(target_scr == NULL || g_scr_running == NULL) return;
	if(!g_ui_child_lock) return;
	if(lv_scr_act() == NULL) return;
	if(lv_scr_act() != g_scr_running) return;
	if(target_scr == g_scr_running) return;
	running_child_lock_apply_locked(false, true);  //应用童锁锁定/解锁 UI 与编码器 group 状态
}

//童锁按钮长按：切换童锁开/关
static void cb_running_child_lock_long(lv_event_t * e)  //童锁按钮长按
{
	if(lv_event_get_code(e) != LV_EVENT_LONG_PRESSED) return;
	if(!g_ui_child_lock) {
		running_child_lock_apply_locked(true, false);
	}
	else {
		running_child_lock_apply_locked(false, false);
	}
	(void)e;
}

static void cb_running_child_lock_released(lv_event_t * e)
{
	(void)e;
}







/* 构建主页：顶部栏、程序轮播、底部五列参数栏与语言切换 */
static void build_home(void)
{
		lv_obj_t * scr = g_scr_home;                       /* 主页屏幕对象 */

		lv_obj_t * root = lv_obj_create(scr);              /* 全屏根容器 */
		lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));   /* 根容器铺满屏幕 */
		lv_obj_set_pos(root, 0, 0);                        /* 根容器左上角对齐 */
		lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, LV_PART_MAIN); /* 根背景透明 */
		lv_obj_set_style_border_width(root, 0, LV_PART_MAIN);       /* 无边框 */
		lv_obj_set_style_pad_all(root, 0, LV_PART_MAIN);            /* 无内边距 */
		lv_obj_set_style_radius(root, 0, LV_PART_MAIN);             /* 无圆角 */

		/* --- 顶部栏：时钟；文本按钮「启停」「电源」（不参与中/英切换） --- */
		lv_obj_t * top = create_top_bar(root, &g_lbl_clock, NULL, NULL);
		g_home_btn_runpause = add_top_text_btn(top, "启停", 100); /* 固定中文：启停 */
		lv_obj_add_event_cb(g_home_btn_runpause, cb_home_runpause, LV_EVENT_ALL, g_scr_running);
		lv_obj_add_event_cb(g_home_btn_runpause, cb_runpause_long, LV_EVENT_LONG_PRESSED, g_scr_running);
		g_home_btn_power = add_top_text_btn(top, "电源", 180); /* 固定中文：电源 */
		lv_obj_add_event_cb(g_home_btn_power, cb_power_long, LV_EVENT_LONG_PRESSED, g_scr_running);

		/* --- 轮播区：5 张程序卡片，每张含图片 + 程序名 label --- */
		lv_obj_t * mid = lv_obj_create(root);              /* 轮播区容器 */
		lv_obj_set_size(mid, LV_PCT(100), LV_PCT(60));                  //轮播区大小
		lv_obj_set_pos(mid, 0, LV_PCT(10));                             //轮播区位置
		lv_obj_set_style_bg_opa(mid, LV_OPA_TRANSP, LV_PART_MAIN);          //轮播区背景透明
		lv_obj_set_style_border_width(mid, 0, LV_PART_MAIN);            //轮播区边框宽度为0
		lv_obj_set_style_pad_all(mid, 0, LV_PART_MAIN);                 //轮播区内边距为0
		lv_obj_set_style_radius(mid, 0, LV_PART_MAIN);                  //轮播区圆角为0
		lv_obj_add_flag(mid, LV_OBJ_FLAG_OVERFLOW_VISIBLE);             //允许轮播区内容超出边界显示
		lv_obj_set_style_layout(mid, LV_LAYOUT_NONE, LV_PART_MAIN);     //轮播区布局为无布局
		lv_obj_set_scroll_dir(mid, LV_DIR_NONE);                        //轮播区滚动方向为无滚动
		lv_obj_set_scrollbar_mode(mid, LV_SCROLLBAR_MODE_OFF);          //轮播区滚动条模式为无滚动条
		g_mode_carousel = mid;                                          //将轮播区赋值给全局变量 g_mode_carousel
		lv_obj_add_event_cb(mid, carousel_size_cb, LV_EVENT_SIZE_CHANGED, NULL); //当轮播区尺寸变化时，调用 carousel_size_cb 函数

		/* 编码器轮播焦点代理：透明铺满轮播区，不参与触摸（事件不穿透需放卡片下层） */
		g_home_carousel_enc = lv_obj_create(mid);
		lv_obj_set_size(g_home_carousel_enc, LV_PCT(100), LV_PCT(100));
		lv_obj_set_style_bg_opa(g_home_carousel_enc, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(g_home_carousel_enc, 0, LV_PART_MAIN);
		lv_obj_remove_flag(g_home_carousel_enc, LV_OBJ_FLAG_SCROLLABLE);
		lv_obj_remove_flag(g_home_carousel_enc, LV_OBJ_FLAG_CLICKABLE);
		lv_obj_add_event_cb(g_home_carousel_enc, cb_home_carousel_encoder, LV_EVENT_ALL, NULL);
		lv_obj_move_background(g_home_carousel_enc);

		const uint32_t borders[] = { 0xFF8C9E, 0x6644AA, 0xFFFFFF, 0x44CCCC, 0xCC6644 };  //卡片边框颜色
		for(int i = 0; i < CAROUSEL_VISIBLE_SLOTS; i++) {
				lv_obj_t * card = lv_obj_create(mid);
				g_mode_cards[i] = card;
				lv_obj_set_size(card, s_mode_card_sz, s_mode_card_sz);                        //卡片大小
				lv_obj_set_style_radius(card, LV_RADIUS_CIRCLE, LV_PART_MAIN);                //卡片圆角为圆形
				lv_obj_set_style_bg_opa(card, LV_OPA_TRANSP, LV_PART_MAIN);                   //卡片背景透明
				lv_obj_set_style_border_width(card, 0, LV_PART_MAIN);                         //框的粗细
				lv_obj_set_style_border_color(card, lv_color_hex(borders[i % 5]), LV_PART_MAIN);  //框的颜色
				lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);                                                   //接收指针/触摸事件
				lv_obj_add_event_cb(card, wheel_pointer_cb, LV_EVENT_PRESSED, NULL);                            //卡片按下事件
				lv_obj_add_event_cb(card, wheel_pointer_cb, LV_EVENT_PRESSING, NULL);                           //卡片拖动事件
				lv_obj_add_event_cb(card, wheel_pointer_cb, LV_EVENT_RELEASED, NULL);                           //卡片释放事件

				lv_obj_t * img = lv_image_create(card);
				g_mode_card_imgs[i] = img;
				lv_obj_center(img);

				/* 轮播卡片程序名 label：文案由 carousel_update_card_images() 写入 ui_program_name_get() */
				lv_obj_t * lab = lv_label_create(card);    /* 程序名文字标签（中：大物/标准洗；英：Heavy/Standard…） */
				g_mode_card_labels[i] = lab;               /* 保存指针供切换语言时刷新 */
				lv_obj_set_style_text_color(lab, lv_color_hex(COL_TEXT), LV_PART_MAIN); /* 白色字 */
				lv_obj_set_style_text_align(lab, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);  /* 居中 */
				ui_set_obj_font(lab, s_font_sc_30);        /* 30 号中文字体（含英 glyph） */
				lv_obj_align(lab, LV_ALIGN_CENTER, 0, 75); /* 相对圆心向下 75px */
				lv_obj_add_flag(lab, LV_OBJ_FLAG_EVENT_BUBBLE); /* 点击事件冒泡到卡片 */
				lv_obj_move_foreground(lab);               /* 文字叠在图片之上 */
		}


		lv_obj_t * dots = lv_obj_create(root);        //指示条状态点区域，5 个程序各一点
		lv_obj_set_size(dots, LV_PCT(100), 30);     //指示条状态点区域大小
		lv_obj_set_pos(dots, 0, 402);               //dots位置
		lv_obj_set_style_bg_opa(dots, LV_OPA_TRANSP, LV_PART_MAIN);                        //指示条状态点区域背景透明
		lv_obj_set_style_border_width(dots, 0, LV_PART_MAIN);                              //指示条状态点区域边框宽度为0
		lv_obj_add_flag(dots, LV_OBJ_FLAG_OVERFLOW_VISIBLE);                               //指示条状态点区域内容超出边界显示
		lv_obj_set_style_layout(dots, LV_LAYOUT_NONE, LV_PART_MAIN);                       //指示条状态点区域布局为无布局
		lv_obj_set_scroll_dir(dots, LV_DIR_NONE);                                          //指示条状态点区域滚动方向为无滚动
		lv_obj_set_scrollbar_mode(dots, LV_SCROLLBAR_MODE_OFF);                            //指示条状态点区域滚动条模式为无滚动条
		lv_obj_remove_flag(dots, LV_OBJ_FLAG_SCROLLABLE);
		for(int i = 0; i < TOTAL_PROGRAMS; i++) {
				lv_obj_t * d = lv_obj_create(dots);
				g_mode_dots[i] = d;
				lv_coord_t ds = (i == g_wheel_sel) ? 13 : 10;
				lv_obj_set_size(d, ds, ds);
				lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, LV_PART_MAIN);
				lv_obj_set_style_bg_color(d, lv_color_hex(i == g_wheel_sel ? COL_TEXT : COL_DIM), LV_PART_MAIN);
				lv_obj_set_style_bg_opa(d, LV_OPA_COVER, LV_PART_MAIN);
				lv_obj_set_style_border_width(d, 0, LV_PART_MAIN);
				lv_obj_set_style_pad_all(d, 0, LV_PART_MAIN);
				lv_obj_remove_flag(d, LV_OBJ_FLAG_SCROLLABLE);
				lv_obj_set_scrollbar_mode(d, LV_SCROLLBAR_MODE_OFF);
				lv_obj_add_flag(d, LV_OBJ_FLAG_CLICKABLE);
				lv_obj_add_event_cb(d, cb_home_prog_dot_focus, LV_EVENT_ALL, (void *)(intptr_t)i);
		}

		lv_obj_update_layout(root);//更新布局，调用 carousel_wrap_relayout 以正确放置轮播区卡片和指示点
		carousel_wrap_relayout();//轮播区布局

		lv_obj_t * bottom_panel = lv_obj_create(root);  //商用洗底部栏区域
		lv_obj_set_size(bottom_panel, LV_PCT(100), LV_PCT(25));
		lv_obj_set_pos(bottom_panel, 0, LV_PCT(75));
		lv_obj_set_style_bg_opa(bottom_panel, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(bottom_panel, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(bottom_panel, 0, LV_PART_MAIN);
		lv_obj_set_style_layout(bottom_panel, LV_LAYOUT_NONE, LV_PART_MAIN);



		const lv_coord_t bar_col1_x = (lv_coord_t)(UI_FIXED_W * 10 / 100 - UI_FIXED_W / 2);
		const lv_coord_t bar_col2_x = (lv_coord_t)(UI_FIXED_W * 30 / 100 - UI_FIXED_W / 2);
		const lv_coord_t bar_col3_x = (lv_coord_t)(UI_FIXED_W * 50 / 100 - UI_FIXED_W / 2);
		const lv_coord_t bar_col4_x = (lv_coord_t)(UI_FIXED_W * 70 / 100 - UI_FIXED_W / 2);
		const lv_coord_t bar_col5_x = (lv_coord_t)(UI_FIXED_W * 90 / 100 - UI_FIXED_W / 2);

		lv_obj_t * lbl_row = lv_obj_create(bottom_panel);   //文字标签行（虚线上方）
		lv_obj_set_size(lbl_row, LV_PCT(100), 50);
		lv_obj_set_pos(lbl_row, 0, 0);
		lv_obj_set_style_bg_opa(lbl_row, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(lbl_row, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(lbl_row, 0, LV_PART_MAIN);
		lv_obj_set_style_layout(lbl_row, LV_LAYOUT_NONE, LV_PART_MAIN);

		/* 第 1 列文字：程序时间，如「20min」，由 home_sync_program_labels() 按选中程序填写 */
		lv_obj_t * lbl_time = lv_label_create(lbl_row);
		g_lbl_home_time = lbl_time;                            /* 全局：切换程序/语言后刷新 */
		lv_obj_set_style_text_color(lbl_time, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(lbl_time, s_font_sc_35);
		lv_obj_align(lbl_time, LV_ALIGN_CENTER, bar_col1_x, 0); /* 第 1 列水平居中 */

		/* 第 2 列文字：洗涤温度，如「30℃」「--」「COLD」 */
		lv_obj_t * lbl_temperature = lv_label_create(lbl_row);
		g_lbl_home_temp = lbl_temperature;
		lv_obj_set_style_text_color(lbl_temperature, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(lbl_temperature, s_font_sc_35);
		lv_obj_align(lbl_temperature, LV_ALIGN_CENTER, bar_col2_x, 0);

		/* 第 3 列文字：程序金额，如「¥8」「--」 */
		lv_obj_t * lbl_pay = lv_label_create(lbl_row);
		g_lbl_home_pay = lbl_pay;
		lv_obj_set_style_text_color(lbl_pay, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(lbl_pay, s_font_sc_35);
		lv_obj_align(lbl_pay, LV_ALIGN_CENTER, bar_col3_x, 0);
		home_sync_program_labels();                            /* 按 g_wheel_sel 写入时间/温度/价格 */

		/* 第 4 列文字：语言指示，中文界面「China」/ 英文界面「English」（STR_LANG_INDICATOR） */
		g_lbl_home_lang = lv_label_create(lbl_row);
		lv_obj_set_style_text_color(g_lbl_home_lang, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(g_lbl_home_lang, s_font_sc_35);
		lv_obj_align(g_lbl_home_lang, LV_ALIGN_CENTER, bar_col4_x, 0);
		ui_lang_bind_label(g_lbl_home_lang, STR_LANG_INDICATOR); /* 绑定 i18n，点击语言图标会刷新 */

		/* 第 5 列文字：管理员入口，固定英文「Login」，不参与语言表 */
		lv_obj_t * lbl_manager = lv_label_create(lbl_row);
		lv_label_set_text(lbl_manager, "Login");               /* 固定文案 */
		lv_obj_set_style_text_color(lbl_manager, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(lbl_manager, s_font_sc_35);
		lv_obj_align(lbl_manager, LV_ALIGN_CENTER, bar_col5_x, 0);
		lv_obj_clear_flag(lbl_manager, LV_OBJ_FLAG_CLICKABLE);

		const int sep_w_pct = 100;             /* 虚线宽度占 bottom_panel 百分比 */
		const lv_coord_t sep_stroke_w = 4;     /* 线粗 */
		const lv_coord_t sep_dash_len = 15;    /* 每段虚线长度 */
		const lv_coord_t sep_dash_gap = 15;    /* 段间空白长度 */
		const lv_coord_t sep_y = 60;           /* 相对 bottom_panel 顶部的 Y 偏移 */
		const lv_opa_t sep_opa = LV_OPA_COVER;
		const lv_coord_t sep_w = (lv_coord_t)(UI_FIXED_W * sep_w_pct / 100);
		static lv_point_precise_t sep_pts[2];

		sep_pts[0].x = 0;
		sep_pts[0].y = 0;
		sep_pts[1].x = sep_w;
		sep_pts[1].y = 0;

		lv_obj_t * sep_line = lv_line_create(bottom_panel);
		lv_obj_set_width(sep_line, sep_w);
		lv_obj_set_height(sep_line, sep_stroke_w + 4);
		lv_obj_align(sep_line, LV_ALIGN_TOP_MID, 0, sep_y);
		lv_obj_set_style_line_width(sep_line, sep_stroke_w, LV_PART_MAIN);
		lv_obj_set_style_line_dash_width(sep_line, sep_dash_len, LV_PART_MAIN);
		lv_obj_set_style_line_dash_gap(sep_line, sep_dash_gap, LV_PART_MAIN);
		lv_obj_set_style_line_color(sep_line, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		lv_obj_set_style_line_opa(sep_line, sep_opa, LV_PART_MAIN);
		lv_obj_set_style_line_rounded(sep_line, false, LV_PART_MAIN);
		lv_obj_set_style_bg_opa(sep_line, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(sep_line, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(sep_line, 0, LV_PART_MAIN);
		lv_obj_remove_flag(sep_line, LV_OBJ_FLAG_SCROLLABLE);
		lv_line_set_points_mutable(sep_line, sep_pts, 2);

		LV_IMAGE_DECLARE(comm_time);
		LV_IMAGE_DECLARE(temperature);
		LV_IMAGE_DECLARE(pay);
		LV_IMAGE_DECLARE(language);
		LV_IMAGE_DECLARE(admin);

		lv_obj_t * icon_row = lv_obj_create(bottom_panel);  //图标行（虚线下方）
		lv_obj_set_size(icon_row, LV_PCT(100), LV_SIZE_CONTENT);
		lv_obj_set_pos(icon_row, 0, 85);
		lv_obj_set_style_bg_opa(icon_row, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(icon_row, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(icon_row, 0, LV_PART_MAIN);
		lv_obj_set_style_layout(icon_row, LV_LAYOUT_NONE, LV_PART_MAIN);

		lv_obj_t * img_time = lv_image_create(icon_row);
		lv_image_set_src(img_time, &comm_time);
		lv_obj_align(img_time, LV_ALIGN_TOP_MID, bar_col1_x, 0);

		lv_obj_t * img_temperature = lv_image_create(icon_row);
		lv_image_set_src(img_temperature, &temperature);
		lv_obj_align(img_temperature, LV_ALIGN_TOP_MID, bar_col2_x, 0);

		lv_obj_t * img_pay = lv_image_create(icon_row);
		lv_image_set_src(img_pay, &pay);
		lv_obj_align(img_pay, LV_ALIGN_TOP_MID, bar_col3_x, 0);

		lv_obj_t * img_lang = lv_image_create(icon_row);
		lv_image_set_src(img_lang, &language);
		lv_obj_align(img_lang, LV_ALIGN_TOP_MID, bar_col4_x, 0);

		g_home_btn_admin = lv_imgbtn_create(icon_row);
		lv_imgbtn_set_src(g_home_btn_admin, LV_IMGBTN_STATE_RELEASED, NULL, &admin, NULL);
		lv_obj_align(g_home_btn_admin, LV_ALIGN_TOP_MID, bar_col5_x, 0);
		lv_obj_remove_flag(g_home_btn_admin, LV_OBJ_FLAG_SCROLLABLE);
		lv_obj_clear_flag(g_home_btn_admin, LV_OBJ_FLAG_CLICKABLE);

		{
			const lv_coord_t admin_hit_w = 280;
			lv_obj_t * admin_hit = lv_obj_create(bottom_panel);
			lv_obj_set_size(admin_hit, admin_hit_w, LV_PCT(100));
			lv_obj_align(admin_hit, LV_ALIGN_RIGHT_MID, 0, 0);
			lv_obj_set_style_bg_opa(admin_hit, LV_OPA_TRANSP, LV_PART_MAIN);
			lv_obj_set_style_border_width(admin_hit, 0, LV_PART_MAIN);
			lv_obj_set_style_pad_all(admin_hit, 0, LV_PART_MAIN);
			lv_obj_set_style_radius(admin_hit, 0, LV_PART_MAIN);
			lv_obj_remove_flag(admin_hit, LV_OBJ_FLAG_SCROLLABLE);
			lv_obj_add_flag(admin_hit, LV_OBJ_FLAG_CLICKABLE);
			lv_obj_add_event_cb(admin_hit, cb_load_admin, LV_EVENT_CLICKED, NULL);
			lv_obj_move_foreground(admin_hit);
		}

		home_encoder_group_build();
}








//获取指定程序索引的参数表项（时间/温度/价格）
static const ui_program_profile_t * program_profile_get(int32_t idx)  //获取指定程序索引的参数表项（时间/温度/价格）
{
	idx = wheel_mod_total(idx);  //程序索引取模
	return &g_program_profiles[(unsigned)idx];
}

/* 格式化为首页/程序设置时间标签（分钟模式为 Nmin，演示秒模式为 Ns） */
static void program_format_time_label(uint32_t sec, char * buf, size_t buf_sz)
{
#if PROG_ADMIN_DEMO_SEC
	snprintf(buf, buf_sz, "%us", (unsigned)sec);
#else
	snprintf(buf, buf_sz, "%umin", (unsigned)(sec / 60u));
#endif
}

//格式化为首页价格标签（¥N 或 --）
static void program_format_price_home(int32_t price, char * buf, size_t buf_sz)  //格式化为首页价格标签（¥N 或 --）
{
	if(price < 0) {
		snprintf(buf, buf_sz, "--");
	} else {
		snprintf(buf, buf_sz, "¥%d", price);
	}
}

//格式化为支付页价格标签（¥ N.00 或 --）
static void program_format_price_pay(int32_t price, char * buf, size_t buf_sz)  //格式化为支付页价格标签（¥ N.00 或 --）
{
	if(price < 0) {
		snprintf(buf, buf_sz, "--");
	} else {
		snprintf(buf, buf_sz, "¥ %d.00", price);
	}
}

/* 刷新主页底部第 1～3 列文字：时间、温度、金额（随程序变，不随语言表） */
static void home_sync_program_labels(void)
{
	const ui_program_profile_t * profile = program_profile_get(g_wheel_sel); /* 当前程序参数 */
	char buf[16];                                        /* 格式化缓冲区 */

	if(g_lbl_home_time != NULL) {                        /* 第1列：程序总运行时间 label */
		program_format_time_label(profile->wash_sec, buf, sizeof(buf)); /* 如 20min */
		lv_label_set_text(g_lbl_home_time, buf);
	}
	if(g_lbl_home_temp != NULL) {                        /* 第2列：温度 label */
		lv_label_set_text(g_lbl_home_temp, profile->temp); /* 如 30℃ / COLD */
	}
	if(g_lbl_home_pay != NULL) {                         /* 第3列：价格 label */
		program_format_price_home(profile->price, buf, sizeof(buf)); /* 如 ¥8 */
		lv_label_set_text(g_lbl_home_pay, buf);
	}
}

/* 刷新支付页金额 label，如「¥ 8.00」 */
static void pay_sync_price_label(void)
{
	int32_t price = g_session_active ? g_session_total_price : program_profile_get(g_wheel_sel)->price;
	char buf[16];
	program_format_price_pay(price, buf, sizeof(buf));
	if(g_lbl_pay_price != NULL) lv_label_set_text(g_lbl_pay_price, buf);
	if(g_lbl_pay_price_alipay != NULL) lv_label_set_text(g_lbl_pay_price_alipay, buf);
	if(g_lbl_pay_price_wechat != NULL) lv_label_set_text(g_lbl_pay_price_wechat, buf);
}

//按支付方式与当前程序刷新支付页二维码与提示布局
static void pay_sync_pay_ui(void)
{
	if(g_pay_qr_img == NULL) return;
	bool alipay = ui_payment_alipay_get();
	bool wechat = ui_payment_wechat_get();
	/* 两者皆取消时，支付页兜底只显示支付宝（不修改 g_ui_payment_*） */
	if(!alipay && !wechat) alipay = true;
	int32_t idx = wheel_mod_total(g_wheel_sel);

	if(alipay && wechat) {
		/* 双码：左支付宝、右微信 */
		if(g_pay_dual_row != NULL) lv_obj_remove_flag(g_pay_dual_row, LV_OBJ_FLAG_HIDDEN);
		if(g_pay_single_col != NULL) lv_obj_add_flag(g_pay_single_col, LV_OBJ_FLAG_HIDDEN);
		if(g_pay_qr_alipay != NULL) lv_image_set_src(g_pay_qr_alipay, g_alipay_qr_imgs[(unsigned)idx]);
		if(g_pay_qr_wechat != NULL) lv_image_set_src(g_pay_qr_wechat, g_wechat_qr_imgs[(unsigned)idx]);
	}
	else {
		/* 单码：按唯一启用的方式选图与提示（含两者皆关时的支付宝兜底） */
		if(g_pay_dual_row != NULL) lv_obj_add_flag(g_pay_dual_row, LV_OBJ_FLAG_HIDDEN);
		if(g_pay_single_col != NULL) lv_obj_remove_flag(g_pay_single_col, LV_OBJ_FLAG_HIDDEN);
		lv_image_set_src(g_pay_qr_img, alipay ? g_alipay_qr_imgs[(unsigned)idx] : g_wechat_qr_imgs[(unsigned)idx]);
		if(g_lbl_pay_hint != NULL) {
			lv_label_set_text(g_lbl_pay_hint, ui_translation(alipay ? STR_PAY_HINT_ALIPAY : STR_PAY_HINT_WECHAT));
		}
	}
}

/* 运行页程序名 label 与当前选中程序 g_wheel_sel 一致（随语言切换） */
static void running_screen_sync_mode_name(void)
{
	if(g_running_mode_label == NULL) return;           /* 运行页未创建则跳过 */
	int32_t idx = wheel_mod_total(g_wheel_sel);      /* 当前程序索引 0..4 */
	lv_label_set_text(g_running_mode_label, ui_program_name_get(idx)); /* 写入中/英程序名 */
	lv_obj_invalidate(g_running_mode_label);         /* 请求重绘 */
	if(g_running_mid != NULL) {
		lv_obj_update_layout(g_running_mid);           /* 程序名变长/变短后 mid 框宽度会变 */
	}
	running_child_lock_align_btn();                  /* 童锁跟随 mid 右缘重对齐 */
}

/* 当前程序运行总秒数（各段之和） */
static uint32_t running_program_total_sec(int32_t idx)
{
	idx = wheel_mod_total(idx);
	const ui_program_run_stages_t * st = &g_program_run_stages[(unsigned)idx];
	return st->wash_sec + st->rinse_sec + st->spin_sec;
}

/* 根据已运行时间与分段配置，决定左侧当前阶段文案 ID（烘干中 / 打冷风中） */
static ui_str_id_t running_status_id_for_elapsed(const ui_program_run_stages_t * st, uint32_t elapsed)
{
	if(st->wash_sec > 0u && elapsed < st->wash_sec) {
		return STR_RUN_WASHING;
	}
	if(st->spin_sec > 0u) {
		return STR_RUN_SPINNING;
	}
	return STR_RUN_WASHING;
}

/* 运行页分段配置：有会话时用追加页确定后的分段，否则用程序基础表 */
static const ui_program_run_stages_t * running_program_stages_get(int32_t idx)
{
	if(g_session_active) return &g_session_run_stages;
	idx = wheel_mod_total(idx);
	return &g_program_run_stages[(unsigned)idx];
}

/* 运行页底部：左=当前阶段（烘干中/打冷风中），右=阶段条（烘干/打冷风） */
static void running_status_sync_labels(void)
{
	if(g_kuaixi_run_lbl_status == NULL && g_kuaixi_run_lbl_stages == NULL) return;

	int32_t idx = wheel_mod_total(g_wheel_sel);
	const ui_program_run_stages_t * st = running_program_stages_get(idx);
	uint32_t total = st->wash_sec + st->rinse_sec + st->spin_sec;
	uint32_t elapsed = 0u;

	if(total > 0u && g_running_remain_sec <= total) {
		elapsed = total - g_running_remain_sec;
	}

	ui_str_id_t status_id = running_status_id_for_elapsed(st, elapsed);

	if(g_kuaixi_run_lbl_status != NULL) {
		lv_label_set_text(g_kuaixi_run_lbl_status, ui_translation(status_id));
	}
	if(g_kuaixi_run_lbl_stages != NULL) {
		lv_label_set_text(g_kuaixi_run_lbl_stages, ui_translation(st->stages_bar_id));
	}
}

//将剩余秒数格式化为 0:MM（向上取整到分钟）
static void running_countdown_format(uint32_t sec, char * buf, size_t buf_sz)
{
	uint32_t min = (sec == 0u) ? 0u : (sec + 59u) / 60u;
	snprintf(buf, buf_sz, "0:%02u", (unsigned)min);
}

//刷新运行页倒计时标签显示
static void running_countdown_update_label(void)  //更新倒计时标签
{
	if(g_running_time_label == NULL) return;
	char buf[12];
	running_countdown_format(g_running_remain_sec, buf, sizeof(buf));
	lv_label_set_text(g_running_time_label, buf);
	running_status_sync_labels();                      /* 剩余时间变化 → 切换烘干/打冷风文案 */
	if(g_running_mid != NULL) {
		lv_obj_update_layout(g_running_mid);           /* 倒计时位数变化时 mid 宽度可能变化 */
	}
	running_child_lock_align_btn();                  /* 童锁跟随 mid 右缘重对齐 */
}

//停止支付完成页 2 秒自动跳转定时器
static void pay_done_timer_stop(void)  //停止支付完成页 2 秒自动跳转定时器
{
	if(g_pay_done_timer != NULL) {
		lv_timer_delete(g_pay_done_timer);
		g_pay_done_timer = NULL;
	}
}

//支付完成页定时器回调：2 秒后进入运行页
static void cb_pay_done_timer(lv_timer_t * t)  //支付完成页定时器回调
{
	(void)t;
	SETFLAG(FSM_FLAG_PAY_DONE_COMPLETE);
	pay_done_timer_stop();  //停止支付完成页 2 秒自动跳转定时器
	fsm_state_change(FSM_RUNNING);
	ui_screen_load(g_scr_running);  //加载目标屏幕
}

//启动支付完成页 2 秒自动跳转定时器
static void pay_done_timer_start(void)  //启动支付完成页 2 秒自动跳转定时器
{
	pay_done_timer_stop();  //停止支付完成页 2 秒自动跳转定时器
	g_pay_done_timer = lv_timer_create(cb_pay_done_timer, 2000, NULL);
	lv_timer_set_repeat_count(g_pay_done_timer, 1);
}

//停止暂停时倒计时闪烁定时器并恢复标签可见
static void running_blink_stop(void)  //停止暂停时倒计时闪烁定时器并恢复标签可见
{
	if(g_running_blink_timer != NULL) {
		lv_timer_delete(g_running_blink_timer);
		g_running_blink_timer = NULL;
	}
	g_running_blink_visible = true;
	if(g_running_time_label != NULL) {
		lv_obj_set_style_opa(g_running_time_label, LV_OPA_COVER, LV_PART_MAIN);
	}
}

//暂停时倒计时标签 500ms 闪烁定时器回调
static void cb_running_blink(lv_timer_t * t)  //暂停时倒计时标签 500ms 闪烁定时器回调
{
	(void)t;
	if(g_running_time_label == NULL) return;

	g_running_blink_visible = !g_running_blink_visible;
	lv_obj_set_style_opa(g_running_time_label,
		g_running_blink_visible ? LV_OPA_COVER : LV_OPA_TRANSP, LV_PART_MAIN);
}

//停止运行页倒计时定时器
static void running_countdown_stop(void)  //停止运行页倒计时定时器
{
	if(g_running_countdown_timer != NULL) {
		lv_timer_delete(g_running_countdown_timer);
		g_running_countdown_timer = NULL;
	}
}

//重置倒计时状态：清除暂停、闪烁与定时器
static void running_countdown_reset_all(void)  //重置倒计时状态
{
	g_running_countdown_paused = false;
	running_blink_stop();  //停止暂停时倒计时闪烁定时器并恢复标签可见
	running_countdown_stop();  //停止运行页倒计时定时器
}

//暂停倒计时并启动时间标签闪烁
static void running_countdown_pause(void)  //暂停倒计时并启动时间标签闪烁
{
	if(g_running_countdown_paused) return;
	if(g_running_countdown_timer == NULL) return;

	g_running_countdown_paused = true;
	running_countdown_stop();  //停止运行页倒计时定时器
	running_blink_stop();  //停止暂停时倒计时闪烁定时器并恢复标签可见
	g_running_blink_visible = true;
	if(g_running_time_label != NULL) {
		lv_obj_set_style_opa(g_running_time_label, LV_OPA_COVER, LV_PART_MAIN);
	}
	g_running_blink_timer = lv_timer_create(cb_running_blink, 500, NULL);
}

//恢复倒计时并停止闪烁
static void running_countdown_resume(void)  //恢复倒计时并停止闪烁
{
	if(!g_running_countdown_paused) return;

	g_running_countdown_paused = false;
	running_blink_stop();  //停止暂停时倒计时闪烁定时器并恢复标签可见
	running_countdown_arm();  //启动倒计时定时器
}

/* 根据 FSM 状态同步运行页倒计时（在运行页且非童锁时） */

static void ui_fsm_poll_running_pause_sync(void)
{
	if(lv_scr_act() != g_scr_running) return;
	if(g_ui_child_lock) return;

	if(fsm.state == FSM_PAUSED) {
		running_countdown_pause();
	} else if(fsm.state == FSM_RUNNING) {
		running_countdown_resume();
	}
}

/* 运行页启停：RUNNING<->PAUSED */
static void ui_fsm_runpause_apply_running_page(void)
{
	if(g_ui_child_lock) return;
	if(g_cycle_active) {
		cycle_abort_run(); // 5.2.1：循环模式运行中启停 = 中断，非暂停
		return;
	}

	if(fsm.state == FSM_RUNNING) {
		fsm_state_change(FSM_PAUSED);
		running_countdown_pause();
	} else if(fsm.state == FSM_PAUSED) {
		fsm_state_change(FSM_RUNNING);
		running_countdown_resume();
	}

	ui_fsm_poll_running_pause_sync();
}

//运行页启停按钮：写 FSM 暂停/继续，并同步 UI 倒计时
static void cb_running_runpause(lv_event_t * e)  //运行页启停按钮
{
	if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
	if(lv_scr_act() != g_scr_running) return;
	ui_fsm_runpause_apply_running_page();
}

#define RUNNING_COUNTDOWN_PERIOD_MS  60000u  /* 运行页倒计时：每分钟递减 */

//运行页每分钟倒计时回调，归零后跳转结束页
static void cb_running_countdown(lv_timer_t * t)
{
	(void)t;
	if(g_running_remain_sec == 0u) return;

	if(g_running_remain_sec <= 60u) {
		g_running_remain_sec = 0u;
	} else {
		g_running_remain_sec -= 60u;
	}
	running_countdown_update_label();

	if(g_running_remain_sec == 0u) {
		running_countdown_reset_all();
		/* 5.2.1：寿命试验完成 → 自动下一轮，不进结束页 */
		if(g_cycle_active && g_cycle_ui_state == CYCLE_UI_RUNNING) {
			cycle_finish_current_run();
			return;
		}
		ui_send_beep_seq(7);
		ui_screen_load(g_scr_end);
	}
}

//若剩余时间>0 则启动分钟倒计时定时器
static void running_countdown_arm(void)
{
	if(g_running_countdown_timer != NULL) return;
	if(g_running_remain_sec == 0u) return;
	g_running_countdown_timer = lv_timer_create(cb_running_countdown, RUNNING_COUNTDOWN_PERIOD_MS, NULL);
}

/* 运行页实时温度：同步到 UI */
static void running_live_params_sync(void)
{
	char buf[32];
	if(g_running_lbl_temp_live != NULL) {
		lv_snprintf(buf, sizeof(buf), "温度：%d ℃", (int)g_running_live_temp_c);
		lv_label_set_text(g_running_lbl_temp_live, buf);
	}
}

/* 实机同步运行时温度 */
void ui_running_live_temp_set(int16_t temp_c)
{
	g_running_live_temp_c = temp_c;
	running_live_params_sync();
}

//进入运行页时按程序时长初始化并开始倒计时
static void running_countdown_start(void)
{
	running_countdown_reset_all();
	g_running_remain_sec = g_session_active ? g_session_total_sec : running_program_total_sec(g_wheel_sel);
	if(g_running_remain_sec == 0u) {
		g_running_remain_sec = 60u;
	}
#if UI_RUNNING_LIVE_PC_SIM
	g_running_live_temp_c = 25;
#endif
	running_live_params_sync();
	running_countdown_update_label();
	running_countdown_arm();
}

// 创建循环模式故障叠层（设定页/运行页共用样式）
static void cycle_fault_overlay_create(lv_obj_t * parent, lv_obj_t ** panel_out, lv_obj_t ** lbl_out)
{
	lv_obj_t * panel = lv_obj_create(parent);
	lv_obj_set_size(panel, LV_PCT(100), LV_PCT(100));
	lv_obj_set_pos(panel, 0, 0);
	lv_obj_set_style_bg_opa(panel, LV_OPA_80, LV_PART_MAIN);
	lv_obj_set_style_bg_color(panel, lv_color_hex(0x000000), LV_PART_MAIN);
	lv_obj_set_style_border_width(panel, 0, LV_PART_MAIN);
	lv_obj_remove_flag(panel, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(panel, LV_OBJ_FLAG_HIDDEN);

	lv_obj_t * lbl = lv_label_create(panel);
	lv_obj_set_style_text_color(lbl, lv_color_hex(COL_ORANGE), LV_PART_MAIN);
	ui_set_obj_font(lbl, s_font_sc_125);
	lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_flag(lbl, LV_OBJ_FLAG_HIDDEN);

	*panel_out = panel;
	*lbl_out = lbl;
}

/* 构建运行页：背景、顶部栏、程序名/倒计时、童锁、底部状态文字（支持中/英） */
static void build_running(void)
{
		lv_obj_t * root = lv_obj_create(g_scr_running);    /* 运行页全屏根容器 */
		lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
		lv_obj_set_pos(root, 0, 0);
		lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(root, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(root, 0, LV_PART_MAIN);
		lv_obj_set_style_radius(root, 0, LV_PART_MAIN);
		lv_obj_set_style_layout(root, LV_LAYOUT_NONE, LV_PART_MAIN);

		LV_IMAGE_DECLARE(yunxingbackground);                 /* 运行页背景图资源 */
		lv_obj_t * bg = lv_image_create(root);
		lv_image_set_src(bg, &yunxingbackground);
		lv_obj_set_size(bg, (lv_coord_t)UI_FIXED_W, (lv_coord_t)UI_FIXED_H);
		lv_obj_align(bg, LV_ALIGN_CENTER, 0, 0);
		lv_image_set_inner_align(bg, LV_IMAGE_ALIGN_STRETCH);
		lv_obj_move_background(bg);                        /* 背景置底 */

		/* 顶部栏：返回、时钟；「启停」「电源」固定中文 */
		lv_obj_t * top = create_top_bar(root, &g_lbl_clock_running, g_scr_home, &g_running_btn_back);
		g_running_btn_runpause = add_top_text_btn(top, "启停", 100);
		lv_obj_add_event_cb(g_running_btn_runpause, cb_running_runpause, LV_EVENT_CLICKED, NULL);
		//lv_obj_add_event_cb(g_running_btn_runpause, cb_runpause, LV_EVENT_CLICKED, NULL);
		lv_obj_add_event_cb(g_running_btn_runpause, cb_runpause_long, LV_EVENT_LONG_PRESSED, NULL);
		g_running_btn_power = add_top_text_btn(top, "电源", 180);
		lv_obj_add_event_cb(g_running_btn_power, cb_power_long, LV_EVENT_LONG_PRESSED, g_scr_off);

		/* 中间横排：程序名 + 倒计时 */
		lv_obj_t * mid = lv_obj_create(root);
		lv_obj_set_size(mid, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
		lv_obj_align(mid, LV_ALIGN_CENTER, 0, -40);
		lv_obj_set_style_bg_opa(mid, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(mid, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(mid, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_column(mid, 24, LV_PART_MAIN);
		lv_obj_set_flex_flow(mid, LV_FLEX_FLOW_ROW);
		lv_obj_set_flex_align(mid, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
		g_running_mid = mid;
		lv_obj_add_event_cb(mid, cb_running_mid_layout_changed, LV_EVENT_SIZE_CHANGED, NULL); /* mid 框变化时童锁重对齐 */

		/* 程序名 label：中/英随 ui_program_name_get；非 i18n 绑定表，由 running_screen_sync_mode_name 刷新 */
		lv_obj_t * mode_txt = lv_label_create(mid);
		g_running_mode_label = mode_txt;
		ui_set_obj_font(mode_txt, s_font_sc_50);
		lv_obj_set_style_text_color(mode_txt, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		running_screen_sync_mode_name();                   /* 写入当前程序名（如 标准洗 / Standard） */

		/* 倒计时 label：0:MM，由 running_countdown_update_label 更新，不参与语言表 */
		lv_obj_t * time_txt = lv_label_create(mid);
		g_running_time_label = time_txt;
		lv_label_set_text(time_txt, "0:24");                 /* 占位初值，进入运行页后按程序时长重设 */
		lv_obj_set_style_text_color(time_txt, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(time_txt, s_font_sc_125);

		lv_obj_t * child_lock = lv_button_create(root);
		g_running_child_lock_btn = child_lock;
		lv_obj_set_size(child_lock, 49, 49);
		lv_obj_set_style_radius(child_lock, LV_RADIUS_CIRCLE, LV_PART_MAIN);
		lv_obj_set_style_pad_all(child_lock, 0, LV_PART_MAIN);
		lv_obj_set_style_shadow_width(child_lock, 0, LV_PART_MAIN);
		lv_obj_set_style_bg_opa(child_lock, LV_OPA_COVER, LV_PART_MAIN);
		lv_obj_set_style_bg_color(child_lock, lv_color_hex(0x555555), LV_PART_MAIN);
		lv_obj_set_style_border_width(child_lock, 0, LV_PART_MAIN);
		lv_obj_remove_flag(child_lock, LV_OBJ_FLAG_SCROLLABLE);
		lv_obj_add_event_cb(child_lock, cb_running_child_lock_long, LV_EVENT_LONG_PRESSED, NULL);
		lv_obj_add_event_cb(child_lock, cb_running_child_lock_released, LV_EVENT_RELEASED, NULL);

		lv_obj_t * lock_img = lv_image_create(child_lock);
		g_running_child_lock_img = lock_img;
		lv_image_set_src(lock_img, &child_lock_logo);
		lv_obj_set_style_image_opa(lock_img, LV_OPA_40, LV_PART_MAIN);
		lv_obj_center(lock_img);

		/* 左侧：实时温度（与商用洗同位置/字号；无水位频率） */
		{
			lv_obj_t * left = lv_obj_create(root);
			lv_obj_set_size(left, 280, LV_SIZE_CONTENT);
			lv_obj_align(left, LV_ALIGN_TOP_LEFT, 60, 110);
			lv_obj_set_style_bg_opa(left, LV_OPA_TRANSP, LV_PART_MAIN);
			lv_obj_set_style_border_width(left, 0, LV_PART_MAIN);
			lv_obj_set_style_pad_all(left, 0, LV_PART_MAIN);
			g_running_lbl_temp_live = lv_label_create(left);
			lv_obj_set_style_text_color(g_running_lbl_temp_live, lv_color_hex(COL_TEXT), LV_PART_MAIN);
			ui_set_obj_font(g_running_lbl_temp_live, s_font_sc_30);
			running_live_params_sync();
		}

		/* 底部栏：左侧状态 + 右侧阶段条 */
		lv_obj_t * bot = lv_obj_create(root);
		lv_obj_set_size(bot, LV_PCT(100), LV_SIZE_CONTENT);
		lv_obj_align(bot, LV_ALIGN_BOTTOM_MID, 0, -60);
		lv_obj_set_style_bg_opa(bot, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(bot, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(bot, 0, LV_PART_MAIN);
		lv_obj_set_style_radius(bot, 0, LV_PART_MAIN);
		lv_obj_set_style_layout(bot, LV_LAYOUT_NONE, LV_PART_MAIN);

		/* 左下：当前阶段（烘干中/打冷风中），由 running_status_sync_labels 按剩余时间刷新 */
		lv_obj_t * st = lv_label_create(bot);
		lv_obj_set_style_text_color(st, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(st, s_font_sc_35);
		lv_obj_align(st, LV_ALIGN_LEFT_MID, 60, 0);
		g_kuaixi_run_lbl_status = st;

		/* 右下：程序阶段条（烘干/打冷风），随程序固定 */
		lv_obj_t * stages = lv_label_create(bot);
		lv_obj_set_style_text_color(stages, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(stages, s_font_sc_35);
		lv_obj_align(stages, LV_ALIGN_RIGHT_MID, -60, 0);
		g_kuaixi_run_lbl_stages = stages;
		running_status_sync_labels();

		g_running_lock_blocker = lv_obj_create(root);
		lv_obj_set_size(g_running_lock_blocker, LV_PCT(100), LV_PCT(100));
		lv_obj_set_pos(g_running_lock_blocker, 0, 0);
		lv_obj_set_style_bg_opa(g_running_lock_blocker, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_bg_color(g_running_lock_blocker, lv_color_hex(0x000000), LV_PART_MAIN);
		lv_obj_set_style_border_width(g_running_lock_blocker, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(g_running_lock_blocker, 0, LV_PART_MAIN);
		lv_obj_remove_flag(g_running_lock_blocker, LV_OBJ_FLAG_SCROLLABLE);
		lv_obj_add_flag(g_running_lock_blocker, LV_OBJ_FLAG_CLICKABLE);
		lv_obj_add_flag(g_running_lock_blocker, LV_OBJ_FLAG_HIDDEN);

		/* 5.2.1：运行中故障时在运行页叠层闪烁，不切回设定页 */
		cycle_fault_overlay_create(root, &g_cycle_panel_fault_run, &g_cycle_lbl_fault_alt_run);

		running_child_lock_align_btn();  //将童锁按钮对齐到运行页中间栏右侧
		lv_obj_move_foreground(g_cycle_panel_fault_run);
		lv_obj_move_foreground(g_running_lock_blocker);
		lv_obj_move_foreground(g_running_child_lock_btn);
}

/* 构建洗涤结束页：图标 + 标题/提示文字（支持中/英） */
static void build_end(void)
{
		lv_obj_t * root = lv_obj_create(g_scr_end);        /* 结束页根容器 */
		lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
		lv_obj_set_pos(root, 0, 0);
		lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(root, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(root, 0, LV_PART_MAIN);
		lv_obj_set_style_radius(root, 0, LV_PART_MAIN);
		lv_obj_set_style_layout(root, LV_LAYOUT_NONE, LV_PART_MAIN);

		lv_obj_t * top = create_top_bar(root, &g_lbl_clock_end, g_scr_home, NULL);
		lv_obj_t * btn_runpause = add_top_text_btn(top, "启停", 100);
		lv_obj_add_event_cb(btn_runpause, cb_load_screen, LV_EVENT_CLICKED, g_scr_home);
		lv_obj_add_event_cb(btn_runpause, cb_runpause, LV_EVENT_CLICKED, g_scr_home);
		lv_obj_add_event_cb(btn_runpause, cb_runpause_long, LV_EVENT_LONG_PRESSED, g_scr_home);
		lv_obj_t * btn_power = add_top_text_btn(top, "电源", 180);
		lv_obj_add_event_cb(btn_power, cb_load_screen, LV_EVENT_CLICKED, g_scr_home);
		lv_obj_add_event_cb(btn_power, cb_power_long, LV_EVENT_LONG_PRESSED, NULL);

		lv_obj_t * center = lv_obj_create(root);           /* 垂直居中内容区 */
		lv_obj_set_size(center, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
		lv_obj_align(center, LV_ALIGN_CENTER, 0, 0);
		lv_obj_set_style_bg_opa(center, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(center, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(center, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_row(center, 16, LV_PART_MAIN);
		lv_obj_set_flex_flow(center, LV_FLEX_FLOW_COLUMN);
		lv_obj_set_flex_align(center, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

		LV_IMAGE_DECLARE(success);                         /* 结束图标 */
		lv_obj_t * img_end = lv_image_create(center);
		lv_image_set_src(img_end, &success);

		/* 主标题：中「烘干完成」/ 英「Drying Complete」（STR_END_TITLE） */
		g_lbl_end_title = lv_label_create(center);
		lv_obj_set_style_text_color(g_lbl_end_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(g_lbl_end_title, s_font_sc_50);
		ui_lang_bind_label(g_lbl_end_title, STR_END_TITLE);

		/* 副提示：中「请及时取衣」/ 英「Please take clothes promptly」（STR_END_HINT） */
		g_lbl_end_hint = lv_label_create(center);
		lv_obj_set_style_text_color(g_lbl_end_hint, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(g_lbl_end_hint, s_font_sc_50);
		ui_lang_bind_label(g_lbl_end_hint, STR_END_HINT);
}

/* ========== 有水自检（厂商维护）UI ==========
 *
 * 筒温显示（S-2/S-3 顶行右侧）：外部模块在自家 .c 顶部 extern 声明即可链接：
 *   extern void ui_selfcheck_live_temp_set(int16_t temp_c);
 *   extern int16_t ui_selfcheck_live_temp_get(void);
 * 用法示例：
 *   ui_selfcheck_live_temp_set(68);  // MCU 读到筒温后更新，标签自动刷新为「68 ℃」
 * MCU 读数后调用 ui_selfcheck_live_temp_set() 刷新标签；勿在 ISR 内直接调 LVGL。
 */

static void selfcheck_temp_label_sync(void)
{
	if(g_selfcheck_lbl_temp_live == NULL) return;
	char buf[16];
	lv_snprintf(buf, sizeof(buf), "%d ℃", (int)g_selfcheck_live_temp_c);
	lv_label_set_text(g_selfcheck_lbl_temp_live, buf);
}

int16_t ui_selfcheck_live_temp_get(void)
{
	return g_selfcheck_live_temp_c;
}

void ui_selfcheck_live_temp_set(int16_t temp_c)
{
	g_selfcheck_live_temp_c = temp_c;
	selfcheck_temp_label_sync();
}

static void selfcheck_timer_stop_all(void)
{
	if(g_selfcheck_timer != NULL) {
		lv_timer_delete(g_selfcheck_timer);
		g_selfcheck_timer = NULL;
	}
	selfcheck_indicator_blink_stop();
}

static void selfcheck_indicator_blink_stop(void)
{
	if(g_selfcheck_blink_timer != NULL) {
		lv_timer_delete(g_selfcheck_blink_timer);
		g_selfcheck_blink_timer = NULL;
	}
	g_selfcheck_blink_target = NULL;
	g_selfcheck_blink_on = true;
	if(g_selfcheck_lbl_dry != NULL) {
		lv_obj_set_style_text_opa(g_selfcheck_lbl_dry, LV_OPA_COVER, LV_PART_MAIN);
	}
	if(g_selfcheck_lbl_cooling != NULL) {
		lv_obj_set_style_text_opa(g_selfcheck_lbl_cooling, LV_OPA_COVER, LV_PART_MAIN);
	}
}

static void cb_selfcheck_indicator_blink(lv_timer_t * t)
{
	(void)t;
	if(g_selfcheck_blink_target == NULL) return;
	g_selfcheck_blink_on = !g_selfcheck_blink_on;
	lv_obj_set_style_text_opa(g_selfcheck_blink_target,
		g_selfcheck_blink_on ? LV_OPA_COVER : LV_OPA_TRANSP, LV_PART_MAIN);
}

static void selfcheck_indicator_blink_start(lv_obj_t * target)
{
	selfcheck_indicator_blink_stop();
	if(target == NULL) return;
	g_selfcheck_blink_target = target;
	g_selfcheck_blink_on = true;
	lv_obj_set_style_text_opa(target, LV_OPA_COVER, LV_PART_MAIN);
	g_selfcheck_blink_timer = lv_timer_create(cb_selfcheck_indicator_blink, SELFCHECK_BLINK_MS, NULL);
}

static void selfcheck_doorlock_set(lv_obj_t * lbl, bool is_open)
{
	if(lbl == NULL) return;
	lv_label_set_text(lbl, is_open ? "门锁开" : "门锁关");
}

static void selfcheck_apply_step_indicators(selfcheck_ui_state_t step)
{
	selfcheck_indicator_blink_stop();

	if(g_selfcheck_lbl_doorlock != NULL) {
		lv_obj_add_flag(g_selfcheck_lbl_doorlock, LV_OBJ_FLAG_HIDDEN);
	}
	if(g_selfcheck_row_status != NULL) {
		lv_obj_add_flag(g_selfcheck_row_status, LV_OBJ_FLAG_HIDDEN);
	}
	if(g_selfcheck_row_indicator != NULL) {
		lv_obj_add_flag(g_selfcheck_row_indicator, LV_OBJ_FLAG_HIDDEN);
	}
	if(g_selfcheck_lbl_temp_live != NULL) {
		lv_obj_add_flag(g_selfcheck_lbl_temp_live, LV_OBJ_FLAG_HIDDEN);
	}
	if(g_selfcheck_lbl_dry != NULL) {
		lv_obj_add_flag(g_selfcheck_lbl_dry, LV_OBJ_FLAG_HIDDEN);
	}
	if(g_selfcheck_lbl_cooling != NULL) {
		lv_obj_add_flag(g_selfcheck_lbl_cooling, LV_OBJ_FLAG_HIDDEN);
	}

	if(step == SELFCHECK_UI_MODEL || step == SELFCHECK_UI_FLASH || step == SELFCHECK_UI_INIT) {
		if(g_selfcheck_lbl_doorlock != NULL) {
			selfcheck_doorlock_set(g_selfcheck_lbl_doorlock, true);
			lv_obj_remove_flag(g_selfcheck_lbl_doorlock, LV_OBJ_FLAG_HIDDEN);
			lv_obj_move_foreground(g_selfcheck_lbl_doorlock);
		}
		return;
	}

	if(step != SELFCHECK_UI_S1 && step != SELFCHECK_UI_S2 && step != SELFCHECK_UI_S3) {
		return;
	}

	if(g_selfcheck_lbl_doorlock != NULL) {
		lv_obj_remove_flag(g_selfcheck_lbl_doorlock, LV_OBJ_FLAG_HIDDEN);
		lv_obj_move_foreground(g_selfcheck_lbl_doorlock);
		/* S-1：门锁上电前显示「开」；S-2/S-3：上电后显示「关」。量产由 MCU 回调切换。 */
		selfcheck_doorlock_set(g_selfcheck_lbl_doorlock, step == SELFCHECK_UI_S1);
	}

	if(step == SELFCHECK_UI_S2 || step == SELFCHECK_UI_S3) {
		if(g_selfcheck_row_status != NULL) {
			lv_obj_remove_flag(g_selfcheck_row_status, LV_OBJ_FLAG_HIDDEN);
		}
	}

	if(step == SELFCHECK_UI_S2 || step == SELFCHECK_UI_S3) {
		if(g_selfcheck_lbl_temp_live != NULL) {
			selfcheck_temp_label_sync();
			lv_obj_remove_flag(g_selfcheck_lbl_temp_live, LV_OBJ_FLAG_HIDDEN);
		}
		if(g_selfcheck_row_indicator != NULL) {
			lv_obj_remove_flag(g_selfcheck_row_indicator, LV_OBJ_FLAG_HIDDEN);
		}
	}

	if(step == SELFCHECK_UI_S2 && g_selfcheck_lbl_dry != NULL) {
		lv_obj_remove_flag(g_selfcheck_lbl_dry, LV_OBJ_FLAG_HIDDEN);
		selfcheck_indicator_blink_start(g_selfcheck_lbl_dry);
	}
	else if(step == SELFCHECK_UI_S3 && g_selfcheck_lbl_cooling != NULL) {
		lv_obj_remove_flag(g_selfcheck_lbl_cooling, LV_OBJ_FLAG_HIDDEN);
		selfcheck_indicator_blink_start(g_selfcheck_lbl_cooling);
	}
}

static void selfcheck_panel_hide_all(void)
{
	if(g_selfcheck_panel_model != NULL) {
		lv_obj_add_flag(g_selfcheck_panel_model, LV_OBJ_FLAG_HIDDEN);
	}
	if(g_selfcheck_panel_init != NULL) {
		lv_obj_add_flag(g_selfcheck_panel_init, LV_OBJ_FLAG_HIDDEN);
	}
	if(g_selfcheck_panel_steps != NULL) {
		lv_obj_add_flag(g_selfcheck_panel_steps, LV_OBJ_FLAG_HIDDEN);
	}
	if(g_selfcheck_panel_end != NULL) {
		lv_obj_add_flag(g_selfcheck_panel_end, LV_OBJ_FLAG_HIDDEN);
	}
}

static void selfcheck_model_label_opa(lv_opa_t opa)
{
	if(g_selfcheck_lbl_model == NULL) return;
	lv_obj_set_style_text_opa(g_selfcheck_lbl_model, opa, LV_PART_MAIN);
}

static void selfcheck_step_highlight(int active_idx)
{
	for(int i = 0; i < SELFCHECK_STEP_COUNT; i++) {
		if(g_selfcheck_step_btns[i] == NULL) continue;
		lv_obj_t * lbl = admin_menu_btn_get_label(g_selfcheck_step_btns[i]);
		if(lbl == NULL) continue;
		uint32_t col = (i == active_idx) ? COL_TEXT : COL_DIM;
		lv_obj_set_style_text_color(lbl, lv_color_hex(col), LV_PART_MAIN);
	}
}

static void selfcheck_ui_show_model(void)
{
	selfcheck_panel_hide_all();
	g_selfcheck_ui_state = SELFCHECK_UI_MODEL;
	if(g_selfcheck_panel_model != NULL) {
		lv_obj_remove_flag(g_selfcheck_panel_model, LV_OBJ_FLAG_HIDDEN);
	}
	selfcheck_model_label_opa(LV_OPA_COVER);
	selfcheck_apply_step_indicators(SELFCHECK_UI_MODEL);
}

static void selfcheck_schedule_step_timer(void);
static void selfcheck_ui_goto_step(selfcheck_ui_state_t step);

static void cb_selfcheck_timer(lv_timer_t * t)
{
	(void)t;
	if(g_selfcheck_timer != NULL) {
		lv_timer_delete(g_selfcheck_timer);
		g_selfcheck_timer = NULL;
	}

	if(g_selfcheck_ui_state == SELFCHECK_UI_FLASH) {
		g_selfcheck_flash_on = !g_selfcheck_flash_on;
		selfcheck_model_label_opa(g_selfcheck_flash_on ? LV_OPA_COVER : LV_OPA_TRANSP);
		if(!g_selfcheck_flash_on) {
			g_selfcheck_flash_count++;
		}
		if(g_selfcheck_flash_count >= 3u) {
			selfcheck_model_label_opa(LV_OPA_COVER);
			selfcheck_ui_goto_step(SELFCHECK_UI_INIT);
			return;
		}
		g_selfcheck_timer = lv_timer_create(cb_selfcheck_timer, SELFCHECK_FLASH_MS, NULL);
		return;
	}

	if(g_selfcheck_ui_state == SELFCHECK_UI_INIT) {
		selfcheck_ui_goto_step(SELFCHECK_UI_S1);
		return;
	}

	/* 本阶段：纯 UI 定时，到点自动切下一步。
	 * 量产：删除此 timer 自动步进，改由 MCU 在条件满足时调用 selfcheck_ui_goto_step(next)。 */
	if(g_selfcheck_ui_state == SELFCHECK_UI_S1) {
		selfcheck_ui_goto_step(SELFCHECK_UI_S2);
	}
	else if(g_selfcheck_ui_state == SELFCHECK_UI_S2) {
		selfcheck_ui_goto_step(SELFCHECK_UI_S3);
	}
	else if(g_selfcheck_ui_state == SELFCHECK_UI_S3) {
		selfcheck_ui_goto_step(SELFCHECK_UI_DONE);
	}
}

static void selfcheck_schedule_step_timer(void)
{
	selfcheck_timer_stop_all();
	g_selfcheck_timer = lv_timer_create(cb_selfcheck_timer, SELFCHECK_STEP_UI_MS, NULL);
}

static void selfcheck_ui_goto_step(selfcheck_ui_state_t step)
{
	selfcheck_timer_stop_all();
	g_selfcheck_ui_state = step;

	switch(step) {
	case SELFCHECK_UI_MODEL:
		selfcheck_ui_show_model();
		break;

	case SELFCHECK_UI_FLASH:
		selfcheck_panel_hide_all();
		g_selfcheck_flash_count = 0u;
		g_selfcheck_flash_on = false;
		if(g_selfcheck_panel_model != NULL) {
			lv_obj_remove_flag(g_selfcheck_panel_model, LV_OBJ_FLAG_HIDDEN);
			lv_obj_move_foreground(g_selfcheck_panel_model);
		}
		selfcheck_model_label_opa(LV_OPA_COVER);
		g_selfcheck_timer = lv_timer_create(cb_selfcheck_timer, SELFCHECK_FLASH_MS, NULL);
		break;

	case SELFCHECK_UI_INIT:
		/* UI：闪烁结束后展示烘干时间/温度值/追加次数三列占位，再进入 S-1。 */
		selfcheck_panel_hide_all();
		if(g_selfcheck_panel_init != NULL) {
			lv_obj_remove_flag(g_selfcheck_panel_init, LV_OBJ_FLAG_HIDDEN);
			lv_obj_move_foreground(g_selfcheck_panel_init);
		}
		g_selfcheck_timer = lv_timer_create(cb_selfcheck_timer, SELFCHECK_INIT_UI_MS, NULL);
		break;

	case SELFCHECK_UI_S1:
		/* UI：高亮 S-1，SELFCHECK_STEP_UI_MS 后 → S-2。
		 * 硬件（5.1.3）：MCU 电磁门锁上电；量产门锁就绪后调用 selfcheck_ui_goto_step(S2)。 */
		selfcheck_panel_hide_all();
		if(g_selfcheck_panel_steps != NULL) {
			lv_obj_remove_flag(g_selfcheck_panel_steps, LV_OBJ_FLAG_HIDDEN);
			lv_obj_move_foreground(g_selfcheck_panel_steps);
		}
		selfcheck_step_highlight(0);
		selfcheck_schedule_step_timer();
		break;

	case SELFCHECK_UI_S2:
		/* UI：高亮 S-2，SELFCHECK_STEP_UI_MS 后 → S-3。
		 * 硬件（5.1.4）：MCU 电机周期转动+加热；量产就绪/异常上报后驱动步进。 */
		selfcheck_step_highlight(1);
		selfcheck_schedule_step_timer();
		break;

	case SELFCHECK_UI_S3:
		/* UI：高亮 S-3，SELFCHECK_STEP_UI_MS 后 → 自检完成页（不显示 END）。
		 * 硬件（5.1.5）：MCU 筒温 70℃ 切冷却；量产冷却完成后调用 selfcheck_ui_goto_step(DONE)。 */
		selfcheck_step_highlight(2);
		selfcheck_schedule_step_timer();
		break;

	case SELFCHECK_UI_DONE:
		/* UI：展示「自检完成」后停留，不启动延时回主页 timer。
		 * 用户按顶栏「启停」或「返回」→ selfcheck_exit_to_vendor_menu() → VENDOR_MENU。 */
		selfcheck_panel_hide_all();
		if(g_selfcheck_panel_end != NULL) {
			lv_obj_remove_flag(g_selfcheck_panel_end, LV_OBJ_FLAG_HIDDEN);
			lv_obj_move_foreground(g_selfcheck_panel_end);
		}
		break;

	default:
		break;
	}

	selfcheck_apply_step_indicators(step);
}

static void selfcheck_ui_reset(void)
{
	selfcheck_timer_stop_all();
	g_selfcheck_live_temp_c = SELFCHECK_LIVE_TEMP_DEFAULT_C;
	selfcheck_temp_label_sync();
	selfcheck_ui_show_model();
}

static void ui_admin_resume_unlocked(void)
{
	if(g_scr_admin == NULL) return;
	lv_screen_load(g_scr_admin);
	ui_set_encoder_group(g_group_admin);
	ui_idle_on_screen_changed(g_scr_admin);
}

static void selfcheck_exit_to_vendor_menu(void)
{
	selfcheck_timer_stop_all();
	selfcheck_ui_reset();
	ui_admin_resume_unlocked();
	admin_panel_show(VENDOR_MENU);
}

static void selfcheck_abort_to_model(void)
{
	selfcheck_timer_stop_all();
	selfcheck_ui_show_model();
}

static void cb_selfcheck_back(lv_event_t * e)
{
	(void)e;
	if(g_selfcheck_ui_state == SELFCHECK_UI_DONE) {
		selfcheck_exit_to_vendor_menu();
		return;
	}
	if(g_selfcheck_ui_state == SELFCHECK_UI_MODEL) {
		ui_admin_resume_unlocked();
		admin_panel_show(VENDOR_MENU);
		return;
	}
	selfcheck_abort_to_model();
}

static void cb_selfcheck_runpause(lv_event_t * e)
{
	(void)e;
	if(g_selfcheck_ui_state == SELFCHECK_UI_DONE) {
		selfcheck_exit_to_vendor_menu();
		return;
	}
	if(g_selfcheck_ui_state == SELFCHECK_UI_MODEL) {
		/* TODO: notify MCU selfcheck start */
		selfcheck_ui_goto_step(SELFCHECK_UI_FLASH);
		return;
	}
	selfcheck_abort_to_model();
}

static void build_selfcheck(void)
{
	lv_obj_t * root = lv_obj_create(g_scr_selfcheck);
	lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
	lv_obj_set_pos(root, 0, 0);
	lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(root, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(root, 0, LV_PART_MAIN);
	lv_obj_set_style_radius(root, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(root, LV_LAYOUT_NONE, LV_PART_MAIN);

	const lv_coord_t body_y = (lv_coord_t)(UI_FIXED_H * 10 / 100);

	lv_obj_t * top = create_top_bar(root, &g_lbl_clock_selfcheck, NULL, NULL);

	g_selfcheck_btn_back = make_top_back_btn(top, cb_selfcheck_back);

	g_selfcheck_btn_runpause = add_top_text_btn(top, "启停", 100);
	lv_obj_add_event_cb(g_selfcheck_btn_runpause, cb_selfcheck_runpause, LV_EVENT_CLICKED, NULL);
	g_selfcheck_btn_power = add_top_text_btn(top, "电源", 180);

	lv_obj_t * lbl_title = lv_label_create(root);
	ui_lang_bind_label(lbl_title, STR_SELF_CHECK_TITLE);
	lv_obj_set_style_text_color(lbl_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(lbl_title, s_font_sc_30);
	lv_obj_align(lbl_title, LV_ALIGN_TOP_MID, 0, body_y + 8);

	/* 门锁状态：挂在 root，与 S 步顶行左缘对齐（机型/三列占位/S 步共用） */
	g_selfcheck_lbl_doorlock = lv_label_create(root);
	selfcheck_doorlock_set(g_selfcheck_lbl_doorlock, true);
	lv_obj_set_style_text_color(g_selfcheck_lbl_doorlock, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(g_selfcheck_lbl_doorlock, s_font_sc_35);
	lv_obj_align(g_selfcheck_lbl_doorlock, LV_ALIGN_TOP_LEFT,
		(lv_coord_t)((UI_FIXED_W - SELFCHECK_STATUS_ROW_W) / 2), SELFCHECK_STATUS_ROW_Y);
	lv_obj_add_flag(g_selfcheck_lbl_doorlock, LV_OBJ_FLAG_HIDDEN);

	g_selfcheck_panel_model = lv_obj_create(root);
	lv_obj_set_size(g_selfcheck_panel_model, LV_PCT(100), LV_PCT(100));
	lv_obj_set_pos(g_selfcheck_panel_model, 0, 0);
	lv_obj_set_style_bg_opa(g_selfcheck_panel_model, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(g_selfcheck_panel_model, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(g_selfcheck_panel_model, 0, LV_PART_MAIN);
	lv_obj_remove_flag(g_selfcheck_panel_model, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_move_background(g_selfcheck_panel_model);

	g_selfcheck_lbl_model = lv_label_create(g_selfcheck_panel_model);
	lv_label_set_text(g_selfcheck_lbl_model, MACHINE_MODEL);
	lv_obj_set_width(g_selfcheck_lbl_model, LV_SIZE_CONTENT);
	lv_obj_set_style_text_color(g_selfcheck_lbl_model, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_set_style_text_align(g_selfcheck_lbl_model, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	ui_set_obj_font(g_selfcheck_lbl_model, s_font_sc_50);
	lv_obj_align(g_selfcheck_lbl_model, LV_ALIGN_CENTER, 0, 0);

	g_selfcheck_panel_init = lv_obj_create(root);
	lv_obj_set_size(g_selfcheck_panel_init, LV_PCT(100), LV_PCT(100));
	lv_obj_set_pos(g_selfcheck_panel_init, 0, 0);
	lv_obj_set_style_bg_opa(g_selfcheck_panel_init, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(g_selfcheck_panel_init, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(g_selfcheck_panel_init, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(g_selfcheck_panel_init, LV_LAYOUT_NONE, LV_PART_MAIN);
	lv_obj_remove_flag(g_selfcheck_panel_init, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(g_selfcheck_panel_init, LV_OBJ_FLAG_HIDDEN);

	g_selfcheck_lbl_init_dry_time = lv_label_create(g_selfcheck_panel_init);
	lv_label_set_text(g_selfcheck_lbl_init_dry_time, "— — —");
	lv_obj_set_width(g_selfcheck_lbl_init_dry_time, LV_SIZE_CONTENT);
	lv_obj_set_style_text_color(g_selfcheck_lbl_init_dry_time, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_set_style_text_align(g_selfcheck_lbl_init_dry_time, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	ui_set_obj_font(g_selfcheck_lbl_init_dry_time, s_font_sc_35);
	lv_obj_align(g_selfcheck_lbl_init_dry_time, LV_ALIGN_CENTER, -280, 0);

	g_selfcheck_lbl_init_temp = lv_label_create(g_selfcheck_panel_init);
	lv_label_set_text(g_selfcheck_lbl_init_temp, "— — —");
	lv_obj_set_width(g_selfcheck_lbl_init_temp, LV_SIZE_CONTENT);
	lv_obj_set_style_text_color(g_selfcheck_lbl_init_temp, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_set_style_text_align(g_selfcheck_lbl_init_temp, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	ui_set_obj_font(g_selfcheck_lbl_init_temp, s_font_sc_35);
	lv_obj_align(g_selfcheck_lbl_init_temp, LV_ALIGN_CENTER, 0, 0);

	g_selfcheck_lbl_init_add_count = lv_label_create(g_selfcheck_panel_init);
	lv_label_set_text(g_selfcheck_lbl_init_add_count, "— — —");
	lv_obj_set_width(g_selfcheck_lbl_init_add_count, LV_SIZE_CONTENT);
	lv_obj_set_style_text_color(g_selfcheck_lbl_init_add_count, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_set_style_text_align(g_selfcheck_lbl_init_add_count, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	ui_set_obj_font(g_selfcheck_lbl_init_add_count, s_font_sc_35);
	lv_obj_align(g_selfcheck_lbl_init_add_count, LV_ALIGN_CENTER, 280, 0);

	g_selfcheck_panel_steps = lv_obj_create(root);
	lv_obj_set_size(g_selfcheck_panel_steps, LV_PCT(100), LV_PCT(100));
	lv_obj_set_pos(g_selfcheck_panel_steps, 0, 0);
	lv_obj_set_style_bg_opa(g_selfcheck_panel_steps, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(g_selfcheck_panel_steps, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(g_selfcheck_panel_steps, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(g_selfcheck_panel_steps, LV_LAYOUT_NONE, LV_PART_MAIN);
	lv_obj_remove_flag(g_selfcheck_panel_steps, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(g_selfcheck_panel_steps, LV_OBJ_FLAG_HIDDEN);

	/* 顶行右侧（约屏高 35%）：筒温；门锁在 root 层同 Y 左对齐 */
	g_selfcheck_row_status = lv_obj_create(g_selfcheck_panel_steps);
	lv_obj_set_size(g_selfcheck_row_status, SELFCHECK_STATUS_ROW_W, LV_SIZE_CONTENT);
	lv_obj_align(g_selfcheck_row_status, LV_ALIGN_TOP_MID, 0, SELFCHECK_STATUS_ROW_Y);
	lv_obj_set_style_bg_opa(g_selfcheck_row_status, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(g_selfcheck_row_status, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(g_selfcheck_row_status, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(g_selfcheck_row_status, LV_LAYOUT_NONE, LV_PART_MAIN);
	lv_obj_remove_flag(g_selfcheck_row_status, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(g_selfcheck_row_status, LV_OBJ_FLAG_HIDDEN);

	g_selfcheck_lbl_temp_live = lv_label_create(g_selfcheck_row_status);
	selfcheck_temp_label_sync();
	lv_obj_set_style_text_color(g_selfcheck_lbl_temp_live, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_set_style_text_align(g_selfcheck_lbl_temp_live, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
	ui_set_obj_font(g_selfcheck_lbl_temp_live, s_font_sc_35);
	lv_obj_align(g_selfcheck_lbl_temp_live, LV_ALIGN_RIGHT_MID, 0, 0);
	lv_obj_add_flag(g_selfcheck_lbl_temp_live, LV_OBJ_FLAG_HIDDEN);

	/* 底行居中：Dry / Cooling（S-2 起显示，量产替换为图案资源） */
	g_selfcheck_row_indicator = lv_obj_create(g_selfcheck_panel_steps);
	lv_obj_set_size(g_selfcheck_row_indicator, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_align(g_selfcheck_row_indicator, LV_ALIGN_BOTTOM_MID, 0, -100);//底行居中
	lv_obj_set_style_bg_opa(g_selfcheck_row_indicator, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(g_selfcheck_row_indicator, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(g_selfcheck_row_indicator, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_column(g_selfcheck_row_indicator, 32, LV_PART_MAIN);
	lv_obj_set_flex_flow(g_selfcheck_row_indicator, LV_FLEX_FLOW_ROW);
	lv_obj_set_flex_align(g_selfcheck_row_indicator, LV_FLEX_ALIGN_CENTER,
		LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
	lv_obj_remove_flag(g_selfcheck_row_indicator, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(g_selfcheck_row_indicator, LV_OBJ_FLAG_HIDDEN);

	g_selfcheck_lbl_dry = lv_label_create(g_selfcheck_row_indicator);
	lv_label_set_text(g_selfcheck_lbl_dry, "Dry");
	lv_obj_set_style_text_color(g_selfcheck_lbl_dry, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(g_selfcheck_lbl_dry, s_font_sc_35);
	lv_obj_add_flag(g_selfcheck_lbl_dry, LV_OBJ_FLAG_HIDDEN);

	g_selfcheck_lbl_cooling = lv_label_create(g_selfcheck_row_indicator);
	lv_label_set_text(g_selfcheck_lbl_cooling, "Cooling");
	lv_obj_set_style_text_color(g_selfcheck_lbl_cooling, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(g_selfcheck_lbl_cooling, s_font_sc_35);
	lv_obj_add_flag(g_selfcheck_lbl_cooling, LV_OBJ_FLAG_HIDDEN);

	{
		/* 干衣机自检：S-1..S-3 横排，宽约 1/8 屏宽、间距约为钮宽一半 */
		const lv_coord_t step_side_margin = (lv_coord_t)((UI_FIXED_W * 6) / 100);
		const lv_coord_t step_gap = (lv_coord_t)((UI_FIXED_W * 5) / 100);
		const lv_coord_t step_row_w = (lv_coord_t)UI_FIXED_W - step_side_margin * 2;
		const lv_coord_t step_btn_w =
			(step_row_w - step_gap * (SELFCHECK_STEP_COUNT - 1)) / SELFCHECK_STEP_COUNT;
		const lv_coord_t step_btn_h = 96;
		static const char * step_labels[SELFCHECK_STEP_COUNT] = {"S-1", "S-2", "S-3"};

		lv_obj_t * steps_row = lv_obj_create(g_selfcheck_panel_steps);
		lv_obj_set_size(steps_row, step_row_w, step_btn_h);
		lv_obj_align(steps_row, LV_ALIGN_CENTER, 0, 0);
		lv_obj_set_style_bg_opa(steps_row, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(steps_row, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(steps_row, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_column(steps_row, step_gap, LV_PART_MAIN);
		lv_obj_set_flex_flow(steps_row, LV_FLEX_FLOW_ROW);
		lv_obj_set_flex_align(steps_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
		lv_obj_remove_flag(steps_row, LV_OBJ_FLAG_CLICKABLE);

		for(int i = 0; i < SELFCHECK_STEP_COUNT; i++) {
			g_selfcheck_step_btns[i] = make_admin_menu_btn(steps_row, step_labels[i], NULL);
			lv_obj_set_size(g_selfcheck_step_btns[i], step_btn_w, step_btn_h);
			ui_set_obj_font(admin_menu_btn_get_label(g_selfcheck_step_btns[i]), s_font_sc_35);
		}
	}

	g_selfcheck_panel_end = lv_obj_create(root);
	lv_obj_set_size(g_selfcheck_panel_end, LV_PCT(100), LV_PCT(100));
	lv_obj_set_pos(g_selfcheck_panel_end, 0, 0);
	lv_obj_set_style_bg_opa(g_selfcheck_panel_end, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(g_selfcheck_panel_end, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(g_selfcheck_panel_end, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(g_selfcheck_panel_end, LV_LAYOUT_NONE, LV_PART_MAIN);
	lv_obj_remove_flag(g_selfcheck_panel_end, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(g_selfcheck_panel_end, LV_OBJ_FLAG_HIDDEN);

	lv_obj_t * center = lv_obj_create(g_selfcheck_panel_end);
	lv_obj_set_size(center, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_align(center, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_bg_opa(center, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(center, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(center, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_row(center, 16, LV_PART_MAIN);
	lv_obj_set_flex_flow(center, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(center, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
	lv_obj_remove_flag(center, LV_OBJ_FLAG_CLICKABLE);

	LV_IMAGE_DECLARE(success);
	lv_obj_t * img_done = lv_image_create(center);
	lv_image_set_src(img_done, &success);

	g_selfcheck_lbl_done_title = lv_label_create(center);
	lv_obj_set_width(g_selfcheck_lbl_done_title, LV_SIZE_CONTENT);
	lv_obj_set_style_text_color(g_selfcheck_lbl_done_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_set_style_text_align(g_selfcheck_lbl_done_title, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	ui_set_obj_font(g_selfcheck_lbl_done_title, s_font_sc_50);
	ui_lang_bind_label(g_selfcheck_lbl_done_title, STR_SELF_CHECK_DONE);

	if(g_selfcheck_btn_back != NULL) {
		ui_encoder_group_add(g_group_selfcheck, g_selfcheck_btn_back);
	}
	if(g_selfcheck_btn_runpause != NULL) {
		ui_encoder_group_add(g_group_selfcheck, g_selfcheck_btn_runpause);
	}
	if(g_selfcheck_btn_power != NULL) {
		ui_encoder_group_add(g_group_selfcheck, g_selfcheck_btn_power);
	}
	if(g_selfcheck_btn_back != NULL) {
		lv_group_focus_obj(g_selfcheck_btn_back);
	}
}

/* ========== 循环程序（5.2.1 寿命试验）UI ========== */

// 刷新「已完成：N 次」标签
static void cycle_run_count_label_sync(void)
{
	if(g_cycle_lbl_run_count == NULL) return;
	char buf[48];
	const char * fmt = ui_translation(STR_CYCLE_RUN_COUNT_FMT);
	lv_snprintf(buf, sizeof(buf), fmt, (unsigned)g_cycle_session.run_count);
	lv_label_set_text(g_cycle_lbl_run_count, buf);
}

static lv_obj_t * cycle_fault_panel_active(void)
{
	if(g_cycle_fault_on_running) return g_cycle_panel_fault_run;
	return g_cycle_panel_fault;
}

static lv_obj_t * cycle_fault_lbl_active(void)
{
	if(g_cycle_fault_on_running) return g_cycle_lbl_fault_alt_run;
	return g_cycle_lbl_fault_alt;
}

// 停止故障 Ex↔次数 四相闪烁
static void cycle_fault_blink_stop(void)
{
	if(g_cycle_fault_blink_timer != NULL) {
		lv_timer_pause(g_cycle_fault_blink_timer);
	}
	if(g_cycle_panel_fault != NULL) {
		lv_obj_add_flag(g_cycle_panel_fault, LV_OBJ_FLAG_HIDDEN);
	}
	if(g_cycle_lbl_fault_alt != NULL) {
		lv_obj_add_flag(g_cycle_lbl_fault_alt, LV_OBJ_FLAG_HIDDEN);
	}
	if(g_cycle_panel_fault_run != NULL) {
		lv_obj_add_flag(g_cycle_panel_fault_run, LV_OBJ_FLAG_HIDDEN);
	}
	if(g_cycle_lbl_fault_alt_run != NULL) {
		lv_obj_add_flag(g_cycle_lbl_fault_alt_run, LV_OBJ_FLAG_HIDDEN);
	}
	g_cycle_fault_phase = CYCLE_FAULT_SHOW_CODE;
}

// 故障叠层四相 0.5s：Ex → 灭 → 次数 → 灭
static void cycle_fault_blink_cb(lv_timer_t * t)
{
	(void)t;
	lv_obj_t * panel = cycle_fault_panel_active();
	lv_obj_t * lbl = cycle_fault_lbl_active();
	if(lbl == NULL || panel == NULL) return;
	if(g_cycle_ui_state != CYCLE_UI_FAULT) return;

	char buf[16];
	switch(g_cycle_fault_phase) {
	case CYCLE_FAULT_SHOW_CODE:
		lv_snprintf(buf, sizeof(buf), "E%u", (unsigned)g_cycle_session.fault_code);
		lv_label_set_text(lbl, buf);
		lv_obj_remove_flag(lbl, LV_OBJ_FLAG_HIDDEN);
		g_cycle_fault_phase = CYCLE_FAULT_GAP1;
		break;
	case CYCLE_FAULT_GAP1:
		lv_obj_add_flag(lbl, LV_OBJ_FLAG_HIDDEN);
		g_cycle_fault_phase = CYCLE_FAULT_SHOW_COUNT;
		break;
	case CYCLE_FAULT_SHOW_COUNT:
		lv_snprintf(buf, sizeof(buf), "%u", (unsigned)g_cycle_session.fault_at_count);
		lv_label_set_text(lbl, buf);
		lv_obj_remove_flag(lbl, LV_OBJ_FLAG_HIDDEN);
		g_cycle_fault_phase = CYCLE_FAULT_GAP2;
		break;
	case CYCLE_FAULT_GAP2:
	default:
		lv_obj_add_flag(lbl, LV_OBJ_FLAG_HIDDEN);
		g_cycle_fault_phase = CYCLE_FAULT_SHOW_CODE;
		break;
	}
}

// 启动故障 Ex↔次数 交替显示
static void cycle_fault_blink_start(void)
{
	lv_obj_t * panel = cycle_fault_panel_active();
	if(panel == NULL) return;
	lv_obj_remove_flag(panel, LV_OBJ_FLAG_HIDDEN);
	g_cycle_fault_phase = CYCLE_FAULT_SHOW_CODE;
	if(g_cycle_fault_blink_timer == NULL) {
		g_cycle_fault_blink_timer = lv_timer_create(
			cycle_fault_blink_cb, CYCLE_FAULT_BLINK_MS, NULL);
	} else {
		lv_timer_reset(g_cycle_fault_blink_timer);
		lv_timer_resume(g_cycle_fault_blink_timer);
	}
	cycle_fault_blink_cb(g_cycle_fault_blink_timer);
}

// 彻底退出循环模式：清会话、计数、故障与定时器
static void cycle_ui_reset(void)
{
	g_cycle_active = false;
	g_cycle_ui_state = CYCLE_UI_SETUP;
	g_cycle_fault_on_running = false;
	lv_memzero(&g_cycle_session, sizeof(g_cycle_session));
	cycle_fault_blink_stop();
	cycle_kb_close();
	add_time_session_clear();
}

// 将快照 cfg 写入运行会话（跳过支付/追加页）
static void cycle_session_apply_to_running(void)
{
	const ui_program_admin_t * c = &g_cycle_session.cfg;

	g_wheel_sel = g_cycle_session.prog_idx;
	g_session_add_count = g_cycle_session.add_sel;
	g_session_total_price = add_time_calc_price(c, g_cycle_session.add_sel);
	g_session_total_sec = add_time_calc_total_sec(c, g_cycle_session.add_sel);
	add_time_calc_run_stages(c, g_cycle_session.add_sel, &g_session_run_stages);
	g_session_active = true;
}

// 锁定当前 UI 参数到 session 快照（后续每轮复用）
static void cycle_session_snapshot_from_ui(void)
{
	prog_ui_save_fields(&g_pui_cycle);
	g_cycle_session.prog_idx = g_pui_cycle.sel;
	g_cycle_session.cfg = g_cycle_cfg[g_pui_cycle.sel];
	if(g_cycle_session.cfg.cap & PROG_CAP_ADD_COUNT) {
		g_cycle_session.add_sel = g_cycle_session.cfg.add_count;
	} else {
		g_cycle_session.add_sel = 0u;
	}
}

// 进入故障显示：运行中留在运行页叠层闪烁；非运行态回设定页
static void cycle_enter_fault_display(void)
{
	bool on_running = (lv_scr_act() == g_scr_running) || (g_cycle_ui_state == CYCLE_UI_RUNNING);

	g_cycle_ui_state = CYCLE_UI_FAULT;
	fsm_state_change(FSM_STANDBY);
	g_cycle_fault_on_running = on_running;

	if(g_cycle_fault_on_running) {
		cycle_run_count_label_sync();
		cycle_fault_blink_start();
	} else {
		ui_screen_load(g_scr_cycle);
		cycle_run_count_label_sync();
		cycle_fault_blink_start();
	}
}

// 锁定快照并跳过支付进入运行页（前向声明，定义见 cycle_on_run_finished 之后）
static void cycle_start_run(void);

// 本轮烘干结束：有故障则闪显，否则立即下一轮
static void cycle_on_run_finished(void)
{
	if(g_cycle_session.fault_code != 0u) {
		cycle_enter_fault_display();
		return;
	}
	cycle_start_run(); // 无故障：用启动快照自动开跑
}

// 锁定快照并跳过支付进入运行页
static void cycle_start_run(void)
{
	if(!g_cycle_active) return;

	/* 首次启动或中断后重开：锁定 UI 快照；自动循环轮次复用已有快照 */
	if(g_cycle_ui_state != CYCLE_UI_RUNNING) {
		cycle_session_snapshot_from_ui();
	}
	cycle_session_apply_to_running();
	g_cycle_ui_state = CYCLE_UI_RUNNING;
	ui_cycle_mode_enter(); // 通知 MCU 进入寿命试验（量产实现）
	fsm_state_change(FSM_RUNNING);
	ui_screen_load(g_scr_running);
}

// 运行中/故障时启停中断：停倒计时，回设定页
static void cycle_abort_run(void)
{
	running_countdown_reset_all();
	cycle_fault_blink_stop();
	cycle_kb_close();
	g_cycle_session.fault_code = 0u;
	g_cycle_ui_state = CYCLE_UI_ABORTED;
	g_cycle_fault_on_running = false;
	fsm_state_change(FSM_STANDBY);
	ui_screen_load(g_scr_cycle);
	cycle_run_count_label_sync();
}

// 单轮完成（UI 倒计时或 MCU 确认）
static void cycle_finish_current_run(void)
{
	if(g_cycle_ui_state != CYCLE_UI_RUNNING) return;
	running_countdown_reset_all();
	g_cycle_session.run_count++;
	cycle_run_count_label_sync();
	cycle_on_run_finished();
}

// 返回厂商维护菜单（两个按钮页）
static void cycle_exit_to_vendor_menu(void)
{
	cycle_kb_close();
	cycle_ui_reset();
	ui_cycle_mode_exit();
	/* 与 selfcheck_exit_to_vendor_menu 相同：勿走 ui_screen_load(g_scr_admin)，否则会 admin_session_reset 回密码页 */
	ui_admin_resume_unlocked();
	admin_panel_show(VENDOR_MENU);
}

static void cb_cycle_back(lv_event_t * e)
{
	(void)e;
	cycle_exit_to_vendor_menu();
}

static void cb_cycle_runpause(lv_event_t * e)
{
	(void)e;
	if(g_cycle_ui_state == CYCLE_UI_RUNNING) {
		cycle_abort_run();
		return;
	}
	if(g_cycle_ui_state == CYCLE_UI_FAULT) {
		cycle_abort_run();
		return;
	}
	/* 设定页 / 中断后：启停开始或重新开始 */
	cycle_start_run();
}

static void build_cycle(void)
{
	lv_obj_t * root = lv_obj_create(g_scr_cycle);
	lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
	lv_obj_set_pos(root, 0, 0);
	lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(root, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(root, 0, LV_PART_MAIN);
	lv_obj_set_style_radius(root, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(root, LV_LAYOUT_NONE, LV_PART_MAIN);

	const lv_coord_t body_y = (lv_coord_t)(UI_FIXED_H * 10 / 100);
	const lv_coord_t panel_h = (lv_coord_t)(UI_FIXED_H * 72 / 100);

	lv_obj_t * top = create_top_bar(root, &g_lbl_clock_cycle, NULL, NULL);

	g_cycle_btn_back = make_top_back_btn(top, cb_cycle_back);

	g_cycle_btn_runpause = add_top_text_btn(top, "启停", 100);
	lv_obj_add_event_cb(g_cycle_btn_runpause, cb_cycle_runpause, LV_EVENT_CLICKED, NULL);
	g_cycle_btn_power = add_top_text_btn(top, "电源", 180);
	lv_obj_add_event_cb(g_cycle_btn_power, cb_power_long, LV_EVENT_LONG_PRESSED, NULL);

	lv_obj_t * lbl_title = lv_label_create(root);
	ui_lang_bind_label(lbl_title, STR_CYCLE_TITLE);
	lv_obj_set_style_text_color(lbl_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(lbl_title, s_font_sc_30);
	lv_obj_align(lbl_title, LV_ALIGN_TOP_MID, 0, body_y + 4);

	g_cycle_panel_program = lv_obj_create(root);
	lv_obj_set_size(g_cycle_panel_program, LV_PCT(100), panel_h);
	lv_obj_align(g_cycle_panel_program, LV_ALIGN_TOP_MID, 0, body_y);
	lv_memzero(&g_pui_cycle, sizeof(g_pui_cycle));
	prog_ui_build_cycle_program_panel(g_cycle_panel_program, panel_h);

	/* 数字键盘（与管理员程序设置相同交互） */
	g_cycle_kb = lv_keyboard_create(root);
	lv_obj_set_size(g_cycle_kb, LV_PCT(100), 190);
	lv_obj_align(g_cycle_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
	lv_keyboard_set_mode(g_cycle_kb, LV_KEYBOARD_MODE_NUMBER);
	if(s_font_admin_kb_ptr == NULL) admin_kb_font_init();
	if(s_font_admin_kb_ptr != NULL) {
		lv_obj_set_style_text_font(g_cycle_kb, s_font_admin_kb_ptr, LV_PART_MAIN);
		lv_obj_set_style_text_font(g_cycle_kb, s_font_admin_kb_ptr, LV_PART_ITEMS);
	}
	cycle_kb_encoder_style_init();
	lv_obj_add_event_cb(g_cycle_kb, cb_cycle_kb_cancel, LV_EVENT_CANCEL, NULL);
	lv_obj_add_flag(g_cycle_kb, LV_OBJ_FLAG_HIDDEN);

	/* 故障叠层：全屏半透明 + 居中大字 Ex↔次数（设定页；运行中故障用运行页叠层） */
	cycle_fault_overlay_create(root, &g_cycle_panel_fault, &g_cycle_lbl_fault_alt);

	cycle_encoder_group_build();
}

/* MCU：通知进入循环程序（寿命试验）模式；量产由驱动层实现 */
void ui_cycle_mode_enter(void)
{
	/* TODO: MCU — 进入寿命试验，下发锁定程序参数与 add_sel */
}

/* MCU：通知退出循环程序模式 */
void ui_cycle_mode_exit(void)
{
	/* TODO: MCU — 退出寿命试验，停止烘干并释放资源 */
}

/* MCU：本轮烘干完成；量产优先于 UI 倒计时，须在 LVGL 任务上下文调用 */
void ui_cycle_run_complete(void)
{
	if(!g_cycle_active) return;
	cycle_finish_current_run();
}

/* MCU：上报故障码 1..14（E1..E14），触发 Ex↔次数 0.5s 交替显示 */
void ui_cycle_report_fault(uint8_t fault_code_1_based)
{
	if(!g_cycle_active) return;
	if(fault_code_1_based < 1u || fault_code_1_based > CYCLE_FAULT_CODE_MAX) return;
	g_cycle_session.fault_code = fault_code_1_based;
	g_cycle_session.fault_at_count = g_cycle_session.run_count;
	if(g_cycle_ui_state == CYCLE_UI_RUNNING) {
		running_countdown_reset_all();
		fsm_state_change(FSM_STANDBY);
	}
	cycle_enter_fault_display();
}

/* 读取当前已完成烘干轮次 */
uint32_t ui_cycle_run_count_get(void)
{
	return g_cycle_session.run_count;
}

/* 是否处于循环程序（寿命试验）模式 */
bool ui_cycle_is_active(void)
{
	return g_cycle_active;
}

static void cb_admin_open_cycle(lv_event_t * e)
{
	(void)e;
	selfcheck_timer_stop_all(); // 与自检互斥
	cycle_ui_reset();
	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		g_cycle_cfg[i] = g_prog_cfg[i]; // 只读拷贝，不写回出厂表
	}
	g_cycle_active = true;
	g_cycle_ui_state = CYCLE_UI_SETUP;
	g_pui_cycle.cfg_tbl = g_cycle_cfg;
	g_pui_cycle.sel = 0;
	prog_ui_load_fields(&g_pui_cycle);
	cycle_run_count_label_sync();
	cycle_encoder_group_build();
	ui_screen_load(g_scr_cycle);
}










/* PC 仿真液位；量产时改为寄存器读取 */
static uint8_t g_fluid_softener_pct = 50u;  //柔顺剂液位百分比
static uint8_t g_fluid_detergent_pct = 50u; //洗涤剂液位百分比

//PC仿真缺液/故障，实机需根据情况删掉或改为读寄存器
#define UI_FLUID_LEVEL_LOW_THRESHOLD  20u   //液位低于此阈值视为缺液（0=空 100=满）
#define UI_ALARM_FAULT_COUNT          14u   //故障码 E1..E14
#define UI_ALARM_FAULT_STRIDE         3u    //每种故障占 3 个 STR id：TITLE/LINE1/LINE2
#define UI_ALARM_ROTATE_PERIOD_MS     3000u //轮播间隔 3 秒
#define UI_ALARM_ROTATE_MAX           16u   //最多 14 故障 + 2 缺液
#define UI_ALARM_FAULT_LOGO_BLINK_MS  500u  //fault_logo 亮/灭各 0.5s
#define UI_ALARM_FAULT_LOGO_BLINK_TOGGLES 4u //闪烁两次=4 次亮灭切换后常亮
#define COL_ALARM_PHONE               0x3399FFu //故障页电话高亮色

typedef enum {
	ALARM_PANEL_NONE = 0,
	ALARM_PANEL_FAULT_E1 = 1,
	ALARM_PANEL_FAULT_E14 = 14,
	ALARM_PANEL_FLUID_SOFTENER,
	ALARM_PANEL_FLUID_DETERGENT,
} alarm_panel_t;

typedef struct {
	bool active;            //当前条件是否成立
	uint32_t trigger_tick;  //上升沿时刻；清除时置 0
} ui_alarm_trigger_t;

typedef struct {
	lv_obj_t * panel;       //故障子面板根，默认隐藏
	lv_obj_t * logo_slot;   //固定尺寸占位，闪烁时仍占位以免文字上移
	lv_obj_t * img_logo;    //fault_logo 警示图标（在 slot 内显隐）
	lv_obj_t * title_row;   //(Ex) + 故障名 同行容器
	lv_obj_t * lbl_code;    //故障码 (E1)..(E14)，不闪烁
	lv_obj_t * lbl_title;   //故障名称
	lv_obj_t * text_block;  //说明+页脚：整体居中、内部左对齐
	lv_obj_t * lbl_line1;   //说明1（可空）
	lv_obj_t * lbl_line2;   //说明2（可空；烘干文案保留）
	lv_obj_t * call_row;    //拨打引导 + 电话
	lv_obj_t * lbl_line3;   //拨打引导（共享页脚）
	lv_obj_t * lbl_phone;   //电话（蓝色，共享页脚）
	lv_obj_t * lbl_line4;   //售后句（共享页脚）
} ui_alarm_fault_ui_t;

static ui_alarm_fault_ui_t g_alarm_fault_ui[UI_ALARM_FAULT_COUNT]; //E1..E14 UI 槽位
static ui_alarm_trigger_t g_alarm_trig_fault[UI_ALARM_FAULT_COUNT]; //E1..E14 触发记录
static ui_alarm_trigger_t g_alarm_trig_softener;  //柔顺剂缺液触发
static ui_alarm_trigger_t g_alarm_trig_detergent; //洗涤剂缺液触发
static bool g_sim_fault_e[UI_ALARM_FAULT_COUNT];  //PC 仿真 E1..E14
static alarm_panel_t g_alarm_rotate_list[UI_ALARM_ROTATE_MAX]; //按出现顺序排列的活跃项
static uint8_t g_alarm_rotate_count;              //轮播队列长度
static uint8_t g_alarm_rotate_idx;              //当前轮播下标
static lv_timer_t * g_alarm_rotate_timer;         //3 秒轮播定时器
static lv_timer_t * g_alarm_fault_logo_blink_timer; //fault_logo 0.5s 闪烁定时器
static uint8_t g_alarm_fault_logo_blink_idx = 0xFFu;  //当前闪烁的故障索引；0xFF=无
static uint8_t g_alarm_fault_logo_blink_remain;       //剩余亮灭切换次数
static alarm_panel_t g_alarm_rotate_cur_panel;    //当前显示的 panel
static bool g_alarm_user_dismissed;               //用户手动返回后暂不再自动弹出
static bool g_alarm_overlay_open;               //弹层是否处于显示状态（含 encoder 切换）

static const char * const g_alarm_fault_codes[UI_ALARM_FAULT_COUNT] = {
	"E1", "E2", "E3", "E4", "E5", "E6", "E7",
	"E8", "E9", "E10", "E11", "E12", "E13", "E14"
};

//根据故障索引与字段下标取 i18n 字符串 id（每种故障 3 个连续 STR：TITLE/LINE1/LINE2）
static ui_str_id_t alarm_fault_str_id(uint8_t fault_idx, uint8_t part)
{
	return (ui_str_id_t)(STR_ALARM_E1_TITLE + (ui_str_id_t)fault_idx * UI_ALARM_FAULT_STRIDE + part);
}

//故障索引转 alarm_panel_t（0=E1）
static alarm_panel_t alarm_panel_from_fault_idx(uint8_t fault_idx)
{
	return (alarm_panel_t)(ALARM_PANEL_FAULT_E1 + fault_idx);
}

//读取第 idx 路故障是否活跃（0=E1；PC 用 g_sim_fault_e）
static bool ui_fault_ex_active(uint8_t fault_idx)
{
	if(fault_idx >= UI_ALARM_FAULT_COUNT) return false; //越界保护
	return g_sim_fault_e[fault_idx];                    //实机：改为读硬件/FSM 位
}

#if !USE_RTOS_FREERTOS
//PC 仿真 API（供 pc_sim/alarm_fault_sim.c 调用；真机 USE_RTOS_FREERTOS=1 时不编译）
void ui_pc_sim_fault_clear_all(void)
{
	for(uint8_t i = 0; i < UI_ALARM_FAULT_COUNT; i++) {
		g_sim_fault_e[i] = false;                                      //清除全部 Ex
	}
}

//PC 仿真：显示单路故障
void ui_pc_sim_fault_show_one(uint8_t fault_idx)
{
	if(fault_idx >= UI_ALARM_FAULT_COUNT) return;                        //越界保护
	ui_pc_sim_fault_clear_all();                                         //先清再开，保证上升沿
	g_sim_fault_e[fault_idx] = true;                                     //仅保留目标故障
	g_alarm_user_dismissed = false;                                      //允许弹层再次弹出
}
#endif

//柔顺剂是否不足
bool ui_is_softener_low(void)
{
	return g_fluid_softener_pct < UI_FLUID_LEVEL_LOW_THRESHOLD; //低于阈值
}

//洗涤剂是否不足
bool ui_is_detergent_low(void)
{
	return g_fluid_detergent_pct < UI_FLUID_LEVEL_LOW_THRESHOLD; //低于阈值
}

//报警弹层是否可见
static bool ui_alarm_overlay_is_visible(void)
{
	if(g_alarm_overlay == NULL) return false;                              //未构建
	return !lv_obj_has_flag(g_alarm_overlay, LV_OBJ_FLAG_HIDDEN);          //未隐藏即可见
}

//边沿检测：更新单路 trigger 的 active 与 trigger_tick
static void ui_alarm_trigger_edge(ui_alarm_trigger_t * trig, bool now_active, uint32_t tick_seed)
{
	if(trig == NULL) return;                                               //空指针保护
	if(now_active && !trig->active) {                                      //上升沿
		trig->active = true;                                               //标记活跃
		trig->trigger_tick = lv_tick_get() + tick_seed;                    //记录顺序（可加 seed 区分同 tick）
	}
	else if(!now_active && trig->active) {                                 //下降沿
		trig->active = false;                                              //清除活跃
		trig->trigger_tick = 0u;                                           //清零时间戳
	}
}

//同步 14 路故障 + 2 路缺液的边沿状态
static void ui_alarm_trigger_sync(void)
{
	uint32_t seed = 0u;                                                    //同 tick 顺序偏移
	for(uint8_t i = 0; i < UI_ALARM_FAULT_COUNT; i++) {                    //遍历 E1..E14
		ui_alarm_trigger_edge(&g_alarm_trig_fault[i], ui_fault_ex_active(i), seed++); //故障边沿
	}
	ui_alarm_trigger_edge(&g_alarm_trig_softener, ui_is_softener_low(), seed++);     //柔顺剂
	ui_alarm_trigger_edge(&g_alarm_trig_detergent, ui_is_detergent_low(), seed++);   //洗涤剂
}

//将 panel 与 trigger 写入收集缓冲（内部用）
static void ui_alarm_rotate_collect(alarm_panel_t panel, uint32_t tick,
	alarm_panel_t * buf, uint32_t * ticks, uint8_t * cnt)
{
	if(*cnt >= UI_ALARM_ROTATE_MAX) return;                               //缓冲满
	buf[*cnt] = panel;                                                   //记录 panel
	ticks[*cnt] = tick;                                                    //记录 tick
	(*cnt)++;                                                              //计数+1
}

//按 trigger_tick 升序重建轮播队列；返回 true 表示队列或当前项有变化
static bool ui_alarm_rotate_rebuild(void)
{
	alarm_panel_t tmp_panel[UI_ALARM_ROTATE_MAX];                          //临时 panel 列表
	uint32_t tmp_tick[UI_ALARM_ROTATE_MAX];                                //临时 tick 列表
	uint8_t n = 0u;                                                        //收集数量

	for(uint8_t i = 0; i < UI_ALARM_FAULT_COUNT; i++) {                    //收集活跃故障
		if(g_alarm_trig_fault[i].active) {
			ui_alarm_rotate_collect(alarm_panel_from_fault_idx(i),
				g_alarm_trig_fault[i].trigger_tick, tmp_panel, tmp_tick, &n);
		}
	}
	if(g_alarm_trig_softener.active) {                                     //收集柔顺剂
		ui_alarm_rotate_collect(ALARM_PANEL_FLUID_SOFTENER,
			g_alarm_trig_softener.trigger_tick, tmp_panel, tmp_tick, &n);
	}
	if(g_alarm_trig_detergent.active) {                                    //收集洗涤剂
		ui_alarm_rotate_collect(ALARM_PANEL_FLUID_DETERGENT,
			g_alarm_trig_detergent.trigger_tick, tmp_panel, tmp_tick, &n);
	}

	//冒泡排序：按 tick 升序（先出现的在前）
	for(uint8_t a = 0; a + 1u < n; a++) {
		for(uint8_t b = 0; b + 1u < n - a; b++) {
			if(tmp_tick[b] > tmp_tick[b + 1u]) {
				uint32_t ts = tmp_tick[b]; tmp_tick[b] = tmp_tick[b + 1u]; tmp_tick[b + 1u] = ts;
				alarm_panel_t tp = tmp_panel[b]; tmp_panel[b] = tmp_panel[b + 1u]; tmp_panel[b + 1u] = tp;
			}
		}
	}

	alarm_panel_t prev_cur = ALARM_PANEL_NONE;                             //记录重建前当前项
	if(g_alarm_rotate_count > 0u && g_alarm_rotate_idx < g_alarm_rotate_count) {
		prev_cur = g_alarm_rotate_list[g_alarm_rotate_idx];                //当前显示 panel
	}

	bool changed = (n != g_alarm_rotate_count);                            //数量变化
	for(uint8_t i = 0; i < n; i++) {                                       //内容变化检测
		if(!changed && g_alarm_rotate_list[i] != tmp_panel[i]) changed = true;
	}

	g_alarm_rotate_count = n;                                              //写回队列长度
	for(uint8_t i = 0; i < n; i++) g_alarm_rotate_list[i] = tmp_panel[i]; //写回队列

	if(n == 0u) {                                                          //无活跃报警
		g_alarm_rotate_idx = 0u;                                           //下标复位
		g_alarm_user_dismissed = false;                                    //清除用户 dismiss
		return changed;                                                    //返回变化标志
	}

	if(prev_cur != ALARM_PANEL_NONE) {                                     //尝试保留当前显示项
		uint8_t found = 0xFFu;
		for(uint8_t i = 0; i < n; i++) {
			if(g_alarm_rotate_list[i] == prev_cur) { found = i; break; }   //找到原 panel 新下标
		}
		if(found == 0xFFu) {                                               //当前项已被移除
			if(g_alarm_rotate_idx >= n) g_alarm_rotate_idx = 0u;           //下标越界则回 0
			changed = true;                                                //需要立即刷新
		}
		else {
			g_alarm_rotate_idx = found;                                    //保持显示同一 panel
		}
	}
	else {
		g_alarm_rotate_idx = 0u;                                         //首次从最早项开始
	}

	return changed;                                                        //返回是否需刷新 UI
}

//是否存在任意活跃报警
static bool ui_alarm_any_active(void)
{
	return g_alarm_rotate_count > 0u;                                      //队列非空
}

//隐藏全部故障/缺液子面板
static void alarm_panel_hide_all(void)
{
	for(uint8_t i = 0; i < UI_ALARM_FAULT_COUNT; i++) {                    //隐藏 E1..E14
		if(g_alarm_fault_ui[i].panel != NULL) {
			lv_obj_add_flag(g_alarm_fault_ui[i].panel, LV_OBJ_FLAG_HIDDEN);
		}
	}
	if(g_alarm_panel_fluid != NULL) {                                      //隐藏缺液
		lv_obj_add_flag(g_alarm_panel_fluid, LV_OBJ_FLAG_HIDDEN);
	}
}

//停止 fault_logo 闪烁，并恢复全部 logo 为可见（常亮）；占位槽始终保留
static void alarm_fault_logo_blink_stop(void)
{
	if(g_alarm_fault_logo_blink_timer != NULL) {
		lv_timer_pause(g_alarm_fault_logo_blink_timer);
	}
	for(uint8_t i = 0; i < UI_ALARM_FAULT_COUNT; i++) {
		if(g_alarm_fault_ui[i].img_logo != NULL) {
			lv_obj_remove_flag(g_alarm_fault_ui[i].img_logo, LV_OBJ_FLAG_HIDDEN);
		}
	}
	g_alarm_fault_logo_blink_idx = 0xFFu;
	g_alarm_fault_logo_blink_remain = 0u;
}

/* 只隐藏/显示 slot 内的图；logo_slot 固定尺寸仍参与 flex，下方文字不跳 */
static void alarm_fault_logo_blink_cb(lv_timer_t * t)
{
	(void)t;
	if(g_alarm_fault_logo_blink_idx >= UI_ALARM_FAULT_COUNT) return;
	lv_obj_t * logo = g_alarm_fault_ui[g_alarm_fault_logo_blink_idx].img_logo;
	if(logo == NULL) return;
	if(lv_obj_has_flag(logo, LV_OBJ_FLAG_HIDDEN)) {
		lv_obj_remove_flag(logo, LV_OBJ_FLAG_HIDDEN);
	}
	else {
		lv_obj_add_flag(logo, LV_OBJ_FLAG_HIDDEN);
	}
	if(g_alarm_fault_logo_blink_remain > 0u) {
		g_alarm_fault_logo_blink_remain--;
	}
	if(g_alarm_fault_logo_blink_remain == 0u) {
		lv_obj_remove_flag(logo, LV_OBJ_FLAG_HIDDEN); /* 结束后常亮 */
		if(g_alarm_fault_logo_blink_timer != NULL) {
			lv_timer_pause(g_alarm_fault_logo_blink_timer);
		}
		g_alarm_fault_logo_blink_idx = 0xFFu;
	}
}

//进入故障子页：fault_logo 亮 0.5s / 灭 0.5s 闪烁两次，随后常亮；(Ex) 不闪烁
static void alarm_fault_logo_blink_start(uint8_t fault_idx)
{
	if(fault_idx >= UI_ALARM_FAULT_COUNT) return;
	alarm_fault_logo_blink_stop();
	g_alarm_fault_logo_blink_idx = fault_idx;
	g_alarm_fault_logo_blink_remain = UI_ALARM_FAULT_LOGO_BLINK_TOGGLES;
	if(g_alarm_fault_ui[fault_idx].img_logo != NULL) {
		lv_obj_remove_flag(g_alarm_fault_ui[fault_idx].img_logo, LV_OBJ_FLAG_HIDDEN);
	}
	if(g_alarm_fault_logo_blink_timer == NULL) {
		g_alarm_fault_logo_blink_timer = lv_timer_create(
			alarm_fault_logo_blink_cb, UI_ALARM_FAULT_LOGO_BLINK_MS, NULL);
	}
	else {
		lv_timer_reset(g_alarm_fault_logo_blink_timer);
		lv_timer_resume(g_alarm_fault_logo_blink_timer);
	}
}

//显示指定子面板并刷新缺液内容
static void alarm_panel_show(alarm_panel_t panel)
{
	alarm_panel_hide_all();                                                //先全部隐藏
	g_alarm_rotate_cur_panel = panel;                                      //记录当前 panel

	if(panel >= ALARM_PANEL_FAULT_E1 && panel <= ALARM_PANEL_FAULT_E14) {   //故障码子页
		uint8_t idx = (uint8_t)(panel - ALARM_PANEL_FAULT_E1);             //转 fault 索引
		if(g_alarm_fault_ui[idx].panel != NULL) {
			lv_obj_remove_flag(g_alarm_fault_ui[idx].panel, LV_OBJ_FLAG_HIDDEN); //显示目标
		}
		alarm_fault_logo_blink_start(idx);                                 //fault_logo 闪烁两次
		alarm_fault_panel_relayout_content(idx);                           //空说明时隐藏说明行
	}
	else if(panel == ALARM_PANEL_FLUID_SOFTENER || panel == ALARM_PANEL_FLUID_DETERGENT) {
		alarm_fault_logo_blink_stop();                                     //缺液页不闪 logo
		if(g_alarm_panel_fluid != NULL) {
			lv_obj_remove_flag(g_alarm_panel_fluid, LV_OBJ_FLAG_HIDDEN); //显示缺液 panel
		}
	}
	else {
		alarm_fault_logo_blink_stop();                                     //其它情况停止 logo 闪烁
	}
	alarm_content_update();                                                //刷新缺液图/文案
}

//刷新轮播当前子页内容（缺液图/文案；故障文案由 ui_lang_bind 负责）
static void alarm_content_update(void)
{
	if(g_alarm_rotate_count == 0u) return;                                 //无报警
	alarm_panel_t panel = g_alarm_rotate_list[g_alarm_rotate_idx];       //当前轮播项
	if(panel != ALARM_PANEL_FLUID_SOFTENER && panel != ALARM_PANEL_FLUID_DETERGENT) {
		return;                                                            //非缺液无需刷新
	}
	if(g_alarm_img_warn == NULL || g_alarm_lbl_hint == NULL) return;       //控件未就绪

	if(panel == ALARM_PANEL_FLUID_SOFTENER) {                              //柔顺剂不足
		lv_image_set_src(g_alarm_img_warn, &lack_of_softener);             //换图
		lv_label_set_text(g_alarm_lbl_hint, ui_translation(STR_ALARM_SOFTENER_LOW)); //换文案
	}
	else if(panel == ALARM_PANEL_FLUID_DETERGENT) {                        //洗涤剂不足
		lv_image_set_src(g_alarm_img_warn, &lack_of_detergent);            //换图
		lv_label_set_text(g_alarm_lbl_hint, ui_translation(STR_ALARM_DETERGENT_LOW)); //换文案
	}
}

//显示轮播队列当前项
static void alarm_rotate_show_current(void)
{
	if(g_alarm_rotate_count == 0u) return;                                 //队列为空
	if(g_alarm_rotate_idx >= g_alarm_rotate_count) g_alarm_rotate_idx = 0u; //下标保护
	alarm_panel_show(g_alarm_rotate_list[g_alarm_rotate_idx]);              //显示当前项
}

//轮播前进到下一项（循环）
static void alarm_rotate_advance(void)
{
	if(g_alarm_rotate_count <= 1u) return;                                 //单项不切换
	g_alarm_rotate_idx = (uint8_t)((g_alarm_rotate_idx + 1u) % g_alarm_rotate_count); //下一项
	alarm_rotate_show_current();                                           //刷新显示
}

//3 秒轮播定时器回调
static void alarm_rotate_timer_cb(lv_timer_t * t)
{
	(void)t;                                                               //未使用
	alarm_rotate_advance();                                                //切下一故障/缺液页
}

//启动 3 秒轮播定时器
static void alarm_rotate_timer_start(void)
{
	if(g_alarm_rotate_timer == NULL) {                                     //首次创建
		g_alarm_rotate_timer = lv_timer_create(alarm_rotate_timer_cb, UI_ALARM_ROTATE_PERIOD_MS, NULL);
	}
	else {
		lv_timer_reset(g_alarm_rotate_timer);                              //重置计时
		lv_timer_resume(g_alarm_rotate_timer);                             //确保运行
	}
}

//停止轮播定时器
static void alarm_rotate_timer_stop(void)
{
	if(g_alarm_rotate_timer != NULL) lv_timer_pause(g_alarm_rotate_timer);  //暂停即可
}

//按当前活动 screen 恢复编码器 focus 组（overlay 关闭时调用）
static void ui_encoder_group_restore_for_active_screen(void)
{
	lv_obj_t * scr = lv_screen_active();
	if(scr == NULL) return;
	if(scr == g_scr_home) {
		home_encoder_group_build();
		ui_set_encoder_group(g_group_home);
	}
	else {
		ui_set_encoder_group(NULL);
	}
}

//隐藏报警弹层并恢复底层 encoder 组
static void alarm_overlay_hide(void)
{
	if(g_alarm_overlay == NULL) return;                                    //未构建
	lv_obj_add_flag(g_alarm_overlay, LV_OBJ_FLAG_HIDDEN);                  //隐藏弹层
	alarm_rotate_timer_stop();                                             //停止轮播
	alarm_fault_logo_blink_stop();                                         //停止 logo 闪烁
	g_alarm_overlay_open = false;                                            //标记关闭
	ui_encoder_group_restore_for_active_screen();                          //恢复焦点组
	if(g_idle_timer != NULL && !ui_idle_screen_keeps_awake(lv_screen_active())) {
		ui_idle_reset();                                                   //恢复空闲计时
	}
}

//显示报警弹层：切 encoder 组、显示当前轮播项、启动 3 秒定时器
static void alarm_overlay_show(void)
{
	if(g_alarm_overlay == NULL) return;                                    //未构建
	if(g_alarm_rotate_count == 0u) return;                                 //无报警不显示

	lv_obj_remove_flag(g_alarm_overlay, LV_OBJ_FLAG_HIDDEN);               //显示弹层
	lv_obj_move_foreground(g_alarm_overlay);                               //压到 top layer 最前（光标之下）
	ui_set_encoder_group(NULL);
	g_alarm_overlay_open = true;                                           //标记打开
	alarm_rotate_show_current();                                           //显示轮播首项/当前项
	alarm_rotate_timer_start();                                            //启动 3 秒轮播

	if(g_idle_timer != NULL) {                                             //报警期间保持常亮
		lv_timer_pause(g_idle_timer);
		g_idle_last_ptr_x = -1;
		g_idle_last_ptr_y = -1;
	}
}

//报警顶栏返回：用户暂时关闭弹层（全部清除后可再次自动弹出）
static void cb_alarm_back(lv_event_t * e)
{
	(void)e;                                                               //未使用
	g_alarm_user_dismissed = true;                                           //用户主动关闭
	alarm_overlay_hide();                                                  //隐藏弹层
}

//报警顶栏启停：复用 FSM 启停逻辑
static void cb_alarm_runpause(lv_event_t * e)
{
	(void)e;                                                               //未使用
	ui_fsm_runpause_apply(false);                                          //短按启停
}

//报警顶栏电源：复用 FSM 电源逻辑
static void cb_alarm_power(lv_event_t * e)
{
	(void)e;                                                               //未使用
	cb_power_long(e);                                                      //与长按电源相同逻辑
}

//每帧：边沿检测 → 重建队列 → 控制弹层显隐与轮播刷新
static void ui_alarm_poll(void)
{
	ui_alarm_trigger_sync();                                               //同步边沿
	bool queue_changed = ui_alarm_rotate_rebuild();                        //重建轮播队列

	if(g_alarm_rotate_count == 0u) {                                       //全部报警消失
		if(ui_alarm_overlay_is_visible()) alarm_overlay_hide();            //自动隐藏弹层
		return;
	}

	if(g_alarm_user_dismissed) return;                                     //用户已手动关闭

	if(!ui_alarm_overlay_is_visible()) {                                   //需要弹出
		alarm_overlay_show();                                              //显示弹层
		return;
	}

	if(queue_changed) {                                                    //队列变化
		if(g_alarm_rotate_cur_panel != ALARM_PANEL_NONE) {                 //当前项被删时 rebuild 已调 idx
			uint8_t found = 0xFFu;
			for(uint8_t i = 0; i < g_alarm_rotate_count; i++) {
				if(g_alarm_rotate_list[i] == g_alarm_rotate_cur_panel) { found = i; break; }
			}
			if(found == 0xFFu) {                                           //当前显示项已消失
				alarm_rotate_show_current();                               //立即切下一项
				alarm_rotate_timer_start();                                //重置 3 秒
			}
		}
	}
}

//构建单个故障码子面板（E1..E14：logo + (Ex)标题居中；说明与页脚左对齐块）
static void build_alarm_fault_panel(lv_obj_t * root, uint8_t fault_idx)
{
	ui_alarm_fault_ui_t * ui = &g_alarm_fault_ui[fault_idx];
	const lv_coord_t body_y = (lv_coord_t)(UI_FIXED_H * 15 / 100);
	char code_buf[16];

	ui->panel = lv_obj_create(root);
	lv_obj_set_size(ui->panel, LV_PCT(100), (lv_coord_t)UI_FIXED_H - 200);
	lv_obj_align(ui->panel, LV_ALIGN_TOP_MID, 0, body_y);
	lv_obj_set_style_bg_opa(ui->panel, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(ui->panel, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(ui->panel, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_row(ui->panel, 28, LV_PART_MAIN);
	lv_obj_set_flex_flow(ui->panel, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(ui->panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
	lv_obj_add_flag(ui->panel, LV_OBJ_FLAG_HIDDEN);

	/* 固定占位槽：闪烁时只隐藏内部图，槽尺寸不变，下方文字位置不动 */
	ui->logo_slot = lv_obj_create(ui->panel);
	lv_obj_set_size(ui->logo_slot, (lv_coord_t)fault_logo.header.w, (lv_coord_t)fault_logo.header.h);
	lv_obj_set_style_bg_opa(ui->logo_slot, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(ui->logo_slot, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(ui->logo_slot, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(ui->logo_slot, LV_LAYOUT_NONE, LV_PART_MAIN);
	lv_obj_remove_flag(ui->logo_slot, LV_OBJ_FLAG_SCROLLABLE);

	ui->img_logo = lv_image_create(ui->logo_slot);
	lv_image_set_src(ui->img_logo, &fault_logo);
	lv_obj_center(ui->img_logo);

	ui->title_row = lv_obj_create(ui->panel);
	lv_obj_set_size(ui->title_row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_set_style_bg_opa(ui->title_row, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(ui->title_row, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(ui->title_row, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_column(ui->title_row, 12, LV_PART_MAIN);
	lv_obj_set_flex_flow(ui->title_row, LV_FLEX_FLOW_ROW);
	lv_obj_set_flex_align(ui->title_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

	ui->lbl_code = lv_label_create(ui->title_row);
	snprintf(code_buf, sizeof(code_buf), "(%s)", g_alarm_fault_codes[fault_idx]);
	lv_label_set_text(ui->lbl_code, code_buf);
	lv_obj_set_style_text_color(ui->lbl_code, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(ui->lbl_code, s_font_sc_30);

	ui->lbl_title = lv_label_create(ui->title_row);
	ui_lang_bind_label(ui->lbl_title, alarm_fault_str_id(fault_idx, 0));
	lv_obj_set_style_text_color(ui->lbl_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(ui->lbl_title, s_font_sc_30);

	/* 说明+页脚：块整体居中，内部左对齐 */
	ui->text_block = lv_obj_create(ui->panel);
	lv_obj_set_size(ui->text_block, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_set_style_bg_opa(ui->text_block, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(ui->text_block, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(ui->text_block, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_row(ui->text_block, 12, LV_PART_MAIN);
	lv_obj_set_flex_flow(ui->text_block, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(ui->text_block, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

	ui->lbl_line1 = lv_label_create(ui->text_block);
	ui_lang_bind_label(ui->lbl_line1, alarm_fault_str_id(fault_idx, 1));
	lv_obj_set_style_text_color(ui->lbl_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_set_style_text_align(ui->lbl_line1, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
	ui_set_obj_font(ui->lbl_line1, s_font_sc_30);

	ui->lbl_line2 = lv_label_create(ui->text_block);
	ui_lang_bind_label(ui->lbl_line2, alarm_fault_str_id(fault_idx, 2));
	lv_obj_set_style_text_color(ui->lbl_line2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_set_style_text_align(ui->lbl_line2, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
	ui_set_obj_font(ui->lbl_line2, s_font_sc_30);

	ui->call_row = lv_obj_create(ui->text_block);
	lv_obj_set_size(ui->call_row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_set_style_bg_opa(ui->call_row, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(ui->call_row, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(ui->call_row, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_column(ui->call_row, 0, LV_PART_MAIN);
	lv_obj_set_flex_flow(ui->call_row, LV_FLEX_FLOW_ROW);
	lv_obj_set_flex_align(ui->call_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

	ui->lbl_line3 = lv_label_create(ui->call_row);
	ui_lang_bind_label(ui->lbl_line3, STR_ALARM_FAULT_CALL);
	lv_obj_set_style_text_color(ui->lbl_line3, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(ui->lbl_line3, s_font_sc_30);

	ui->lbl_phone = lv_label_create(ui->call_row);
	ui_lang_bind_label(ui->lbl_phone, STR_ALARM_FAULT_PHONE);
	lv_obj_set_style_text_color(ui->lbl_phone, lv_color_hex(COL_ALARM_PHONE), LV_PART_MAIN);
	ui_set_obj_font(ui->lbl_phone, s_font_sc_30);

	ui->lbl_line4 = lv_label_create(ui->text_block);
	ui_lang_bind_label(ui->lbl_line4, STR_ALARM_FAULT_SERVICE);
	lv_obj_set_style_text_color(ui->lbl_line4, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_set_style_text_align(ui->lbl_line4, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
	ui_set_obj_font(ui->lbl_line4, s_font_sc_30);

	alarm_fault_panel_relayout_content(fault_idx);
}

//故障说明是否为空（part 1=LINE1，2=LINE2）
static bool alarm_fault_line_empty(uint8_t fault_idx, uint8_t part)
{
	const char * text = ui_translation(alarm_fault_str_id(fault_idx, part));
	if(text == NULL || text[0] == '\0') return true;
	for(const char * p = text; *p != '\0'; p++) {
		if(*p != ' ' && *p != '\t' && *p != '\n' && *p != '\r') return false;
	}
	return true;
}

//说明显隐（页脚由 flex 自动跟排；空说明行隐藏）
static void alarm_fault_panel_relayout_content(uint8_t fault_idx)
{
	ui_alarm_fault_ui_t * ui = &g_alarm_fault_ui[fault_idx];
	if(ui->panel == NULL || ui->lbl_line1 == NULL || ui->lbl_line2 == NULL) return;

	if(alarm_fault_line_empty(fault_idx, 1)) {
		lv_obj_add_flag(ui->lbl_line1, LV_OBJ_FLAG_HIDDEN);
	}
	else {
		lv_obj_remove_flag(ui->lbl_line1, LV_OBJ_FLAG_HIDDEN);
	}
	if(alarm_fault_line_empty(fault_idx, 2)) {
		lv_obj_add_flag(ui->lbl_line2, LV_OBJ_FLAG_HIDDEN);
	}
	else {
		lv_obj_remove_flag(ui->lbl_line2, LV_OBJ_FLAG_HIDDEN);
	}
}

//语言切换后，全部故障子页重新排内容
static void alarm_fault_panels_relayout_content(void)
{
	for(uint8_t i = 0; i < UI_ALARM_FAULT_COUNT; i++) {                    //E1..E14
		alarm_fault_panel_relayout_content(i);
	}
}

//构建 lv_layer_top() 报警全屏弹层：顶栏 + E1..E14 + 缺液 + 底栏
static void build_alarm_overlay(void)
{
	lv_obj_t * layer = lv_layer_top();                                     //全局顶层
	g_alarm_overlay = lv_obj_create(layer);                                //全屏弹层根
	lv_obj_set_size(g_alarm_overlay, UI_FIXED_W, UI_FIXED_H);              //铺满屏幕
	lv_obj_set_pos(g_alarm_overlay, 0, 0);                                 //左上角
	lv_obj_set_style_bg_color(g_alarm_overlay, lv_color_hex(COL_BG), LV_PART_MAIN); //黑底
	lv_obj_set_style_bg_opa(g_alarm_overlay, LV_OPA_COVER, LV_PART_MAIN);  //不透明
	lv_obj_set_style_border_width(g_alarm_overlay, 0, LV_PART_MAIN);       //无边框
	lv_obj_set_style_pad_all(g_alarm_overlay, 0, LV_PART_MAIN);            //无内边距
	lv_obj_set_style_radius(g_alarm_overlay, 0, LV_PART_MAIN);             //无圆角
	lv_obj_set_style_layout(g_alarm_overlay, LV_LAYOUT_NONE, LV_PART_MAIN);//绝对布局
	lv_obj_add_flag(g_alarm_overlay, LV_OBJ_FLAG_HIDDEN);                  //默认隐藏
	lv_obj_add_flag(g_alarm_overlay, LV_OBJ_FLAG_CLICKABLE);               //拦截点击

	lv_obj_t * top = create_top_bar(g_alarm_overlay, &g_lbl_clock_alarm, NULL, NULL); //顶栏+状态栏

	g_alarm_btn_back = make_top_back_btn(top, cb_alarm_back);

	g_alarm_btn_runpause = add_top_text_btn(top, "启停", 100); //启停
	lv_obj_add_event_cb(g_alarm_btn_runpause, cb_alarm_runpause, LV_EVENT_CLICKED, NULL);

	g_alarm_btn_power = add_top_text_btn(top, "电源", 180);    //电源
	lv_obj_add_event_cb(g_alarm_btn_power, cb_alarm_power, LV_EVENT_CLICKED, NULL);

	g_alarm_img_bar = lv_image_create(g_alarm_overlay);                      //底栏 bar_01
	lv_image_set_src(g_alarm_img_bar, &bar_01);
	lv_obj_align(g_alarm_img_bar, LV_ALIGN_BOTTOM_MID, 0, -60);

	for(uint8_t i = 0; i < UI_ALARM_FAULT_COUNT; i++) {                     //E1..E14 子面板
		build_alarm_fault_panel(g_alarm_overlay, i);
	}

	g_alarm_panel_fluid = lv_obj_create(g_alarm_overlay);                  //缺液子面板（独立布局，不受故障页偏移影响）
	lv_obj_set_size(g_alarm_panel_fluid, LV_PCT(100), (lv_coord_t)UI_FIXED_H - 120);
	lv_obj_align(g_alarm_panel_fluid, LV_ALIGN_TOP_MID, 0, (lv_coord_t)(UI_FIXED_H * 10 / 100));
	lv_obj_set_style_bg_opa(g_alarm_panel_fluid, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(g_alarm_panel_fluid, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(g_alarm_panel_fluid, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_row(g_alarm_panel_fluid, 20, LV_PART_MAIN);       //图与文字间距
	lv_obj_set_flex_flow(g_alarm_panel_fluid, LV_FLEX_FLOW_COLUMN);        //纵向排列
	lv_obj_set_flex_align(g_alarm_panel_fluid, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); //居中
	lv_obj_add_flag(g_alarm_panel_fluid, LV_OBJ_FLAG_HIDDEN);              //默认隐藏

	g_alarm_img_warn = lv_image_create(g_alarm_panel_fluid);                 //缺液提示图
	g_alarm_lbl_hint = lv_label_create(g_alarm_panel_fluid);               //缺液提示文字
	lv_obj_set_width(g_alarm_lbl_hint, LV_PCT(100));
	lv_obj_set_style_text_color(g_alarm_lbl_hint, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_set_style_text_align(g_alarm_lbl_hint, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	ui_set_obj_font(g_alarm_lbl_hint, s_font_sc_30);
}











/* ---------- 程序设置：表3.1 初值、与主页/运行页参数同步 ---------- */

static const char PROG_ADMIN_ADD_TIME_ROLLER_OPTS[] =
	"0\n10\n20\n30\n40\n50\n60\n70\n80\n90\n100\n110\n120\n130\n140\n150\n160\n170\n180";

static char g_program_profile_temp_buf[TOTAL_PROGRAMS][8];

/* 按程序类型与温度档位索引取烘干温度（℃） */
static int16_t program_admin_temp_celsius(int32_t prog_idx, int8_t temp_idx)
{
	static const int16_t low_tbl[] = { 36, 40, 44 };
	static const int16_t mid_tbl[] = { 46, 50, 54 };
	static const int16_t high_tbl[] = { 56, 60, 64 };

	prog_idx = wheel_mod_total(prog_idx);
	if(temp_idx < 0 || temp_idx > 2) return 0;
	switch(prog_idx) {
	case 0:
		return low_tbl[temp_idx];
	case 1:
		return mid_tbl[temp_idx];
	case 2:
		return high_tbl[temp_idx];
	default:
		return 0;
	}
}

/* 切换程序 Tab 时刷新烘干温度 roller 选项（36/40/44 等） */
static void prog_ui_temp_roller_apply(prog_ui_ctx_t * pui, int32_t prog_idx)
{
	if(pui == NULL || pui->roller[0] == NULL) return;
	prog_idx = wheel_mod_total(prog_idx);
	const char * opts = NULL;
	switch(prog_idx) {
	case 0: opts = "36℃\n40℃\n44℃"; break;
	case 1: opts = "46℃\n50℃\n54℃"; break;
	case 2: opts = "56℃\n60℃\n64℃"; break;
	default: return;
	}
	uint32_t sel = 0;
	if(pui->sel >= 0 && pui->sel < TOTAL_PROGRAMS && pui->cfg_tbl != NULL) {
		const ui_program_admin_t * c = &pui->cfg_tbl[pui->sel];
		if(c->temp_idx >= 0 && c->temp_idx <= 2) sel = (uint32_t)c->temp_idx;
	}
	lv_roller_set_options(pui->roller[0], opts, LV_ROLLER_MODE_NORMAL);
	lv_roller_set_selected(pui->roller[0], sel, LV_ANIM_OFF);
}

static void program_admin_temp_roller_apply(int32_t prog_idx)
{
	prog_ui_temp_roller_apply(&g_pui_admin, prog_idx);
}

/* 追加时间（秒/分钟）→ roller 选中项（步进 10） */
static uint32_t program_admin_add_time_to_roller(uint16_t add_time_min)
{
	if(add_time_min > 180u) add_time_min = 180u;
	return (uint32_t)(add_time_min / 10u);
}

/* roller 选中项 → 追加时间（步进 10） */
static uint16_t program_admin_roller_to_add_time(uint32_t sel)
{
	if(sel > 18u) sel = 18u;
	return (uint16_t)(sel * 10u);
}

/* 计算程序基础总时间（秒）：初始烘干 + 冷却；不含追加×次数（追加次数仅为追加页上限） */
static uint32_t program_admin_total_sec(const ui_program_admin_t * c)
{
	uint32_t total = 0;

	if(c->cap & PROG_CAP_INIT_DRY) total += c->init_dry_min;
	if(c->cap & PROG_CAP_COOL) total += c->cool_min;
	// if((c->cap & PROG_CAP_ADD_COUNT) && (c->cap & PROG_CAP_ADD_TIME)) {
	// 	total += (uint32_t)c->add_count * (uint32_t)c->add_time_min;
	// }
#if !PROG_ADMIN_DEMO_SEC
	total *= 60u;
#endif
	return total;
}

/* 刷新冷却时间只读标签（分钟模式为 Nmin，演示秒模式为 Ns） */
static void prog_ui_update_cool_display(prog_ui_ctx_t * pui, const ui_program_admin_t * c)
{
	if(pui == NULL || pui->cool_lbl == NULL || c == NULL) return;
	char buf[16];
#if PROG_ADMIN_DEMO_SEC
	lv_snprintf(buf, sizeof(buf), "%us", (unsigned)c->cool_min);
#else
	lv_snprintf(buf, sizeof(buf), "%umin", (unsigned)c->cool_min);
#endif
	lv_label_set_text(pui->cool_lbl, buf);
}

static void program_admin_update_cool_display(const ui_program_admin_t * c)
{
	prog_ui_update_cool_display(&g_pui_admin, c);
}

/* 写入表3.1 程序初值；程序金额：风自洁 1 元，其余 6 元；时间为分钟 */
static void program_admin_init_factory(void)
{
	static const ui_program_admin_t factory[TOTAL_PROGRAMS] = {
		/* 0 低温：6元 + 追加1元 + 18min + 40℃ + 2min冷却 + 追加7×10min */
		{ 6, 1, 18, 7, 10, 1, 2, PROG_CAP_DRY_FULL },
		/* 1 中温：6元 + 追加1元 + 18min + 50℃ + 2min + 7×10min */
		{ 6, 1, 18, 7, 10, 1, 2, PROG_CAP_DRY_FULL },
		/* 2 高温：6元 + 追加1元 + 18min + 60℃ + 2min + 7×10min */
		{ 6, 1, 18, 7, 10, 1, 2, PROG_CAP_DRY_FULL },
		/* 3 冷风：6元 + 追加1元 + 10min冷却 + 7×10min */
		{ 6, 1, 0, 7, 10, -1, 10, PROG_CAP_COLD_AIR },
		/* 4 风自洁：1元 + 2min冷却，无追加（追加金额 --） */
		{ 1, 0, 0, 0, 0, -1, 2, PROG_CAP_AIR_CLEAN },
	};
	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		g_prog_cfg_factory[i] = factory[i];
		g_prog_cfg[i] = factory[i];
	}
	program_admin_apply_all();
}

/* 将 g_prog_cfg[idx] 同步到主页/运行页 profile 与 stages */
static void program_admin_apply_one(int32_t idx)
{
	idx = wheel_mod_total(idx);
	const ui_program_admin_t * c = &g_prog_cfg[idx];
	const uint32_t total_sec = program_admin_total_sec(c);

	g_program_profiles[idx].price = c->price;
	if(c->cap & PROG_CAP_TEMP) {
		lv_snprintf(g_program_profile_temp_buf[idx], sizeof(g_program_profile_temp_buf[idx]),
			"%d℃", (int)program_admin_temp_celsius(idx, c->temp_idx));
		g_program_profiles[idx].temp = g_program_profile_temp_buf[idx];
	} else {
		g_program_profiles[idx].temp = "--";
	}
	g_program_profiles[idx].wash_sec = total_sec;

	uint32_t dry_sec = 0;
	if(c->cap & PROG_CAP_INIT_DRY) dry_sec += c->init_dry_min;
	// 主页/运行 profile 基础值：不含追加段（用户在追加时间页选定后再写入会话）
	uint32_t cool_sec = (c->cap & PROG_CAP_COOL) ? (uint32_t)c->cool_min : 0u;
#if !PROG_ADMIN_DEMO_SEC
	dry_sec *= 60u;
	cool_sec *= 60u;
#endif
	g_program_run_stages[idx].wash_sec = dry_sec;
	g_program_run_stages[idx].rinse_sec = 0;
	g_program_run_stages[idx].spin_sec = cool_sec;
	if(dry_sec > 0u && cool_sec > 0u) {
		g_program_run_stages[idx].stages_bar_id = STR_RUN_STAGES;
	} else if(cool_sec > 0u) {
		g_program_run_stages[idx].stages_bar_id = STR_RUN_STAGES_SPIN_ONLY;
	} else {
		g_program_run_stages[idx].stages_bar_id = STR_RUN_STAGES_RINSE_SPIN;
	}
}

/* 将全部程序 cfg 同步到主页/运行页 */
static void program_admin_apply_all(void)
{
	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		program_admin_apply_one(i);
	}
}

/* 程序设置参数范围钳位（金额 0-999，烘干时间 0-90 分钟，追加时间 0-180 分钟） */
static void program_admin_clamp_cfg(ui_program_admin_t * c)
{
	if(c->price > 999) c->price = 999;
	if(c->add_price > 999) c->add_price = 999;
	if(c->init_dry_min > 90) c->init_dry_min = 90;
	if(c->add_count > 20) c->add_count = 20;
	c->add_time_min = (uint16_t)((c->add_time_min / 10u) * 10u);
	if(c->add_time_min > 180) c->add_time_min = 180;
	if(c->cap & PROG_CAP_TEMP) {
		if(c->temp_idx < 0) c->temp_idx = 0;
		if(c->temp_idx > 2) c->temp_idx = 2;
	}
}

/* 从程序设置 UI 控件写回 cfg 表（管理员 / 循环程序共用） */
static void prog_ui_save_fields(prog_ui_ctx_t * pui)
{
	if(pui == NULL || pui->cfg_tbl == NULL) return;
	if(pui->ui_loading) return;
	if(pui->sel < 0 || pui->sel >= TOTAL_PROGRAMS) return;
	ui_program_admin_t * c = &pui->cfg_tbl[pui->sel];

	if(pui->ta[0] != NULL && (c->cap & PROG_CAP_INIT_DRY)) {
		const char * t = lv_textarea_get_text(pui->ta[0]);
		if(t != NULL && t[0] != '\0') c->init_dry_min = (uint16_t)atoi(t);
	}
	if(pui->ta[1] != NULL && (c->cap & PROG_CAP_ADD_COUNT)) {
		const char * t = lv_textarea_get_text(pui->ta[1]);
		if(t != NULL && t[0] != '\0') c->add_count = (uint8_t)atoi(t);
	}
	if(pui->ta[2] != NULL && (c->cap & PROG_CAP_PRICE)) {
		const char * t = lv_textarea_get_text(pui->ta[2]);
		if(t != NULL && t[0] != '\0') c->price = (int32_t)atoi(t);
	}
	if(pui->ta[3] != NULL && (c->cap & PROG_CAP_ADD_PRICE)) {
		const char * t = lv_textarea_get_text(pui->ta[3]);
		if(t != NULL && t[0] != '\0') c->add_price = (int32_t)atoi(t);
	}
	if(pui->roller[0] != NULL && (c->cap & PROG_CAP_TEMP)) {
		c->temp_idx = (int8_t)lv_roller_get_selected(pui->roller[0]);
	}
	if(pui->roller[1] != NULL && (c->cap & PROG_CAP_ADD_TIME)) {
		c->add_time_min = program_admin_roller_to_add_time(lv_roller_get_selected(pui->roller[1]));
	}
	program_admin_clamp_cfg(c);
}

/* 从程序设置 UI 控件写回 g_prog_cfg[当前程序]（时间字段为分钟） */
static void program_admin_ui_save_fields(void)
{
	prog_ui_save_fields(&g_pui_admin);
}

/* 将 cfg 表当前程序加载到程序设置 UI 控件 */
static void prog_ui_load_fields(prog_ui_ctx_t * pui)
{
	if(pui == NULL || pui->cfg_tbl == NULL) return;
	if(pui->sel < 0 || pui->sel >= TOTAL_PROGRAMS) return;
	const ui_program_admin_t * c = &pui->cfg_tbl[pui->sel];
	char buf[16];

	pui->ui_loading = true;

	prog_ui_temp_roller_apply(pui, pui->sel);

	if(pui->ta[0] != NULL && (c->cap & PROG_CAP_INIT_DRY)) {
		lv_snprintf(buf, sizeof(buf), "%u", (unsigned)c->init_dry_min);
		lv_textarea_set_text(pui->ta[0], buf);
	}
	if(pui->ta[1] != NULL && (c->cap & PROG_CAP_ADD_COUNT)) {
		lv_snprintf(buf, sizeof(buf), "%u", (unsigned)c->add_count);
		lv_textarea_set_text(pui->ta[1], buf);
	}
	if(pui->ta[2] != NULL && (c->cap & PROG_CAP_PRICE)) {
		if(c->price < 0) lv_textarea_set_text(pui->ta[2], "");
		else lv_snprintf(buf, sizeof(buf), "%d", (int)c->price), lv_textarea_set_text(pui->ta[2], buf);
	}
	if(pui->ta[3] != NULL && (c->cap & PROG_CAP_ADD_PRICE)) {
		if(c->add_price < 0) lv_textarea_set_text(pui->ta[3], "");
		else lv_snprintf(buf, sizeof(buf), "%d", (int)c->add_price), lv_textarea_set_text(pui->ta[3], buf);
	}
	if(pui->roller[0] != NULL && (c->cap & PROG_CAP_TEMP) && c->temp_idx >= 0) {
		lv_roller_set_selected(pui->roller[0], (uint32_t)c->temp_idx, LV_ANIM_OFF);
	}
	if(pui->roller[1] != NULL && (c->cap & PROG_CAP_ADD_TIME)) {
		lv_roller_set_selected(pui->roller[1],
			program_admin_add_time_to_roller(c->add_time_min), LV_ANIM_OFF);
	}
	prog_ui_update_cool_display(pui, c);

	pui->ui_loading = false;

	prog_ui_apply_caps(pui);
	prog_ui_sync_prog_pick_ui(pui);
}

/* 将 g_prog_cfg[当前程序] 加载到程序设置 UI 控件 */
static void program_admin_ui_load_fields(void)
{
	g_pui_admin.sel = g_admin_prog_sel;
	prog_ui_load_fields(&g_pui_admin);
	if(g_admin_view == PROGRAM_SETTINGS) {
		admin_encoder_rebuild();
	}
}

/* 初始化程序设置上栏程序 Tab 按钮的普通/选中样式 */
static void admin_prog_btn_style_init(void)
{
	if(s_admin_prog_btn_style_inited) return;
	lv_style_init(&s_admin_prog_btn_style);
	lv_style_set_radius(&s_admin_prog_btn_style, 8);
	lv_style_set_bg_opa(&s_admin_prog_btn_style, LV_OPA_COVER);
	lv_style_set_bg_color(&s_admin_prog_btn_style, lv_color_hex(0x0a2a5a));
	lv_style_set_bg_grad_color(&s_admin_prog_btn_style, lv_color_hex(0x000810));
	lv_style_set_bg_grad_dir(&s_admin_prog_btn_style, LV_GRAD_DIR_VER);
	lv_style_set_border_width(&s_admin_prog_btn_style, 2);
	lv_style_set_border_color(&s_admin_prog_btn_style, lv_color_hex(0x2a7fff));
	lv_style_set_border_opa(&s_admin_prog_btn_style, LV_OPA_COVER);

	lv_style_init(&s_admin_prog_btn_sel_style);
	lv_style_set_radius(&s_admin_prog_btn_sel_style, 8);
	lv_style_set_bg_opa(&s_admin_prog_btn_sel_style, LV_OPA_COVER);
	lv_style_set_bg_color(&s_admin_prog_btn_sel_style, lv_color_hex(COL_ORANGE));
	lv_style_set_bg_grad_color(&s_admin_prog_btn_sel_style, lv_color_hex(0x994400));
	lv_style_set_bg_grad_dir(&s_admin_prog_btn_sel_style, LV_GRAD_DIR_VER);
	lv_style_set_border_width(&s_admin_prog_btn_sel_style, 2);
	lv_style_set_border_color(&s_admin_prog_btn_sel_style, lv_color_hex(0xFFD4A8));
	lv_style_set_border_opa(&s_admin_prog_btn_sel_style, LV_OPA_COVER);
	s_admin_prog_btn_style_inited = true;
}

/* 刷新程序设置上栏总时长标签（Nmin）；循环程序含追加次数×追加时间 */
static void prog_ui_update_total_display(prog_ui_ctx_t * pui)
{
	if(pui == NULL || pui->total_val_lbl == NULL || pui->cfg_tbl == NULL) return;
	if(pui->sel < 0 || pui->sel >= TOTAL_PROGRAMS) return;

	const ui_program_admin_t * c = &pui->cfg_tbl[pui->sel];
	uint32_t total;
	if(pui == &g_pui_cycle) {
		uint8_t add_sel = 0u;
		if((c->cap & PROG_CAP_ADD_COUNT) && (c->cap & PROG_CAP_ADD_TIME)) {
			add_sel = c->add_count;
		}
		total = add_time_calc_total_sec(c, add_sel);
	} else {
		total = program_admin_total_sec(c);
	}

	char buf[16];
	program_format_time_label(total, buf, sizeof(buf));
	lv_label_set_text(pui->total_val_lbl, buf);
}

static void program_admin_update_total_display(void)
{
	prog_ui_update_total_display(&g_pui_admin);
}

/* 刷新程序选择上栏按钮选中样式，并更新总时长显示 */
static void prog_ui_sync_prog_pick_ui(prog_ui_ctx_t * pui)
{
	if(pui == NULL) return;
	admin_prog_btn_style_init();
	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		lv_obj_t * b = pui->btns[i];
		if(b == NULL) continue;
		if(i == pui->sel) {
			lv_obj_remove_style(b, &s_admin_prog_btn_style, LV_PART_MAIN);
			lv_obj_add_style(b, &s_admin_prog_btn_sel_style, LV_PART_MAIN);
		} else {
			lv_obj_remove_style(b, &s_admin_prog_btn_sel_style, LV_PART_MAIN);
			lv_obj_add_style(b, &s_admin_prog_btn_style, LV_PART_MAIN);
		}
	}
	prog_ui_update_total_display(pui);
}

static void program_admin_sync_prog_pick_ui(void)
{
	prog_ui_sync_prog_pick_ui(&g_pui_admin);
}

/* 程序设置参数字段变更：写回 cfg 并刷新总时长 */
static void cb_admin_prog_time_field_changed(lv_event_t * e)
{
	(void)e;
	if(g_pui_admin.ui_loading) return;
	if(g_admin_view != PROGRAM_SETTINGS) return;
	program_admin_ui_save_fields();
	program_admin_update_total_display();
}

/* 根据当前程序 cfg 更新 UI 参数开关（烘干温度/烘干时间/冷却时间等） */
static void prog_ui_apply_caps(prog_ui_ctx_t * pui)
{
	if(pui == NULL || pui->cfg_tbl == NULL) return;
	if(pui->sel < 0 || pui->sel >= TOTAL_PROGRAMS) return;
	const ui_program_admin_t * c = &pui->cfg_tbl[pui->sel];
	static const uint16_t cap_map[PROG_ADMIN_FIELD_CNT] = {
		PROG_CAP_TEMP, PROG_CAP_INIT_DRY, PROG_CAP_COOL,
		PROG_CAP_ADD_COUNT, PROG_CAP_ADD_TIME, PROG_CAP_PRICE, PROG_CAP_ADD_PRICE
	};
	lv_obj_t * widgets[PROG_ADMIN_FIELD_CNT] = {
		pui->roller[0], pui->ta[0], pui->cool_lbl,
		pui->ta[1], pui->roller[1], pui->ta[2], pui->ta[3]
	};
	const int field_cnt = (pui == &g_pui_cycle) ? (int)CYCLE_PROG_FIELD_CNT : PROG_ADMIN_FIELD_CNT;
	for(int i = 0; i < field_cnt; i++) {
		bool en = (c->cap & cap_map[i]) != 0;
		if(i == 2 && (c->cap & PROG_CAP_COOL)) {
			en = true;
			prog_ui_update_cool_display(pui, c);
		}
		if(widgets[i] != NULL) {
			if(en) lv_obj_remove_flag(widgets[i], LV_OBJ_FLAG_HIDDEN);
			else lv_obj_add_flag(widgets[i], LV_OBJ_FLAG_HIDDEN);
		}
		if(pui->dash[i] != NULL) {
			if(en) lv_obj_add_flag(pui->dash[i], LV_OBJ_FLAG_HIDDEN);
			else lv_obj_remove_flag(pui->dash[i], LV_OBJ_FLAG_HIDDEN);
		}
	}
}

static void program_admin_ui_apply_caps(void)
{
	prog_ui_apply_caps(&g_pui_admin);
}

/* 创建程序设置上栏程序 Tab 按钮（带 SC_30 标签） */
static lv_obj_t * make_admin_prog_btn(lv_obj_t * parent, const char * txt)
{
	admin_prog_btn_style_init();
	lv_obj_t * b = lv_button_create(parent);
	lv_obj_add_style(b, &s_admin_prog_btn_style, LV_PART_MAIN);
	lv_obj_set_style_shadow_width(b, 0, LV_PART_MAIN);
	lv_obj_t * l = lv_label_create(b);
	lv_label_set_text(l, txt);
	lv_obj_set_style_text_color(l, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(l, s_font_sc_30);
	lv_obj_center(l);
	return b;
}

/* 创建程序设置页：白底外框（文本框容器），Flex 居中子控件 */
static lv_obj_t * program_admin_make_value_box(lv_obj_t * parent, lv_coord_t w, lv_coord_t h)
{
	lv_obj_t * box = lv_obj_create(parent);
	lv_obj_set_size(box, w, h);
	lv_obj_set_style_bg_color(box, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
	lv_obj_set_style_bg_opa(box, LV_OPA_COVER, LV_PART_MAIN);
	lv_obj_set_style_radius(box, 6, LV_PART_MAIN);
	lv_obj_set_style_border_width(box, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(box, 0, LV_PART_MAIN);
	lv_obj_remove_flag(box, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_flex_flow(box, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(box, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
	return box;
}

/* 程序设置页：外框内 textarea/roller 透明、水平铺满，高度随内容（由 Flex 垂直居中） */
static void program_admin_style_field_inner(lv_obj_t * obj)
{
	lv_obj_set_width(obj, LV_PCT(100));
	lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);
	lv_obj_set_style_radius(obj, 0, LV_PART_MAIN);
	lv_obj_set_style_shadow_width(obj, 0, LV_PART_MAIN);
	lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
}

/* 单行 textarea：宽度铺满外框，高度随内容 */
static void program_admin_style_ta(lv_obj_t * ta)
{
	program_admin_style_field_inner(ta);
	lv_textarea_set_one_line(ta, true);
}

static void program_admin_ta_close_kb(void)
{
	admin_kb_close();
}

static void program_admin_ta_begin_edit(lv_obj_t * ta)
{
	if(g_admin_kb == NULL || g_admin_view != PROGRAM_SETTINGS) return;
	if(ta == NULL || lv_obj_has_flag(ta, LV_OBJ_FLAG_HIDDEN)) return;
	g_admin_ta_prog_active = ta;
	g_admin_kb_ta = ta;
	lv_obj_remove_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
	admin_kb_encoder_enter();
}

/* 程序设置 textarea 获焦/点击：弹出数字键盘进入编辑 */
static void cb_admin_prog_ta_focus(lv_event_t * e)
{
	lv_obj_t * ta = lv_event_get_target_obj(e);
	lv_event_code_t code = lv_event_get_code(e);
	if(g_admin_view != PROGRAM_SETTINGS) return;

	if(code == LV_EVENT_FOCUSED) {
		/* 编码器：进入 group 编辑模式后再弹键盘；不因失焦自动关闭 */
		if(g_group_admin != NULL && (g_group_admin && lv_group_get_editing(g_group_admin))) {
			program_admin_ta_begin_edit(ta);
		}
	} else if(code == LV_EVENT_CLICKED) {
		/* 触摸：点击文本框后进入编辑 */
		program_admin_ta_begin_edit(ta);
		if(g_group_admin != NULL) {
			if(g_group_admin) lv_group_set_editing(g_group_admin, true);
		}
	}
}

static void program_admin_show_list(void)
{
	g_admin_prog_is_detail = false;
	if(g_admin_prog_list != NULL) lv_obj_remove_flag(g_admin_prog_list, LV_OBJ_FLAG_HIDDEN);
	if(g_admin_prog_detail != NULL) lv_obj_add_flag(g_admin_prog_detail, LV_OBJ_FLAG_HIDDEN);
	if(g_admin_img_prog_title_box != NULL) lv_obj_remove_flag(g_admin_img_prog_title_box, LV_OBJ_FLAG_HIDDEN);
	if(g_admin_lbl_prog_title != NULL) lv_obj_remove_flag(g_admin_lbl_prog_title, LV_OBJ_FLAG_HIDDEN);
	if(g_admin_kb != NULL) {
		g_admin_kb_ta = NULL;
		lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
	}
}

static void program_admin_show_detail(void)
{
	g_admin_prog_is_detail = true;
	if(g_admin_prog_list != NULL) lv_obj_add_flag(g_admin_prog_list, LV_OBJ_FLAG_HIDDEN);
	if(g_admin_prog_detail != NULL) lv_obj_remove_flag(g_admin_prog_detail, LV_OBJ_FLAG_HIDDEN);
	if(g_admin_img_prog_title_box != NULL) lv_obj_add_flag(g_admin_img_prog_title_box, LV_OBJ_FLAG_HIDDEN);
	if(g_admin_lbl_prog_title != NULL) lv_obj_add_flag(g_admin_lbl_prog_title, LV_OBJ_FLAG_HIDDEN);
	program_admin_ui_load_fields();
}

static void cb_admin_prog_detail_gesture(lv_event_t * e)
{
	if(lv_event_get_code(e) != LV_EVENT_GESTURE) return;
	if(g_admin_view != PROGRAM_SETTINGS || !g_admin_prog_is_detail) return;
	lv_indev_t * indev = lv_indev_active();
	if(indev == NULL) return;
	lv_dir_t dir = lv_indev_get_gesture_dir(indev);
	if(dir == LV_DIR_RIGHT) {
		lv_indev_wait_release(indev);
		program_admin_ui_save_fields();
		program_admin_show_list();
	}
}

/* 选择程序：进入详情（字段仍为烘干参数） */
static void cb_admin_prog_pick(lv_event_t * e)
{
	program_admin_ui_save_fields();
	g_admin_prog_sel = (int32_t)(intptr_t)lv_event_get_user_data(e);
	program_admin_show_detail();
}

/* 恢复当前程序为表3.1 出厂初值（风自洁 1 元，其余 6 元） */
static void cb_admin_prog_reset(lv_event_t * e)
{
	(void)e;
	if(g_admin_prog_sel < 0 || g_admin_prog_sel >= TOTAL_PROGRAMS) return;
	g_prog_cfg[g_admin_prog_sel] = g_prog_cfg_factory[g_admin_prog_sel];
	program_admin_ui_load_fields();
}

/* 确认程序设置：同步全部 cfg 到主页/运行页并返回管理员菜单 */
static void cb_admin_prog_confirm(lv_event_t * e)
{
	(void)e;
	program_admin_ui_save_fields();
	program_admin_apply_all();
	home_sync_program_labels();
	pay_sync_price_label();
	program_admin_show_list();
}

/* 从管理员菜单进入程序设置页 */
static void cb_admin_open_program_settings(lv_event_t * e)
{
	(void)e;
	g_admin_prog_sel = 0;
	admin_panel_show(PROGRAM_SETTINGS);
}

/* 离开程序设置页：保存 cfg、关闭键盘并返回管理员菜单 */
static void program_admin_back_to_menu1(void)
{
	program_admin_ui_save_fields();
	if(g_group_admin != NULL) {
		if(g_group_admin) lv_group_set_editing(g_group_admin, false);
	}
	program_admin_ta_close_kb();
	if(g_admin_prog_is_detail) {
		program_admin_show_list();
		return;
	}
	admin_panel_show(MENU1);
}

/* ---------- 循环程序页：编码器 + 数字键盘（对齐管理员程序设置） ---------- */

static bool cycle_kb_is_visible(void)
{
	if(g_cycle_kb == NULL) return false;
	return !lv_obj_has_flag(g_cycle_kb, LV_OBJ_FLAG_HIDDEN);
}

/* 循环程序页：数字键盘编码器样式初始化 */
static void cycle_kb_encoder_style_init(void)
{
	if(g_cycle_kb == NULL || s_cycle_kb_encoder_inited) return;

	lv_display_t * disp = lv_obj_get_display(g_cycle_kb);
	lv_coord_t ow = 3;
	if(disp != NULL) ow = (lv_coord_t)lv_display_dpx(disp, 3);

	lv_obj_set_style_outline_color(g_cycle_kb, lv_color_hex(COL_TEXT), LV_PART_ITEMS | LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_width(g_cycle_kb, ow, LV_PART_ITEMS | LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_pad(g_cycle_kb, ow, LV_PART_ITEMS | LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_opa(g_cycle_kb, LV_OPA_COVER, LV_PART_ITEMS | LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_color(g_cycle_kb, lv_color_hex(COL_TEXT), LV_PART_ITEMS | LV_STATE_FOCUS_KEY | LV_STATE_CHECKED);
	lv_obj_set_style_outline_width(g_cycle_kb, ow, LV_PART_ITEMS | LV_STATE_FOCUS_KEY | LV_STATE_CHECKED);
	lv_obj_set_style_outline_pad(g_cycle_kb, ow, LV_PART_ITEMS | LV_STATE_FOCUS_KEY | LV_STATE_CHECKED);
	lv_obj_set_style_outline_opa(g_cycle_kb, LV_OPA_COVER, LV_PART_ITEMS | LV_STATE_FOCUS_KEY | LV_STATE_CHECKED);
	lv_obj_set_style_outline_width(g_cycle_kb, 0, LV_STATE_FOCUS_KEY);
	lv_obj_set_style_outline_opa(g_cycle_kb, LV_OPA_TRANSP, LV_STATE_FOCUS_KEY);

	lv_obj_add_event_cb(g_cycle_kb, cb_cycle_kb_encoder, LV_EVENT_FOCUSED, NULL);
	lv_obj_add_event_cb(g_cycle_kb, cb_cycle_kb_encoder, LV_EVENT_KEY | LV_EVENT_PREPROCESS, NULL);
	lv_obj_add_event_cb(g_cycle_kb, cb_cycle_kb_encoder, LV_EVENT_VALUE_CHANGED, NULL);
	s_cycle_kb_encoder_inited = true;
}

/* 循环程序页：数字键盘编码器进入编辑模式 */
static void cycle_kb_encoder_enter(void)
{
	if(g_cycle_kb == NULL || g_group_cycle == NULL) return;
	lv_group_focus_obj(g_cycle_kb);
	if(g_group_cycle) lv_group_set_editing(g_group_cycle, true);
	admin_kb_encoder_select_first(g_cycle_kb);
}

/* 循环程序页：数字键盘编码器退出编辑模式 */
static void cycle_kb_close(void)
{
	if(g_cycle_kb == NULL || !cycle_kb_is_visible()) return;

	prog_ui_save_fields(&g_pui_cycle);
	g_cycle_ta_active = NULL;
	lv_keyboard_set_textarea(g_cycle_kb, NULL);
	lv_obj_add_flag(g_cycle_kb, LV_OBJ_FLAG_HIDDEN);

	if(g_group_cycle != NULL) {
		if(g_group_cycle) lv_group_set_editing(g_group_cycle, false);
		if(g_pui_cycle.sel >= 0 && g_pui_cycle.sel < TOTAL_PROGRAMS &&
		   g_pui_cycle.btns[g_pui_cycle.sel] != NULL) {
			lv_group_focus_obj(g_pui_cycle.btns[g_pui_cycle.sel]);
		}
	}
}

static void cb_cycle_kb_cancel(lv_event_t * e)
{
	if(lv_event_get_code(e) != LV_EVENT_CANCEL) return;
	cycle_kb_close();
}

static void cb_cycle_kb_encoder(lv_event_t * e)
{
	lv_obj_t * kb = lv_event_get_target_obj(e);
	lv_event_code_t code = lv_event_get_code(e);
	if(kb == NULL) return;

	if(code == LV_EVENT_FOCUSED) {
		if(g_group_cycle != NULL && (g_group_cycle && lv_group_get_editing(g_group_cycle)) &&
		   lv_keyboard_get_selected_button(kb) == LV_BUTTONMATRIX_BUTTON_NONE) {
			admin_kb_encoder_select_first(kb);
		}
	}
	else if(code == LV_EVENT_KEY) {
		if(g_group_cycle != NULL && (g_group_cycle && lv_group_get_editing(g_group_cycle))) {
			uint32_t key = lv_event_get_key(e);
			if(key == LV_KEY_RIGHT) {
				admin_kb_encoder_step(kb, +1);
				lv_event_stop_processing(e);
			}
			else if(key == LV_KEY_LEFT) {
				admin_kb_encoder_step(kb, -1);
				lv_event_stop_processing(e);
			}
		}
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		lv_obj_invalidate(kb);
	}
}

static void cycle_ta_begin_edit(lv_obj_t * ta)
{
	if(g_cycle_kb == NULL || lv_scr_act() != g_scr_cycle) return;
	if(ta == NULL || lv_obj_has_flag(ta, LV_OBJ_FLAG_HIDDEN)) return;
	g_cycle_ta_active = ta;
	lv_keyboard_set_textarea(g_cycle_kb, ta);
	lv_obj_remove_flag(g_cycle_kb, LV_OBJ_FLAG_HIDDEN);
	cycle_kb_encoder_enter();
}

static void cb_cycle_prog_ta_focus(lv_event_t * e)
{
	lv_obj_t * ta = lv_event_get_target_obj(e);
	lv_event_code_t code = lv_event_get_code(e);
	if(lv_scr_act() != g_scr_cycle) return;

	if(code == LV_EVENT_FOCUSED) {
		if(g_group_cycle != NULL && (g_group_cycle && lv_group_get_editing(g_group_cycle))) {
			cycle_ta_begin_edit(ta);
		}
	} else if(code == LV_EVENT_CLICKED) {
		cycle_ta_begin_edit(ta);
		if(g_group_cycle != NULL) {
			if(g_group_cycle) lv_group_set_editing(g_group_cycle, true);
		}
	}
}

static void cycle_prog_roller_exit_edit(lv_obj_t * roller)
{
	if(g_group_cycle == NULL || roller == NULL) return;
	if((g_group_cycle ? lv_group_get_focused(g_group_cycle) : NULL) != roller) return;
	if(!(g_group_cycle && lv_group_get_editing(g_group_cycle))) return;
	uint32_t sel = lv_roller_get_selected(roller);
	lv_roller_set_selected(roller, sel, LV_ANIM_OFF);
	prog_ui_save_fields(&g_pui_cycle);
	if(g_group_cycle) lv_group_set_editing(g_group_cycle, false);
}

static void cb_cycle_prog_roller_encoder(lv_event_t * e)
{
	lv_obj_t * roller = lv_event_get_target_obj(e);
	lv_event_code_t code = lv_event_get_code(e);

	if(lv_scr_act() != g_scr_cycle) return;
	if(roller == NULL || lv_obj_has_flag(roller, LV_OBJ_FLAG_HIDDEN)) return;

	if(code == LV_EVENT_DEFOCUSED) {
		if(g_group_cycle != NULL && (g_group_cycle && lv_group_get_editing(g_group_cycle))) {
			cycle_prog_roller_exit_edit(roller);
		}
		return;
	}

	if(cycle_kb_is_visible()) return;

	if(code != LV_EVENT_CLICKED) return;
	if(g_group_cycle == NULL || (g_group_cycle ? lv_group_get_focused(g_group_cycle) : NULL) != roller) return;

	if((g_group_cycle && lv_group_get_editing(g_group_cycle))) {
		cycle_prog_roller_exit_edit(roller);
	} else {
		uint32_t sel = lv_roller_get_selected(roller);
		lv_roller_set_selected(roller, sel, LV_ANIM_OFF);
		if(g_group_cycle) lv_group_set_editing(g_group_cycle, true);
	}
	lv_event_stop_processing(e);
}

/* 循环程序页：编码器组构建 */
static void cycle_encoder_group_build(void)
{
	if(g_group_cycle == NULL) return;

	lv_group_remove_all_objs(g_group_cycle);

	if(g_cycle_btn_back != NULL) ui_encoder_group_add(g_group_cycle, g_cycle_btn_back);
	if(g_cycle_btn_runpause != NULL) ui_encoder_group_add(g_group_cycle, g_cycle_btn_runpause);
	if(g_cycle_btn_power != NULL) ui_encoder_group_add(g_group_cycle, g_cycle_btn_power);

	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		if(g_pui_cycle.btns[i] != NULL) {
			ui_encoder_group_add(g_group_cycle, g_pui_cycle.btns[i]);
		}
	}

	if(g_pui_cycle.sel >= 0 && g_pui_cycle.sel < TOTAL_PROGRAMS) {
		const ui_program_admin_t * c = &g_cycle_cfg[g_pui_cycle.sel];
		if(g_pui_cycle.roller[0] != NULL && (c->cap & PROG_CAP_TEMP)) {
			ui_encoder_group_add_prog_field(g_group_cycle, g_pui_cycle.roller[0]);
		}
		if(g_pui_cycle.ta[0] != NULL && (c->cap & PROG_CAP_INIT_DRY)) {
			ui_encoder_group_add_prog_field(g_group_cycle, g_pui_cycle.ta[0]);
		}
		if(g_pui_cycle.ta[1] != NULL && (c->cap & PROG_CAP_ADD_COUNT)) {
			ui_encoder_group_add_prog_field(g_group_cycle, g_pui_cycle.ta[1]);
		}
		if(g_pui_cycle.roller[1] != NULL && (c->cap & PROG_CAP_ADD_TIME)) {
			ui_encoder_group_add_prog_field(g_group_cycle, g_pui_cycle.roller[1]);
		}
	}

	if(g_cycle_kb != NULL) {
		lv_obj_add_flag(g_cycle_kb, LV_OBJ_FLAG_CLICK_FOCUSABLE);
		lv_obj_set_style_outline_width(g_cycle_kb, 0, LV_STATE_FOCUS_KEY);
		lv_obj_set_style_outline_opa(g_cycle_kb, LV_OPA_TRANSP, LV_STATE_FOCUS_KEY);
		lv_group_add_obj(g_group_cycle, g_cycle_kb);
	}

	lv_obj_t * focus_first = g_cycle_btn_back;
	if(cycle_kb_is_visible() && g_cycle_ta_active != NULL) {
		if(g_group_cycle) lv_group_set_editing(g_group_cycle, true);
		focus_first = g_cycle_kb;
	} else {
		if(g_group_cycle) lv_group_set_editing(g_group_cycle, false);
		if(g_pui_cycle.sel >= 0 && g_pui_cycle.sel < TOTAL_PROGRAMS &&
		   g_pui_cycle.btns[g_pui_cycle.sel] != NULL) {
			focus_first = g_pui_cycle.btns[g_pui_cycle.sel];
		}
	}

	if(focus_first != NULL) {
		lv_group_focus_obj(focus_first);
	}
	if(focus_first == g_cycle_kb && g_group_cycle != NULL && (g_group_cycle && lv_group_get_editing(g_group_cycle))) {
		admin_kb_encoder_select_first(g_cycle_kb);
	}
}

/* 循环程序页：程序 Tab 切换 */
static void cb_cycle_prog_pick(lv_event_t * e)
{
	prog_ui_save_fields(&g_pui_cycle);
	g_pui_cycle.sel = (int32_t)(intptr_t)lv_event_get_user_data(e);
	prog_ui_load_fields(&g_pui_cycle);
	cycle_encoder_group_build();
}

/* 循环程序页：参数字段变更 */
static void cb_cycle_prog_field_changed(lv_event_t * e)
{
	(void)e;
	if(g_pui_cycle.ui_loading) return;
	prog_ui_save_fields(&g_pui_cycle);
	prog_ui_update_total_display(&g_pui_cycle);
}

/* 循环程序页：构建程序参数区（5 参：无程序金额/追加时间金额） */
static void prog_ui_build_cycle_program_panel(lv_obj_t * panel, lv_coord_t panel_h)
{
	static const ui_str_id_t field_ids[CYCLE_PROG_FIELD_CNT] = {
		STR_PROG_FIELD_DRY_TEMP, STR_PROG_FIELD_INIT_DRY, STR_PROG_FIELD_COOL_TIME,
		STR_PROG_FIELD_ADD_COUNT, STR_PROG_FIELD_ADD_TIME,
	};

	lv_obj_set_size(panel, LV_PCT(100), panel_h);
	lv_obj_set_style_bg_opa(panel, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(panel, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(panel, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(panel, LV_LAYOUT_NONE, LV_PART_MAIN);

	const lv_coord_t prog_btn_w = 140;
	const lv_coord_t prog_btn_h = 48;
	const lv_coord_t prog_gap = 12;
	const lv_coord_t prog_row_w = TOTAL_PROGRAMS * prog_btn_w + (TOTAL_PROGRAMS - 1) * prog_gap;
	const lv_coord_t prog_row_x_shift = 30;   /* 上栏整行水平偏移：正值左移，负值右移 */
	const lv_coord_t prog_row_x0 = (lv_coord_t)((UI_FIXED_W - prog_row_w) / 2) - prog_row_x_shift;
	/* 与管理员 program_admin_build_panel 相同坐标 */
	const lv_coord_t prog_row_y = 36 + 30;   //上栏程序列表的Y坐标

	const lv_coord_t total_box_w = 120;
	const lv_coord_t total_gap = 12;
	const lv_coord_t total_x = prog_row_x0 - total_gap - total_box_w;

	lv_obj_t * total_box = program_admin_make_value_box(panel, total_box_w, prog_btn_h);
	lv_obj_set_pos(total_box, total_x, prog_row_y);
	g_pui_cycle.total_val_lbl = lv_label_create(total_box);
	lv_label_set_text(g_pui_cycle.total_val_lbl, "0min");
	ui_set_obj_font(g_pui_cycle.total_val_lbl, s_font_sc_30);
	lv_obj_set_style_text_color(g_pui_cycle.total_val_lbl, lv_color_hex(0x333333), LV_PART_MAIN);

	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		g_pui_cycle.btns[i] = make_admin_prog_btn(panel, ui_program_name_get(i));
		admin_prog_btn_bind_i18n(g_pui_cycle.btns[i], g_mode_name_ids[i]);
		lv_obj_set_size(g_pui_cycle.btns[i], prog_btn_w, prog_btn_h);
		lv_obj_set_pos(g_pui_cycle.btns[i],
			prog_row_x0 + i * (prog_btn_w + prog_gap), prog_row_y);
		lv_obj_add_event_cb(g_pui_cycle.btns[i], cb_cycle_prog_pick, LV_EVENT_CLICKED, (void *)(intptr_t)i);
	}

	/* 上栏右侧：已完成次数（与左侧总时长框对称位置，框宽加大以容纳完整文案） */
	const lv_coord_t count_box_w = 200;
	const lv_coord_t count_box_x = prog_row_x0 + prog_row_w + total_gap;
	lv_obj_t * count_box = program_admin_make_value_box(panel, count_box_w, prog_btn_h);
	lv_obj_set_pos(count_box, count_box_x, prog_row_y);
	g_cycle_lbl_run_count = lv_label_create(count_box);
	lv_obj_set_width(g_cycle_lbl_run_count, LV_PCT(100));
	lv_label_set_long_mode(g_cycle_lbl_run_count, LV_LABEL_LONG_CLIP);
	lv_obj_set_style_text_align(g_cycle_lbl_run_count, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	ui_set_obj_font(g_cycle_lbl_run_count, s_font_sc_30);
	lv_obj_set_style_text_color(g_cycle_lbl_run_count, lv_color_hex(0x333333), LV_PART_MAIN);
	cycle_run_count_label_sync();

	const lv_coord_t field_x0 = 340;            //第一列左边距
	const lv_coord_t field_gap = 25;            //列与列之间的间距
	const lv_coord_t field_h = 60;              //白底外框（文本框容器）的高
	const lv_coord_t field_w = 158;             //白底外框（文本框容器）的宽
	const lv_coord_t field_y_val = 132 + 70;    //文本框/滚轮的 Y 坐标

	for(int i = 0; i < (int)CYCLE_PROG_FIELD_CNT; i++) {
		const lv_coord_t fx = field_x0 + i * (field_w + field_gap);

		g_pui_cycle.field_box[i] = program_admin_make_value_box(panel, field_w, field_h);
		lv_obj_set_pos(g_pui_cycle.field_box[i], fx, field_y_val);
		lv_obj_add_flag(g_pui_cycle.field_box[i], LV_OBJ_FLAG_OVERFLOW_VISIBLE);

		g_pui_cycle.dash[i] = lv_label_create(g_pui_cycle.field_box[i]);
		lv_label_set_text(g_pui_cycle.dash[i], "--");
		ui_set_obj_font(g_pui_cycle.dash[i], s_font_sc_30);
		lv_obj_set_style_text_color(g_pui_cycle.dash[i], lv_color_hex(0x333333), LV_PART_MAIN);
		lv_obj_add_flag(g_pui_cycle.dash[i], LV_OBJ_FLAG_HIDDEN);

		lv_obj_t * lbl = lv_label_create(panel);
		g_pui_cycle.field_lbl[i] = lbl;
		ui_lang_bind_label(lbl, field_ids[i]);
		ui_set_obj_font(lbl, s_font_sc_30);
		lv_obj_set_style_text_color(lbl, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		lv_obj_set_width(lbl, LV_SIZE_CONTENT);
		lv_obj_align_to(lbl, g_pui_cycle.field_box[i], LV_ALIGN_OUT_TOP_MID, 0, -PROG_ADMIN_LBL_BOX_GAP);
	}

	g_pui_cycle.roller[0] = lv_roller_create(g_pui_cycle.field_box[0]);
	program_admin_style_field_inner(g_pui_cycle.roller[0]);
	lv_roller_set_visible_row_count(g_pui_cycle.roller[0], 1);
	lv_obj_set_style_text_font(g_pui_cycle.roller[0], s_font_sc_30, LV_PART_MAIN);
	lv_obj_set_style_text_font(g_pui_cycle.roller[0], s_font_sc_30, LV_PART_SELECTED);
	lv_obj_set_style_bg_opa(g_pui_cycle.roller[0], LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(g_pui_cycle.roller[0], LV_OPA_TRANSP, LV_PART_SELECTED);
	lv_obj_set_style_text_color(g_pui_cycle.roller[0], lv_color_hex(0x000000), LV_PART_MAIN);
	lv_obj_set_style_text_color(g_pui_cycle.roller[0], lv_color_hex(0x000000), LV_PART_SELECTED);
	lv_obj_add_event_cb(g_pui_cycle.roller[0], cb_cycle_prog_roller_encoder, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(g_pui_cycle.roller[0], cb_cycle_prog_roller_encoder, LV_EVENT_DEFOCUSED, NULL);
	lv_obj_add_event_cb(g_pui_cycle.roller[0], cb_cycle_prog_field_changed, LV_EVENT_VALUE_CHANGED, NULL);
	style_prog_field_encoder_focus_inner(g_pui_cycle.roller[0]);
	lv_roller_set_options(g_pui_cycle.roller[0], "36℃\n40℃\n44℃", LV_ROLLER_MODE_NORMAL);

	/* 仅初始烘干时间、追加次数；无程序金额/追加时间金额 */
	for(int i = 0; i < 2; i++) {
		const int box_idx = (i == 0) ? 1 : 3;
		g_pui_cycle.ta[i] = lv_textarea_create(g_pui_cycle.field_box[box_idx]);
		program_admin_style_ta(g_pui_cycle.ta[i]);
		lv_textarea_set_max_length(g_pui_cycle.ta[i], (i <= 1) ? 2 : 3);
		lv_textarea_set_accepted_chars(g_pui_cycle.ta[i], "0123456789");
		lv_obj_set_style_text_font(g_pui_cycle.ta[i], s_font_sc_30, LV_PART_MAIN);
		lv_obj_set_style_text_color(g_pui_cycle.ta[i], lv_color_hex(0x000000), LV_PART_MAIN);
		lv_obj_add_flag(g_pui_cycle.ta[i], LV_OBJ_FLAG_CLICKABLE);
		lv_obj_add_event_cb(g_pui_cycle.ta[i], cb_cycle_prog_ta_focus, LV_EVENT_ALL, NULL);
		style_prog_field_encoder_focus_inner(g_pui_cycle.ta[i]);
		lv_obj_add_event_cb(g_pui_cycle.ta[i], cb_cycle_prog_field_changed, LV_EVENT_VALUE_CHANGED, NULL);
	}

	g_pui_cycle.cool_lbl = lv_label_create(g_pui_cycle.field_box[2]);
	lv_label_set_text(g_pui_cycle.cool_lbl, "0min");
	ui_set_obj_font(g_pui_cycle.cool_lbl, s_font_sc_30);
	lv_obj_set_style_text_color(g_pui_cycle.cool_lbl, lv_color_hex(0x333333), LV_PART_MAIN);
	lv_obj_set_width(g_pui_cycle.cool_lbl, LV_PCT(100));
	lv_obj_set_style_text_align(g_pui_cycle.cool_lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

	g_pui_cycle.roller[1] = lv_roller_create(g_pui_cycle.field_box[4]);
	program_admin_style_field_inner(g_pui_cycle.roller[1]);
	lv_roller_set_visible_row_count(g_pui_cycle.roller[1], 1);
	lv_obj_set_style_text_font(g_pui_cycle.roller[1], s_font_sc_30, LV_PART_MAIN);
	lv_obj_set_style_text_font(g_pui_cycle.roller[1], s_font_sc_30, LV_PART_SELECTED);
	lv_obj_set_style_bg_opa(g_pui_cycle.roller[1], LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(g_pui_cycle.roller[1], LV_OPA_TRANSP, LV_PART_SELECTED);
	lv_obj_set_style_text_color(g_pui_cycle.roller[1], lv_color_hex(0x000000), LV_PART_MAIN);
	lv_obj_set_style_text_color(g_pui_cycle.roller[1], lv_color_hex(0x000000), LV_PART_SELECTED);
	lv_obj_add_event_cb(g_pui_cycle.roller[1], cb_cycle_prog_roller_encoder, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(g_pui_cycle.roller[1], cb_cycle_prog_roller_encoder, LV_EVENT_DEFOCUSED, NULL);
	lv_obj_add_event_cb(g_pui_cycle.roller[1], cb_cycle_prog_field_changed, LV_EVENT_VALUE_CHANGED, NULL);
	style_prog_field_encoder_focus_inner(g_pui_cycle.roller[1]);
	lv_roller_set_options(g_pui_cycle.roller[1], PROG_ADMIN_ADD_TIME_ROLLER_OPTS, LV_ROLLER_MODE_NORMAL);

	g_pui_cycle.cfg_tbl = g_cycle_cfg;
	g_pui_cycle.sel = 0;
	prog_ui_load_fields(&g_pui_cycle);
}

/* 构建程序设置子页：上栏程序列表，下栏 7 参数（label/value/edit） */
static void program_admin_build_panel(lv_obj_t * root, lv_coord_t body_y, lv_coord_t body_h)
{
	static const ui_str_id_t field_ids[PROG_ADMIN_FIELD_CNT] = {
		STR_PROG_FIELD_DRY_TEMP, STR_PROG_FIELD_INIT_DRY, STR_PROG_FIELD_COOL_TIME,
		STR_PROG_FIELD_ADD_COUNT, STR_PROG_FIELD_ADD_TIME,
		STR_PROG_FIELD_PRICE, STR_PROG_FIELD_ADD_PRICE
	};

	g_admin_panel_program = lv_obj_create(root);
	lv_obj_set_size(g_admin_panel_program, LV_PCT(100), body_h);
	lv_obj_align(g_admin_panel_program, LV_ALIGN_TOP_MID, 0, body_y);
	lv_obj_set_style_bg_opa(g_admin_panel_program, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(g_admin_panel_program, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(g_admin_panel_program, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(g_admin_panel_program, LV_LAYOUT_NONE, LV_PART_MAIN);
	lv_obj_remove_flag(g_admin_panel_program, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_add_flag(g_admin_panel_program, LV_OBJ_FLAG_HIDDEN);

	g_admin_img_prog_title_box = lv_image_create(g_admin_panel_program);
	lv_image_set_src(g_admin_img_prog_title_box, &title_box);
	lv_obj_align(g_admin_img_prog_title_box, LV_ALIGN_TOP_MID, 0, 25);

	g_admin_lbl_prog_title = lv_label_create(g_admin_panel_program);
	ui_lang_bind_label(g_admin_lbl_prog_title, STR_ADMIN_M1_PROGRAM);
	ui_set_obj_font(g_admin_lbl_prog_title, s_font_sc_30);
	lv_obj_set_style_text_color(g_admin_lbl_prog_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_align(g_admin_lbl_prog_title, LV_ALIGN_TOP_MID, 0, 25);

	g_admin_prog_list = lv_obj_create(g_admin_panel_program);
	lv_obj_set_size(g_admin_prog_list, LV_PCT(100), body_h);
	lv_obj_set_pos(g_admin_prog_list, 0, 0);
	lv_obj_set_style_bg_opa(g_admin_prog_list, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(g_admin_prog_list, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(g_admin_prog_list, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(g_admin_prog_list, LV_LAYOUT_NONE, LV_PART_MAIN);
	lv_obj_remove_flag(g_admin_prog_list, LV_OBJ_FLAG_SCROLLABLE);

	const lv_coord_t prog_btn_w = 187;
	const lv_coord_t prog_btn_h = 186;
	const lv_coord_t prog_gap = 100;
	const lv_coord_t prog_row_w = TOTAL_PROGRAMS * prog_btn_w + (TOTAL_PROGRAMS - 1) * prog_gap;
	const lv_coord_t prog_row_x0 = (lv_coord_t)((UI_FIXED_W - prog_row_w) / 2);
	const lv_coord_t prog_row_y = 190;

	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		g_admin_prog_btns[i] = lv_button_create(g_admin_prog_list);
		lv_obj_set_size(g_admin_prog_btns[i], prog_btn_w, prog_btn_h);
		lv_obj_set_pos(g_admin_prog_btns[i],
			prog_row_x0 + i * (prog_btn_w + prog_gap), prog_row_y);
		lv_obj_set_style_bg_opa(g_admin_prog_btns[i], LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(g_admin_prog_btns[i], 0, LV_PART_MAIN);
		lv_obj_set_style_shadow_width(g_admin_prog_btns[i], 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(g_admin_prog_btns[i], 0, LV_PART_MAIN);
		lv_obj_t * icon_wrap = lv_obj_create(g_admin_prog_btns[i]);
		lv_obj_set_size(icon_wrap, 140, 140);
		lv_obj_align(icon_wrap, LV_ALIGN_TOP_MID, 0, 0);
		lv_obj_set_style_bg_opa(icon_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(icon_wrap, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(icon_wrap, 0, LV_PART_MAIN);
		lv_obj_remove_flag(icon_wrap, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
		lv_obj_t * pimg = lv_image_create(icon_wrap);
		const lv_image_dsc_t * dsc = g_program_imgs[i];
		lv_image_set_src(pimg, dsc);
		/* 主页程序图约 303px，缩进 140×140 框内 */
		if(dsc != NULL && dsc->header.w > 0) {
			lv_image_set_scale(pimg, (int32_t)((140u * 256u) / (uint32_t)dsc->header.w));
		}
		lv_obj_center(pimg);
		g_admin_prog_btn_lbls[i] = lv_label_create(g_admin_prog_btns[i]);
		lv_label_set_text(g_admin_prog_btn_lbls[i], ui_program_name_get(i));
		ui_set_obj_font(g_admin_prog_btn_lbls[i], s_font_sc_20);
		lv_obj_set_style_text_color(g_admin_prog_btn_lbls[i], lv_color_hex(COL_TEXT), LV_PART_MAIN);
		lv_obj_align(g_admin_prog_btn_lbls[i], LV_ALIGN_BOTTOM_MID, 0, -8);
		ui_lang_bind_label(g_admin_prog_btn_lbls[i], g_mode_name_ids[i]);
		lv_obj_add_event_cb(g_admin_prog_btns[i], cb_admin_prog_pick, LV_EVENT_CLICKED, (void *)(intptr_t)i);
	}

	g_admin_prog_detail = lv_obj_create(g_admin_panel_program);
	lv_obj_set_size(g_admin_prog_detail, LV_PCT(100), body_h);
	lv_obj_set_pos(g_admin_prog_detail, 0, 0);
	lv_obj_set_style_bg_opa(g_admin_prog_detail, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(g_admin_prog_detail, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(g_admin_prog_detail, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(g_admin_prog_detail, LV_LAYOUT_NONE, LV_PART_MAIN);
	lv_obj_remove_flag(g_admin_prog_detail, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_add_flag(g_admin_prog_detail, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_remove_flag(g_admin_prog_detail, LV_OBJ_FLAG_GESTURE_BUBBLE);
	lv_obj_add_event_cb(g_admin_prog_detail, cb_admin_prog_detail_gesture, LV_EVENT_GESTURE, NULL);
	lv_obj_add_flag(g_admin_prog_detail, LV_OBJ_FLAG_HIDDEN);

	admin_panel_apply_shell(g_admin_prog_detail, NULL);

	const lv_coord_t total_box_w = 120;
	const lv_coord_t total_x = 80;
	const lv_coord_t prog_btn_h_detail = 48;
	const lv_coord_t prog_row_y_detail = 90;

	lv_obj_t * total_box = program_admin_make_value_box(g_admin_prog_detail, total_box_w, prog_btn_h_detail);
	lv_obj_set_pos(total_box, total_x, prog_row_y_detail);
	g_admin_prog_total_val_lbl = lv_label_create(total_box);
	lv_label_set_text(g_admin_prog_total_val_lbl, "0min");
	ui_set_obj_font(g_admin_prog_total_val_lbl, s_font_sc_30);
	lv_obj_set_style_text_color(g_admin_prog_total_val_lbl, lv_color_hex(0x333333), LV_PART_MAIN);

	/* 下栏：左侧 8 参数 + 右侧重置/确认（field_w × field_h 全部一致） */
	const lv_coord_t btn_col_w = 100;
	const lv_coord_t btn_col_x = (lv_coord_t)UI_FIXED_W - btn_col_w - 80;
	const lv_coord_t field_x0 = 70;//第一列左边距
	const lv_coord_t field_gap = 25;//列与列之间的间距
	const lv_coord_t field_h = 60;//白底外框（文本框容器）的高
	const lv_coord_t field_w = 158;//白底外框（文本框容器）的宽
	const lv_coord_t field_y_val = 132 + 70;//文本框/滚轮的 Y 坐标

	for(int i = 0; i < PROG_ADMIN_FIELD_CNT; i++) {
		const lv_coord_t fx = field_x0 + i * (field_w + field_gap);

		/* 统一外框：8 列同宽同高，内部控件铺满外框 */
		g_admin_prog_field_box[i] = program_admin_make_value_box(g_admin_prog_detail, field_w, field_h);
		lv_obj_set_pos(g_admin_prog_field_box[i], fx, field_y_val);
		lv_obj_add_flag(g_admin_prog_field_box[i], LV_OBJ_FLAG_OVERFLOW_VISIBLE);

		g_admin_prog_dash[i] = lv_label_create(g_admin_prog_field_box[i]);
		lv_label_set_text(g_admin_prog_dash[i], "--");
		ui_set_obj_font(g_admin_prog_dash[i], s_font_sc_30);
		lv_obj_set_style_text_color(g_admin_prog_dash[i], lv_color_hex(0x333333), LV_PART_MAIN);
		lv_obj_add_flag(g_admin_prog_dash[i], LV_OBJ_FLAG_HIDDEN);

		lv_obj_t * lbl = lv_label_create(g_admin_prog_detail);
		g_admin_prog_field_lbl[i] = lbl;
		ui_lang_bind_label(lbl, field_ids[i]);
		ui_set_obj_font(lbl, s_font_sc_30);
		lv_obj_set_style_text_color(lbl, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		lv_obj_set_width(lbl, LV_SIZE_CONTENT);
		lv_obj_align_to(lbl, g_admin_prog_field_box[i], LV_ALIGN_OUT_TOP_MID, 0, -PROG_ADMIN_LBL_BOX_GAP);
	}

	/* col0: 烘干温度 roller */
	g_admin_prog_roller[0] = lv_roller_create(g_admin_prog_field_box[0]);
	program_admin_style_field_inner(g_admin_prog_roller[0]);
	lv_roller_set_visible_row_count(g_admin_prog_roller[0], 1);
	lv_obj_set_style_text_font(g_admin_prog_roller[0], s_font_sc_30, LV_PART_MAIN);
	lv_obj_set_style_text_font(g_admin_prog_roller[0], s_font_sc_30, LV_PART_SELECTED);
	lv_obj_set_style_bg_opa(g_admin_prog_roller[0], LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(g_admin_prog_roller[0], LV_OPA_TRANSP, LV_PART_SELECTED);
	lv_obj_set_style_text_color(g_admin_prog_roller[0], lv_color_hex(0x000000), LV_PART_MAIN);
	lv_obj_set_style_text_color(g_admin_prog_roller[0], lv_color_hex(0x000000), LV_PART_SELECTED);
	lv_obj_add_event_cb(g_admin_prog_roller[0], cb_admin_prog_roller_encoder, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(g_admin_prog_roller[0], cb_admin_prog_roller_encoder, LV_EVENT_DEFOCUSED, NULL);
	lv_obj_add_event_cb(g_admin_prog_roller[0], cb_admin_prog_time_field_changed, LV_EVENT_VALUE_CHANGED, NULL);
	style_prog_field_encoder_focus_inner(g_admin_prog_roller[0]);
	lv_roller_set_options(g_admin_prog_roller[0], "36℃\n40℃\n44℃", LV_ROLLER_MODE_NORMAL);

	/* col1: 初始烘干时间；col3: 追加次数；col5/6: 金额 */
	for(int i = 0; i < 4; i++) {
		const int box_idx = (i == 0) ? 1 : (i == 1) ? 3 : (i == 2) ? 5 : 6;
		g_admin_prog_ta[i] = lv_textarea_create(g_admin_prog_field_box[box_idx]);
		program_admin_style_ta(g_admin_prog_ta[i]);
		lv_textarea_set_max_length(g_admin_prog_ta[i], (i <= 1) ? 2 : 3);
		lv_textarea_set_accepted_chars(g_admin_prog_ta[i], "0123456789");
		lv_obj_set_style_text_font(g_admin_prog_ta[i], s_font_sc_30, LV_PART_MAIN);
		lv_obj_set_style_text_color(g_admin_prog_ta[i], lv_color_hex(0x000000), LV_PART_MAIN);
		lv_obj_add_flag(g_admin_prog_ta[i], LV_OBJ_FLAG_CLICKABLE);
		lv_obj_add_event_cb(g_admin_prog_ta[i], cb_admin_prog_ta_focus, LV_EVENT_ALL, NULL);
		style_prog_field_encoder_focus_inner(g_admin_prog_ta[i]);
		lv_obj_add_event_cb(g_admin_prog_ta[i], cb_admin_prog_time_field_changed, LV_EVENT_VALUE_CHANGED, NULL);
	}

	/* col2: 冷却时间只读 */
	g_admin_prog_cool_lbl = lv_label_create(g_admin_prog_field_box[2]);
	lv_label_set_text(g_admin_prog_cool_lbl, "0min");
	ui_set_obj_font(g_admin_prog_cool_lbl, s_font_sc_30);
	lv_obj_set_style_text_color(g_admin_prog_cool_lbl, lv_color_hex(0x333333), LV_PART_MAIN);
	lv_obj_set_width(g_admin_prog_cool_lbl, LV_PCT(100));
	lv_obj_set_style_text_align(g_admin_prog_cool_lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

	/* col4: 追加时间 roller */
	g_admin_prog_roller[1] = lv_roller_create(g_admin_prog_field_box[4]);
	program_admin_style_field_inner(g_admin_prog_roller[1]);
	lv_roller_set_visible_row_count(g_admin_prog_roller[1], 1);
	lv_obj_set_style_text_font(g_admin_prog_roller[1], s_font_sc_30, LV_PART_MAIN);
	lv_obj_set_style_text_font(g_admin_prog_roller[1], s_font_sc_30, LV_PART_SELECTED);
	lv_obj_set_style_bg_opa(g_admin_prog_roller[1], LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(g_admin_prog_roller[1], LV_OPA_TRANSP, LV_PART_SELECTED);
	lv_obj_set_style_text_color(g_admin_prog_roller[1], lv_color_hex(0x000000), LV_PART_MAIN);
	lv_obj_set_style_text_color(g_admin_prog_roller[1], lv_color_hex(0x000000), LV_PART_SELECTED);
	lv_obj_add_event_cb(g_admin_prog_roller[1], cb_admin_prog_roller_encoder, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(g_admin_prog_roller[1], cb_admin_prog_roller_encoder, LV_EVENT_DEFOCUSED, NULL);
	lv_obj_add_event_cb(g_admin_prog_roller[1], cb_admin_prog_time_field_changed, LV_EVENT_VALUE_CHANGED, NULL);
	style_prog_field_encoder_focus_inner(g_admin_prog_roller[1]);
	lv_roller_set_options(g_admin_prog_roller[1], PROG_ADMIN_ADD_TIME_ROLLER_OPTS, LV_ROLLER_MODE_NORMAL);

	/* 下栏右侧：重置（上）、确认（下） */
	g_admin_btn_prog_reset = make_orange_outline_btn(g_admin_prog_detail, ui_translation(STR_BTN_RESET), btn_col_w, 44);
	lv_obj_set_pos(g_admin_btn_prog_reset, btn_col_x, field_y_val-35);
	ui_set_obj_font(lv_obj_get_child(g_admin_btn_prog_reset, 0), s_font_sc_30);
	orange_btn_bind_i18n(g_admin_btn_prog_reset, STR_BTN_RESET);
	lv_obj_add_event_cb(g_admin_btn_prog_reset, cb_admin_prog_reset, LV_EVENT_CLICKED, NULL);

	g_admin_btn_prog_confirm = make_orange_fill_btn(g_admin_prog_detail, ui_translation(STR_BTN_CONFIRM), btn_col_w, 44);
	lv_obj_set_pos(g_admin_btn_prog_confirm, btn_col_x, field_y_val-35 + 54);
	ui_set_obj_font(lv_obj_get_child(g_admin_btn_prog_confirm, 0), s_font_sc_30);
	orange_btn_bind_i18n(g_admin_btn_prog_confirm, STR_BTN_CONFIRM);
	lv_obj_add_event_cb(g_admin_btn_prog_confirm, cb_admin_prog_confirm, LV_EVENT_CLICKED, NULL);

	/* 管理员程序页：绑定 prog_ui 上下文供 save/load 复用 */
	g_pui_admin.cfg_tbl = g_prog_cfg;
	g_pui_admin.sel = 0;
	g_pui_admin.total_val_lbl = g_admin_prog_total_val_lbl;
	g_pui_admin.cool_lbl = g_admin_prog_cool_lbl;
	for(int i = 0; i < TOTAL_PROGRAMS; i++) g_pui_admin.btns[i] = g_admin_prog_btns[i];
	for(int i = 0; i < 4; i++) g_pui_admin.ta[i] = g_admin_prog_ta[i];
	for(int i = 0; i < 2; i++) g_pui_admin.roller[i] = g_admin_prog_roller[i];
	for(int i = 0; i < PROG_ADMIN_FIELD_CNT; i++) {
		g_pui_admin.field_box[i] = g_admin_prog_field_box[i];
		g_pui_admin.dash[i] = g_admin_prog_dash[i];
		g_pui_admin.field_lbl[i] = g_admin_prog_field_lbl[i];
	}

	g_admin_prog_sel = 0;
	g_admin_prog_is_detail = false;
	program_admin_ui_load_fields();
}

/* ---------- 管理员界面：子面板显示/隐藏、密码与机器 ID ---------- */
static void admin_menu_style_init(void)
{
	if(s_admin_menu_style_inited) return;
	/* 外层露出的 pad 环即为渐变边框（LVGL border 不支持纵向渐变） */
	lv_style_init(&s_admin_menu_frame_style);
	lv_style_set_radius(&s_admin_menu_frame_style, 8);
	lv_style_set_bg_opa(&s_admin_menu_frame_style, LV_OPA_COVER);
	lv_style_set_bg_color(&s_admin_menu_frame_style, lv_color_hex(0x2a7fff));
	lv_style_set_bg_grad_color(&s_admin_menu_frame_style, lv_color_hex(0x000000));
	lv_style_set_bg_grad_dir(&s_admin_menu_frame_style, LV_GRAD_DIR_VER);
	lv_style_set_border_width(&s_admin_menu_frame_style, 0);
	lv_style_set_pad_all(&s_admin_menu_frame_style, ADMIN_MENU_BORDER_W);
	lv_style_set_shadow_width(&s_admin_menu_frame_style, 0);

	lv_style_init(&s_admin_menu_inner_style);
	lv_style_set_radius(&s_admin_menu_inner_style, 6);
	lv_style_set_bg_opa(&s_admin_menu_inner_style, LV_OPA_COVER);
	lv_style_set_bg_color(&s_admin_menu_inner_style, lv_color_hex(0x0a2a5a));
	lv_style_set_bg_grad_color(&s_admin_menu_inner_style, lv_color_hex(0x000810));
	lv_style_set_bg_grad_dir(&s_admin_menu_inner_style, LV_GRAD_DIR_VER);
	lv_style_set_border_width(&s_admin_menu_inner_style, 0);
	lv_style_set_pad_all(&s_admin_menu_inner_style, 0);
	s_admin_menu_style_inited = true;
}

/* 创建管理员界面按钮 */
static lv_obj_t * make_admin_menu_btn(lv_obj_t * parent, const char * txt, const lv_image_dsc_t * icon)
{
	lv_obj_t * b = lv_button_create(parent);
	lv_obj_set_style_bg_opa(b, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(b, 0, LV_PART_MAIN);
	lv_obj_set_style_shadow_width(b, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(b, 0, LV_PART_MAIN);
	lv_obj_remove_flag(b, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_size(b, ADMIN_MENU_BTN_W, ADMIN_MENU_BTN_H);

	lv_obj_t * img_bg = lv_image_create(b);
	lv_image_set_src(img_bg, &admin_button_box);
	lv_obj_set_style_image_recolor_opa(img_bg, 0, LV_PART_MAIN);
	lv_obj_center(img_bg);
	lv_obj_add_flag(img_bg, LV_OBJ_FLAG_EVENT_BUBBLE);
	lv_obj_clear_flag(img_bg, LV_OBJ_FLAG_CLICKABLE);

	if(icon != NULL) {
		lv_obj_t * img_icon = lv_image_create(b);
		lv_image_set_src(img_icon, icon);
		lv_obj_set_style_image_recolor_opa(img_icon, 0, LV_PART_MAIN);
		lv_obj_align(img_icon, LV_ALIGN_TOP_MID, 0, 28);
		lv_obj_add_flag(img_icon, LV_OBJ_FLAG_EVENT_BUBBLE);
		lv_obj_clear_flag(img_icon, LV_OBJ_FLAG_CLICKABLE);
	}

	lv_obj_t * l = lv_label_create(b);
	lv_label_set_text(l, txt);
	lv_obj_set_style_text_color(l, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(l, s_font_sc_30);
	lv_obj_align(l, LV_ALIGN_TOP_MID, 0, 80);
	lv_obj_add_flag(l, LV_OBJ_FLAG_EVENT_BUBBLE);
	lv_obj_clear_flag(l, LV_OBJ_FLAG_CLICKABLE);
	return b;
}

/* 刷新机器 ID「当前 ID」标签 */
static void admin_machine_id_label_update(void)
{
    if(g_admin_lbl_machine_id_cur == NULL) return;
    char buf[32];
    if(g_machine_id == 0) {
        lv_label_set_text(g_admin_lbl_machine_id_cur, ui_translation(STR_MACHINE_ID_CUR_NONE));
    } else {
        lv_snprintf(buf, sizeof(buf), ui_translation(STR_MACHINE_ID_CUR_FMT), (unsigned)g_machine_id);
        lv_label_set_text(g_admin_lbl_machine_id_cur, buf);
    }
}

/* menu1 第 8 钮编码器右转：进入 menu2 */
static void admin_encoder_rebuild(void)
{
    if(g_group_admin == NULL) return;
    lv_group_remove_all_objs(g_group_admin);
    if(g_admin_btn_back != NULL) ui_encoder_group_add(g_group_admin, g_admin_btn_back);
    if(g_admin_btn_runpause != NULL) ui_encoder_group_add(g_group_admin, g_admin_btn_runpause);
    if(g_admin_btn_power != NULL) ui_encoder_group_add(g_group_admin, g_admin_btn_power);

    lv_obj_t * focus_first = g_admin_btn_back;
    switch(g_admin_view) {
    case PASSWORD:
        if(g_admin_ta_pwd != NULL) ui_encoder_group_add(g_group_admin, g_admin_ta_pwd);
        if(g_admin_kb != NULL) admin_encoder_group_add_kb(g_group_admin);
        if(admin_kb_is_visible() && g_admin_kb != NULL &&
           g_admin_kb_ta == g_admin_ta_pwd) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, true);
            focus_first = g_admin_kb;
        } else {
            focus_first = (g_admin_ta_pwd != NULL) ? g_admin_ta_pwd : g_admin_btn_back;
        }
        break;
    case MENU1:
        for(int i = 0; i < 8; i++) {
            if(g_admin_menu1_btns[i] != NULL) {
                ui_encoder_group_add(g_group_admin, g_admin_menu1_btns[i]);
            }
        }
        focus_first = (g_admin_menu1_btns[0] != NULL) ? g_admin_menu1_btns[0] : g_admin_btn_back;
        break;
    case MENU2:
        for(int i = 0; i < 8; i++) {
            if(g_admin_menu2_btns[i] != NULL) {
                ui_encoder_group_add(g_group_admin, g_admin_menu2_btns[i]);
            }
        }
        focus_first = (g_admin_menu2_btns[0] != NULL) ? g_admin_menu2_btns[0] : g_admin_btn_back;
        break;
    case VENDOR_SERIAL:
        if(g_admin_ta_vendor_serial != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_ta_vendor_serial);
        }
        if(g_admin_kb != NULL) {
            admin_encoder_group_add_kb(g_group_admin);
        }
        if(admin_kb_is_visible() && g_admin_kb != NULL &&
           g_admin_kb_ta == g_admin_ta_vendor_serial) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, true);
            focus_first = g_admin_kb;
        } else {
            focus_first = (g_admin_ta_vendor_serial != NULL) ? g_admin_ta_vendor_serial : g_admin_btn_back;
        }
        break;
    case VENDOR_MENU:
        if(g_admin_btn_vendor_self_check != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_btn_vendor_self_check);
        }
        if(g_admin_btn_vendor_self_learn != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_btn_vendor_self_learn);
        }
        focus_first = (g_admin_btn_vendor_self_check != NULL) ?
            g_admin_btn_vendor_self_check : g_admin_btn_back;
        break;
    case MACHINE_ID:
        if(g_admin_ta_machine_id != NULL) ui_encoder_group_add(g_group_admin, g_admin_ta_machine_id);
        if(g_admin_btn_machine_confirm != NULL) ui_encoder_group_add(g_group_admin, g_admin_btn_machine_confirm);
        if(g_admin_kb != NULL) admin_encoder_group_add_kb(g_group_admin);
        if(admin_kb_is_visible() && g_admin_kb != NULL &&
           g_admin_kb_ta == g_admin_ta_machine_id) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, true);
            focus_first = g_admin_kb;
        } else {
            focus_first = (g_admin_ta_machine_id != NULL) ? g_admin_ta_machine_id : g_admin_btn_back;
        }
        break;
    case PROGRAM_SETTINGS:
        for(int i = 0; i < TOTAL_PROGRAMS; i++) {
            if(g_admin_prog_btns[i] != NULL) ui_encoder_group_add(g_group_admin, g_admin_prog_btns[i]);
        }
        if(g_admin_prog_sel >= 0 && g_admin_prog_sel < TOTAL_PROGRAMS) {
            const ui_program_admin_t * c = &g_prog_cfg[g_admin_prog_sel];
            if(g_admin_prog_roller[0] != NULL && (c->cap & PROG_CAP_TEMP)) {
                ui_encoder_group_add_prog_field(g_group_admin, g_admin_prog_roller[0]);
            }
            if(g_admin_prog_ta[0] != NULL && (c->cap & PROG_CAP_INIT_DRY)) {
                ui_encoder_group_add_prog_field(g_group_admin, g_admin_prog_ta[0]);
            }
            if(g_admin_prog_ta[1] != NULL && (c->cap & PROG_CAP_ADD_COUNT)) {
                ui_encoder_group_add_prog_field(g_group_admin, g_admin_prog_ta[1]);
            }
            if(g_admin_prog_roller[1] != NULL && (c->cap & PROG_CAP_ADD_TIME)) {
                ui_encoder_group_add_prog_field(g_group_admin, g_admin_prog_roller[1]);
            }
            if(g_admin_prog_ta[2] != NULL && (c->cap & PROG_CAP_PRICE)) {
                ui_encoder_group_add_prog_field(g_group_admin, g_admin_prog_ta[2]);
            }
            if(g_admin_prog_ta[3] != NULL && (c->cap & PROG_CAP_ADD_PRICE)) {
                ui_encoder_group_add_prog_field(g_group_admin, g_admin_prog_ta[3]);
            }
        }
        if(g_admin_btn_prog_reset != NULL) ui_encoder_group_add(g_group_admin, g_admin_btn_prog_reset);
        if(g_admin_btn_prog_confirm != NULL) ui_encoder_group_add(g_group_admin, g_admin_btn_prog_confirm);
        if(g_admin_kb != NULL) admin_encoder_group_add_kb(g_group_admin);
        if(admin_kb_is_visible() && g_admin_ta_prog_active != NULL) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, true);
            focus_first = g_admin_kb;
        } else {
            if(g_group_admin) lv_group_set_editing(g_group_admin, false);
            if(g_admin_prog_sel >= 0 && g_admin_prog_sel < TOTAL_PROGRAMS &&
                g_admin_prog_btns[g_admin_prog_sel] != NULL) {
                focus_first = g_admin_prog_btns[g_admin_prog_sel];
            } else {
                focus_first = (g_admin_prog_btns[0] != NULL) ? g_admin_prog_btns[0] : g_admin_btn_back;
            }
        }
        break;
    case SCREEN_BRIGHTNESS:
        /* 焦点顺序：返回 → 启停 → 电源 → 常亮开关 → 亮度滑条 */
        if(g_admin_sw_run_always_on != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_sw_run_always_on);
        }
        if(g_admin_slider_brightness != NULL) {
            ui_encoder_group_add_brightness_slider(g_group_admin, g_admin_slider_brightness);
        }
        focus_first = (g_admin_sw_run_always_on != NULL) ?
            g_admin_sw_run_always_on : g_admin_btn_back;
        break;
    case SOUND_CONTROL:
        if(g_admin_sw_touch_sound != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_sw_touch_sound);
        }
        if(g_admin_sw_voice_broadcast != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_sw_voice_broadcast);
        }
        if(g_admin_slider_sound_volume != NULL) {
            ui_encoder_group_add_brightness_slider(g_group_admin, g_admin_slider_sound_volume);
        }
        if(g_admin_slider_touch_sound_volume != NULL) {
            ui_encoder_group_add_brightness_slider(g_group_admin, g_admin_slider_touch_sound_volume);
        }
        focus_first = (g_admin_sw_touch_sound != NULL) ?
            g_admin_sw_touch_sound : g_admin_btn_back;
        break;
    case DORMANCY_STANDBY:
        if(g_admin_dormancy_sw_time != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_dormancy_sw_time);
        }
        if(g_admin_dormancy_sw_no_sleep != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_dormancy_sw_no_sleep);
        }
        focus_first = (g_admin_dormancy_sw_time != NULL) ? g_admin_dormancy_sw_time : g_admin_btn_back;
        break;
    case LANGUAGE_SETTINGS:
        if(g_admin_btn_lang_zh != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_btn_lang_zh);
        }
        if(g_admin_btn_lang_en != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_btn_lang_en);
        }
        if(ui_lang_get() == UI_LANG_EN && g_admin_btn_lang_en != NULL) {
            focus_first = g_admin_btn_lang_en;
        }
        else if(g_admin_btn_lang_zh != NULL) {
            focus_first = g_admin_btn_lang_zh;
        }
        break;
    case FACTORY_RESET:
        if(g_admin_factory_phase == ADMIN_FACTORY_PHASE_PROMPT) {
            if(g_admin_btn_factory_ok != NULL) {
                ui_encoder_group_add(g_group_admin, g_admin_btn_factory_ok);
            }
            if(g_admin_btn_factory_cancel != NULL) {
                ui_encoder_group_add(g_group_admin, g_admin_btn_factory_cancel);
            }
            focus_first = (g_admin_btn_factory_ok != NULL) ? g_admin_btn_factory_ok : g_admin_btn_back;
        }
        else {
            focus_first = g_admin_btn_back;
        }
        break;
    case CONTACT_US:
        focus_first = g_admin_btn_back;
        break;
    case AUTO_DISPENSE:
        if(g_admin_btn_auto_dispense_on != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_btn_auto_dispense_on);
        }
        if(g_admin_btn_auto_dispense_off != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_btn_auto_dispense_off);
        }
        if(!ui_auto_dispense_get() && g_admin_btn_auto_dispense_off != NULL) {
            focus_first = g_admin_btn_auto_dispense_off;
        }
        else if(g_admin_btn_auto_dispense_on != NULL) {
            focus_first = g_admin_btn_auto_dispense_on;
        }
        break;
    case FRESH_AIR_CARE:
        if(g_admin_btn_fresh_air_care_on != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_btn_fresh_air_care_on);
        }
        if(g_admin_btn_fresh_air_care_off != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_btn_fresh_air_care_off);
        }
        if(!ui_fresh_air_care_get() && g_admin_btn_fresh_air_care_off != NULL) {
            focus_first = g_admin_btn_fresh_air_care_off;
        }
        else if(g_admin_btn_fresh_air_care_on != NULL) {
            focus_first = g_admin_btn_fresh_air_care_on;
        }
        break;
    case SYSTEM_UPGRADE:
        if(g_admin_system_upgrade_phase == ADMIN_SYSTEM_UPGRADE_PHASE_PROMPT) {
            if(g_admin_btn_system_upgrade_ok != NULL) {
                ui_encoder_group_add(g_group_admin, g_admin_btn_system_upgrade_ok);
            }
            focus_first = (g_admin_btn_system_upgrade_ok != NULL) ?
                g_admin_btn_system_upgrade_ok : g_admin_btn_back;
        }
        else {
            focus_first = g_admin_btn_back;
        }
        break;
    case PAYMENT_SETTINGS:
        if(g_admin_sw_payment_method != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_sw_payment_method);
        }
        if(g_admin_sw_payment_timeout != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_sw_payment_timeout);
        }
        if(g_admin_sw_payment_order != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_sw_payment_order);
        }
        focus_first = (g_admin_sw_payment_method != NULL) ? g_admin_sw_payment_method : g_admin_btn_back;
        break;
    case DATA_SETTINGS: {
        unsigned di;

        for(di = 0; di < 7; di++) {
            if(g_admin_cb_data_upload[di] != NULL) {
                ui_encoder_group_add(g_group_admin, g_admin_cb_data_upload[di]);
            }
        }
        for(di = 0; di < 5; di++) {
            if(g_admin_cb_data_strategy[di] != NULL) {
                ui_encoder_group_add(g_group_admin, g_admin_cb_data_strategy[di]);
            }
        }
        if(g_group_admin != NULL) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, false);
        }
        focus_first = (g_admin_cb_data_upload[0] != NULL) ?
            g_admin_cb_data_upload[0] : g_admin_btn_back;
        break;
    }
    case NETWORK_SETTINGS:
        if(g_admin_btn_network_wifi != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_btn_network_wifi);
        }
        if(g_admin_btn_network_4g != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_btn_network_4g);
        }
        focus_first = (g_admin_btn_network_wifi != NULL) ?
            g_admin_btn_network_wifi : g_admin_btn_back;
        break;
    case WIFI_SETTINGS:
        focus_first = g_admin_btn_back;
        break;
    case SETTINGS_4G:
        if(g_admin_4g_phase == ADMIN_4G_PHASE_PROMPT && g_admin_sw_4g != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_sw_4g);
            focus_first = g_admin_sw_4g;
        }
        else {
            focus_first = g_admin_btn_back;
        }
        break;
    case PASSWORD_CHANGE_OLD:
        if(g_admin_ta_pwd_chg_old != NULL) ui_encoder_group_add(g_group_admin, g_admin_ta_pwd_chg_old);
        if(g_admin_kb != NULL) admin_encoder_group_add_kb(g_group_admin);
        if(admin_kb_is_visible() && g_admin_kb != NULL &&
           g_admin_kb_ta == g_admin_ta_pwd_chg_old) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, true);
            focus_first = g_admin_kb;
        } else {
            focus_first = (g_admin_ta_pwd_chg_old != NULL) ? g_admin_ta_pwd_chg_old : g_admin_btn_back;
        }
        break;
    case PASSWORD_CHANGE_NEW:
        if(g_admin_ta_pwd_chg_new1 != NULL) ui_encoder_group_add(g_group_admin, g_admin_ta_pwd_chg_new1);
        if(g_admin_ta_pwd_chg_new2 != NULL) ui_encoder_group_add(g_group_admin, g_admin_ta_pwd_chg_new2);
        if(g_admin_kb != NULL) admin_encoder_group_add_kb(g_group_admin);
        if(admin_kb_is_visible() && g_admin_kb != NULL) {
            lv_obj_t * kb_ta = g_admin_kb_ta;
            if(kb_ta == g_admin_ta_pwd_chg_new1 || kb_ta == g_admin_ta_pwd_chg_new2) {
                if(g_group_admin) lv_group_set_editing(g_group_admin, true);
                focus_first = g_admin_kb;
            }
        }
        if(focus_first == g_admin_btn_back) {
            focus_first = (g_admin_pwd_chg_page1_step == 1 && g_admin_ta_pwd_chg_new2 != NULL) ?
                g_admin_ta_pwd_chg_new2 : g_admin_ta_pwd_chg_new1;
            if(focus_first == NULL) focus_first = g_admin_btn_back;
        }
        break;
    default:
        break;
    }
    if(focus_first != NULL) {
        lv_group_focus_obj(focus_first);
        /* lv_group_focus_obj 经 POINTER indev 派发 FOCUSED 时可能无 FOCUS_KEY，补态以显示描边/内框 */
        if((g_group_admin ? lv_group_get_focused(g_group_admin) : NULL) == focus_first) {
            lv_obj_add_state(focus_first, LV_STATE_FOCUS_KEY);
        }
        if(focus_first == g_admin_kb && g_group_admin != NULL && (g_group_admin && lv_group_get_editing(g_group_admin))) {
            admin_kb_encoder_select_first(g_admin_kb);
        }
    }
}

/* 切换管理员子面板显示、键盘绑定并重建编码器组 */
static void admin_panel_show(admin_view_t view)
{
    if(!g_admin_unlocked && view != PASSWORD) {
        view = PASSWORD;
    }
    g_admin_view = view;

    if(g_admin_panel_pwd != NULL) lv_obj_add_flag(g_admin_panel_pwd, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_menu1 != NULL) lv_obj_add_flag(g_admin_panel_menu1, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_menu2 != NULL) lv_obj_add_flag(g_admin_panel_menu2, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_machine_id != NULL) lv_obj_add_flag(g_admin_panel_machine_id, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_program != NULL) lv_obj_add_flag(g_admin_panel_program, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_brightness != NULL) lv_obj_add_flag(g_admin_panel_brightness, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_sound != NULL) lv_obj_add_flag(g_admin_panel_sound, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_dormancy != NULL) lv_obj_add_flag(g_admin_panel_dormancy, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_lang != NULL) lv_obj_add_flag(g_admin_panel_lang, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_factory != NULL) lv_obj_add_flag(g_admin_panel_factory, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_contact != NULL) lv_obj_add_flag(g_admin_panel_contact, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_auto_dispense != NULL) lv_obj_add_flag(g_admin_panel_auto_dispense, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_fresh_air_care != NULL) lv_obj_add_flag(g_admin_panel_fresh_air_care, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_system_upgrade != NULL) lv_obj_add_flag(g_admin_panel_system_upgrade, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_payment != NULL) lv_obj_add_flag(g_admin_panel_payment, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_data != NULL) lv_obj_add_flag(g_admin_panel_data, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_network != NULL) lv_obj_add_flag(g_admin_panel_network, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_wifi != NULL) lv_obj_add_flag(g_admin_panel_wifi, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_4g != NULL) lv_obj_add_flag(g_admin_panel_4g, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_pwd_chg_old != NULL) lv_obj_add_flag(g_admin_panel_pwd_chg_old, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_pwd_chg_new != NULL) lv_obj_add_flag(g_admin_panel_pwd_chg_new, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_vendor_serial != NULL) lv_obj_add_flag(g_admin_panel_vendor_serial, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_vendor_menu != NULL) lv_obj_add_flag(g_admin_panel_vendor_menu, LV_OBJ_FLAG_HIDDEN);

    if(g_admin_lbl_msg_pwd != NULL) {
        lv_label_set_text(g_admin_lbl_msg_pwd, "");
        lv_obj_add_flag(g_admin_lbl_msg_pwd, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_lbl_msg_machine_id != NULL) {
        lv_label_set_text(g_admin_lbl_msg_machine_id, "");
        lv_obj_add_flag(g_admin_lbl_msg_machine_id, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_mid_success_overlay != NULL) {
        lv_obj_add_flag(g_admin_mid_success_overlay, LV_OBJ_FLAG_HIDDEN);
    }

    if(view == PASSWORD && g_admin_panel_pwd != NULL) {
        lv_obj_remove_flag(g_admin_panel_pwd, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
    }
    else if(view == MENU1 && g_admin_panel_menu1 != NULL) {
        lv_obj_remove_flag(g_admin_panel_menu1, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
    }
    else if(view == MENU2 && g_admin_panel_menu2 != NULL) {
        lv_obj_remove_flag(g_admin_panel_menu2, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
    }
    else if(view == MACHINE_ID && g_admin_panel_machine_id != NULL) {
        lv_obj_remove_flag(g_admin_panel_machine_id, LV_OBJ_FLAG_HIDDEN);
        admin_machine_id_label_update();
        if(g_admin_ta_machine_id != NULL) {
            char buf[8];
            if(g_machine_id > 0) {
                lv_snprintf(buf, sizeof(buf), "%06u", (unsigned)g_machine_id);
            } else {
                buf[0] = '\0';
            }
            lv_textarea_set_text(g_admin_ta_machine_id, buf);
        }
        if(g_admin_kb != NULL) {
            lv_obj_remove_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
            g_admin_kb_ta = g_admin_ta_machine_id;
        }
    }
    else if(view == PROGRAM_SETTINGS && g_admin_panel_program != NULL) {
        lv_obj_remove_flag(g_admin_panel_program, LV_OBJ_FLAG_HIDDEN);
        g_admin_ta_prog_active = NULL;
        if(g_group_admin != NULL) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, false);
        }
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        program_admin_show_list();
        program_admin_ui_load_fields();
    }
    else if(view == SCREEN_BRIGHTNESS && g_admin_panel_brightness != NULL) {
        lv_obj_remove_flag(g_admin_panel_brightness, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, false);
        }
        admin_brightness_sync_ui();
    }
    else if(view == SOUND_CONTROL && g_admin_panel_sound != NULL) {
        lv_obj_remove_flag(g_admin_panel_sound, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, false);
        }
        admin_sound_sync_ui();
    }
    else if(view == DORMANCY_STANDBY && g_admin_panel_dormancy != NULL) {
        lv_obj_remove_flag(g_admin_panel_dormancy, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_dormancy_tp_active) {
            admin_dormancy_hide_time_picker();
        }
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, false);
        }
        admin_dormancy_sync_switches();
    }
    else if(view == LANGUAGE_SETTINGS && g_admin_panel_lang != NULL) {
        lv_obj_remove_flag(g_admin_panel_lang, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, false);
        }
        admin_lang_sync_btn_ui();
    }
    else if(view == FACTORY_RESET && g_admin_panel_factory != NULL) {
        lv_obj_remove_flag(g_admin_panel_factory, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, false);
        }
        admin_factory_reset_ui_enter();
    }
    else if(view == CONTACT_US && g_admin_panel_contact != NULL) {
        lv_obj_remove_flag(g_admin_panel_contact, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, false);
        }
    }
    else if(view == AUTO_DISPENSE && g_admin_panel_auto_dispense != NULL) {
        lv_obj_remove_flag(g_admin_panel_auto_dispense, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, false);
        }
        admin_auto_dispense_sync_btn_ui();
    }
    else if(view == FRESH_AIR_CARE && g_admin_panel_fresh_air_care != NULL) {
        lv_obj_remove_flag(g_admin_panel_fresh_air_care, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, false);
        }
        admin_fresh_air_care_sync_btn_ui();
    }
    else if(view == SYSTEM_UPGRADE && g_admin_panel_system_upgrade != NULL) {
        lv_obj_remove_flag(g_admin_panel_system_upgrade, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, false);
        }
        admin_system_upgrade_ui_enter();
    }
    else if(view == PAYMENT_SETTINGS && g_admin_panel_payment != NULL) {
        lv_obj_remove_flag(g_admin_panel_payment, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        admin_payment_set_page(ADMIN_PAYMENT_PAGE_LIST);
        admin_payment_sync_list_ui();
        admin_payment_sync_list_ui();
    admin_payment_sync_method_ui();
        g_admin_payment_ui_loading = true;
        if(g_admin_sw_payment_method != NULL)
            lv_obj_remove_state(g_admin_sw_payment_method, LV_STATE_CHECKED);
        if(g_admin_sw_payment_timeout != NULL)
            lv_obj_remove_state(g_admin_sw_payment_timeout, LV_STATE_CHECKED);
        if(g_admin_sw_payment_order != NULL)
            lv_obj_remove_state(g_admin_sw_payment_order, LV_STATE_CHECKED);
        g_admin_payment_ui_loading = false;
    }
    else if(view == DATA_SETTINGS && g_admin_panel_data != NULL) {
        lv_obj_remove_flag(g_admin_panel_data, LV_OBJ_FLAG_HIDDEN);
        admin_data_set_page(ADMIN_DATA_PAGE_LIST);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        admin_data_sync_upload_items_ui();
        admin_data_sync_strategy_ui();
    }
    else if(view == NETWORK_SETTINGS && g_admin_panel_network != NULL) {
        lv_obj_remove_flag(g_admin_panel_network, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            if(g_group_admin) lv_group_set_editing(g_group_admin, false);
        }
    }
    else if(view == WIFI_SETTINGS && g_admin_panel_wifi != NULL) {
        lv_obj_remove_flag(g_admin_panel_wifi, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        admin_wifi_ui_enter();
    }
    else if(view == SETTINGS_4G && g_admin_panel_4g != NULL) {
        lv_obj_remove_flag(g_admin_panel_4g, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        admin_4g_ui_enter();
    }
    else if(view == PASSWORD_CHANGE_OLD && g_admin_panel_pwd_chg_old != NULL) {
        lv_obj_remove_flag(g_admin_panel_pwd_chg_old, LV_OBJ_FLAG_HIDDEN);
        admin_pwd_chg_hide_result();
        g_admin_pwd_chg_page1_step = 0;
        if(g_admin_ta_pwd_chg_old != NULL) lv_textarea_set_text(g_admin_ta_pwd_chg_old, "");
        if(g_admin_ta_pwd_chg_new1 != NULL) lv_textarea_set_text(g_admin_ta_pwd_chg_new1, "");
        if(g_admin_kb != NULL) {
            lv_obj_remove_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
            g_admin_kb_ta = g_admin_ta_pwd_chg_old;
        }
    }
    else if(view == PASSWORD_CHANGE_NEW && g_admin_panel_pwd_chg_new != NULL) {
        lv_obj_remove_flag(g_admin_panel_pwd_chg_new, LV_OBJ_FLAG_HIDDEN);
        admin_pwd_chg_hide_result();
        if(g_admin_ta_pwd_chg_new2 != NULL) lv_textarea_set_text(g_admin_ta_pwd_chg_new2, "");
        if(g_admin_kb != NULL && g_admin_ta_pwd_chg_new2 != NULL) {
            lv_obj_remove_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
            g_admin_kb_ta = g_admin_ta_pwd_chg_new2;
        }
    }
    else if(view == VENDOR_SERIAL && g_admin_panel_vendor_serial != NULL) {
        lv_obj_remove_flag(g_admin_panel_vendor_serial, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_ta_vendor_serial != NULL) {
            lv_textarea_set_text(g_admin_ta_vendor_serial, "");
        }
        if(g_admin_kb != NULL) {
            lv_obj_remove_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
            g_admin_kb_ta = g_admin_ta_vendor_serial;
        }
    }
    else if(view == VENDOR_MENU && g_admin_panel_vendor_menu != NULL) {
        lv_obj_remove_flag(g_admin_panel_vendor_menu, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
    }

    admin_encoder_rebuild();
}

/* 退出管理员会话：清除解锁并回到密码页 */
static void admin_session_reset(void)
{
    g_admin_unlocked = false;
    if(g_admin_ta_pwd != NULL) lv_textarea_set_text(g_admin_ta_pwd, "");
    if(g_admin_ta_machine_id != NULL) lv_textarea_set_text(g_admin_ta_machine_id, "");
    admin_panel_show(PASSWORD);
}

/* 校验 6 位管理员密码，成功则进入 8 宫格菜单 */
static void admin_password_try(void)
{
    if(g_admin_ta_pwd == NULL) return;
    const char * t = lv_textarea_get_text(g_admin_ta_pwd);
    if(t == NULL || lv_strlen(t) != 6) return;
    if(lv_strcmp(t, g_admin_pwd) != 0) {
        if(g_admin_lbl_msg_pwd != NULL) {
            g_admin_pwd_err_id = STR_PWD_WRONG_RETRY;
            lv_label_set_text(g_admin_lbl_msg_pwd, ui_translation(STR_PWD_WRONG_RETRY));
            lv_obj_remove_flag(g_admin_lbl_msg_pwd, LV_OBJ_FLAG_HIDDEN);
        }
        lv_textarea_set_text(g_admin_ta_pwd, "");
        if(g_admin_lbl_serial_title != NULL) lv_obj_add_flag(g_admin_lbl_serial_title, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_input_wrap != NULL) lv_obj_add_flag(g_admin_input_wrap, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        lv_obj_t * blocker = lv_obj_create(lv_layer_top());
        lv_obj_set_size(blocker, LV_PCT(100), LV_PCT(100));
        lv_obj_set_style_bg_opa(blocker, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(blocker, 0, LV_PART_MAIN);
        lv_obj_add_flag(blocker, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(blocker, cb_admin_err_blocker, LV_EVENT_CLICKED, NULL);
        return;
    }
    g_admin_unlocked = true;
    g_admin_pwd_err_id = STR_COUNT;
    lv_textarea_set_text(g_admin_ta_pwd, "");
    admin_panel_show(MENU1);
}

/* 密码修改：menu2 入口，进入原密码校验页 */
static void admin_machine_id_back_to_menu1(void)
{
	if(g_admin_ta_machine_id != NULL) lv_textarea_set_text(g_admin_ta_machine_id, "");
	if(g_admin_lbl_msg_machine_id != NULL) {
		lv_label_set_text(g_admin_lbl_msg_machine_id, "");
		lv_obj_add_flag(g_admin_lbl_msg_machine_id, LV_OBJ_FLAG_HIDDEN);
	}
	admin_panel_show(MENU1);
}

//管理员页：机器 ID 页确认：保存机器 ID 并更新标签
static bool admin_machine_id_apply(void)
{
    if(g_admin_ta_machine_id == NULL) return false;
    const char * t = lv_textarea_get_text(g_admin_ta_machine_id);
    if(t == NULL || lv_strlen(t) != 6) return false;

    for(size_t i = 0; t[i] != '\0'; i++) {
        if(t[i] < '0' || t[i] > '9') return false;
    }

    unsigned long val = 0;
    for(size_t i = 0; i < 6; i++) {
        val = val * 10u + (unsigned long)(t[i] - '0');
    }
    if(val > 999999u) return false;

    g_machine_id = (uint32_t)val;
    admin_machine_id_label_update();
    return true;
}

/* 输入框键盘 OK：密码页校验或机器 ID 页仅保存 */
static void cb_admin_ta_ready(lv_event_t * e)
{
	(void)e;
	if(g_admin_view == PASSWORD) {
		admin_password_try();
	} else if(g_admin_view == PASSWORD_CHANGE_OLD) {
		admin_pwd_chg_page1_on_ready();
	} else if(g_admin_view == PASSWORD_CHANGE_NEW) {
		admin_pwd_chg_page2_on_ready();
	} else if(g_admin_view == MACHINE_ID) {
		/* 键盘 OK：仅保存 ID，不切页；返回设置页请点「确认」或顶栏返回 */
		(void)admin_machine_id_apply();
	} else if(g_admin_view == VENDOR_SERIAL) {
		admin_vendor_serial_try();
	}
}

//管理员页：顶栏返回：子页回退或退出到主页
static void cb_admin_back(lv_event_t * e)
{
    (void)e;
    if(g_admin_view == MACHINE_ID) {
        admin_machine_id_back_to_menu1();
        return;
    }
    if(g_admin_view == PROGRAM_SETTINGS) {
        program_admin_back_to_menu1();
        return;
    }
    if(g_admin_view == SCREEN_BRIGHTNESS) {
        admin_brightness_back_to_menu1();
        return;
    }
    if(g_admin_view == SOUND_CONTROL) {
        admin_sound_back_to_menu2();
        return;
    }
    if(g_admin_view == DORMANCY_STANDBY) {
        if(g_admin_dormancy_tp_active) {
            admin_dormancy_hide_time_picker();
        } else {
            admin_dormancy_back_to_menu1();
        }
        return;
    }
    if(g_admin_view == LANGUAGE_SETTINGS) {
        admin_lang_back_to_menu2();
        return;
    }
    if(g_admin_view == FACTORY_RESET) {
        admin_factory_back_to_menu2();
        return;
    }
    if(g_admin_view == CONTACT_US) {
        admin_contact_back_to_menu2();
        return;
    }
    if(g_admin_view == AUTO_DISPENSE) {
        admin_auto_dispense_back_to_menu1();
        return;
    }
    if(g_admin_view == FRESH_AIR_CARE) {
        admin_fresh_air_care_back_to_menu1();
        return;
    }
    if(g_admin_view == SYSTEM_UPGRADE) {
        admin_system_upgrade_back_to_menu2();
        return;
    }
    if(g_admin_view == PAYMENT_SETTINGS) {
        if(g_admin_payment_page == ADMIN_PAYMENT_PAGE_TIMEOUT && g_admin_payment_tp_active) {
            admin_payment_hide_time_picker();
        }
        else if(g_admin_payment_page == ADMIN_PAYMENT_PAGE_ORDER_DETAIL) {
            admin_payment_set_page(ADMIN_PAYMENT_PAGE_ORDER_SUMMARY);
        }
        else if(g_admin_payment_page == ADMIN_PAYMENT_PAGE_ORDER_SUMMARY) {
            admin_payment_set_page(ADMIN_PAYMENT_PAGE_ORDERS);
        }
        else if(g_admin_payment_page == ADMIN_PAYMENT_PAGE_ORDERS ||
                g_admin_payment_page == ADMIN_PAYMENT_PAGE_METHOD) {
            admin_payment_set_page(ADMIN_PAYMENT_PAGE_LIST);
            admin_payment_sync_list_ui();
        }
        else {
            admin_payment_back_to_menu2();
        }
        return;
    }
    if(g_admin_view == DATA_SETTINGS) {
        admin_data_back_to_menu1();
        return;
    }
    if(g_admin_view == NETWORK_SETTINGS) {
        admin_network_back_to_menu1();
        return;
    }
    if(g_admin_view == WIFI_SETTINGS) {
        admin_wifi_back_to_network();
        return;
    }
    if(g_admin_view == SETTINGS_4G) {
        admin_4g_back_to_network();
        return;
    }
    if(g_admin_view == PASSWORD_CHANGE_NEW || g_admin_view == PASSWORD_CHANGE_OLD) {
        admin_pwd_chg_back_to_menu2();
        return;
    }
    if(g_admin_view == VENDOR_SERIAL) {
        admin_vendor_serial_back_to_menu1();
        return;
    }
    if(g_admin_view == VENDOR_MENU) {
        admin_vendor_menu_back_to_menu1();
        return;
    }
    if(g_admin_view == MENU2) {
        admin_panel_show(MENU1);
        return;
    }
    ui_screen_load(g_scr_home);
}

/* 管理员菜单进入机器 ID 设置子页 */
static void cb_admin_open_machine_id(lv_event_t * e)
{
	(void)e;
	admin_panel_show(MACHINE_ID);
}

static void cb_admin_machine_confirm(lv_event_t * e)
{
	(void)e;
	if(admin_machine_id_apply()) {
		/* 显示成功提示：点击任意处返回 MENU1，不自动消失 */
		if(g_admin_mid_success_overlay != NULL) {
			lv_obj_remove_flag(g_admin_mid_success_overlay, LV_OBJ_FLAG_HIDDEN);
			lv_obj_move_foreground(g_admin_mid_success_overlay);
		}
		if(g_admin_kb != NULL) {
			g_admin_kb_ta = NULL;
			lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
		}
	}
}


static void ui_idle_apply_dormancy_period(void)
{
    if(g_idle_timer == NULL) return;
    if(g_ui_dormancy_no_sleep || g_ui_dormancy_timeout_ms == UI_DORMANCY_DISABLED_MS) {
        lv_timer_pause(g_idle_timer);
        return;
    }
    lv_timer_set_period(g_idle_timer, g_ui_dormancy_timeout_ms);
    lv_timer_resume(g_idle_timer);
    ui_idle_reset();
}


/* 管理员页：程序设置 roller 退出编辑态 */
static void admin_prog_roller_exit_edit(lv_obj_t * roller)
{
    if(g_group_admin == NULL || roller == NULL) return;
    if((g_group_admin ? lv_group_get_focused(g_group_admin) : NULL) != roller) return;
    if(!(g_group_admin && lv_group_get_editing(g_group_admin))) return;
    uint32_t sel = lv_roller_get_selected(roller);
    lv_roller_set_selected(roller, sel, LV_ANIM_OFF);
    program_admin_ui_save_fields();
    if(g_group_admin) lv_group_set_editing(g_group_admin, false);
}

/*
 * 程序设置 roller：外设编码器短按发 LV_EVENT_CLICKED，在编辑/导航间切换。
 * 短按进入编辑（旋转改值），再按退回选择（旋转切焦点）；失焦时自动退出编辑。
 */
static void cb_admin_prog_roller_encoder(lv_event_t * e)
{
    lv_obj_t * roller = lv_event_get_target_obj(e);
    lv_event_code_t code = lv_event_get_code(e);

    if(g_admin_view != PROGRAM_SETTINGS) return;
    if(roller == NULL || lv_obj_has_flag(roller, LV_OBJ_FLAG_HIDDEN)) return;

    if(code == LV_EVENT_DEFOCUSED) {
        if(g_group_admin != NULL && (g_group_admin && lv_group_get_editing(g_group_admin))) {
            admin_prog_roller_exit_edit(roller);
        }
        return;
    }

    if(admin_kb_is_visible()) return;

    if(code != LV_EVENT_CLICKED) return;
    if(g_group_admin == NULL || (g_group_admin ? lv_group_get_focused(g_group_admin) : NULL) != roller) return;

    if((g_group_admin && lv_group_get_editing(g_group_admin))) {
        admin_prog_roller_exit_edit(roller);
    } else {
        uint32_t sel = lv_roller_get_selected(roller);
        lv_roller_set_selected(roller, sel, LV_ANIM_OFF);
        if(g_group_admin) lv_group_set_editing(g_group_admin, true);
    }
    lv_event_stop_processing(e);
}

/* 程序设置：绑定 textarea 并弹出数字键盘 */
static const char * ui_dormancy_timeout_label(uint32_t ms)
{
    uint32_t i;
    for(i = 0; i < DORMANCY_ROLLER_CNT; i++) {
        if(g_dormancy_timeout_ms_tbl[i] == ms) {
            return ui_translation(g_dormancy_label_ids[i]);
        }
    }
    return ui_translation(STR_DORM_5MIN);
}

/* 已保存的待机毫秒值 → roller 选项下标 */
static uint32_t ui_dormancy_roller_index_from_ms(uint32_t ms)
{
    for(uint32_t i = 0; i < DORMANCY_ROLLER_CNT; i++) {
        if(g_dormancy_timeout_ms_tbl[i] == ms) return i;
    }
    return DORMANCY_ROLLER_DEFAULT_IDX;
}

/* 刷新「当前：xx」标签（按 g_ui_dormancy_timeout_ms 已保存值） */


/* 进入待机时间页时：roller 选中项与「当前」标签对齐已保存配置 */


static void admin_dormancy_back_to_menu1(void)
{
    admin_panel_show(MENU1);
}


/* 待机 roller 数值变化：仅更新「当前」预览，不写 g_ui_dormancy_timeout_ms（确认键才保存） */


/*
 * 待机时间 roller 退出编码器编辑态。
 * 短按第二次编码器 / 失焦时调用；先同步 LVGL 内部 ori 再 editing=false，避免选项被回滚。
 */
static void admin_dormancy_roller_exit_edit(lv_obj_t * roller)
{
    if(g_group_admin == NULL || roller == NULL) return;
    if((g_group_admin ? lv_group_get_focused(g_group_admin) : NULL) != roller) return;
    if(!(g_group_admin && lv_group_get_editing(g_group_admin))) return;
    /* LVGL：editing→false 触发的 FOCUSED 会把选中项恢复为 sel_opt_id_ori */
    uint32_t sel = lv_roller_get_selected(roller);
    if(sel < DORMANCY_ROLLER_CNT) {
        lv_roller_set_selected(roller, sel, LV_ANIM_OFF);
    }
    if(g_group_admin) lv_group_set_editing(g_group_admin, false);
}

/*
 * 待机时间 roller：外设编码器短按发 LV_EVENT_CLICKED，在编辑/导航间切换。
 * 进页默认焦点在「确认」；聚焦 roller 后短按进入编辑，再短按退出；失焦时自动退出编辑。
 */


/* 待机时间「确认」：写入 g_ui_dormancy_timeout_ms、刷新空闲定时器并回菜单 */


/* 管理员菜单「待机时间」入口：进入待机时间设置子页 */
static void cb_admin_open_dormancy_standby(lv_event_t * e)
{
    (void)e;
    admin_panel_show(DORMANCY_STANDBY);
}

/*
 * 屏幕亮度 常亮开关可调参数（改后必须重新编译并运行新程序，仅保存文件不会生效）
 *
 * ADMIN_SW_SIZE_W / ADMIN_SW_SIZE_H
 *   开关控件整体外框宽高（lv_obj_set_size），改这两个才会变大/变长。
 *
 * ADMIN_SW_BORDER_W（描边宽度）
 *   轨道最外圈线的粗细（px）。关闭=白边，开启=橙边；越大边线越粗，占用的“视觉厚度”越大。
 *
 * ADMIN_SW_PAD_MAIN（轨道内边距）
 *   描边与内部黑色区域之间的空隙（px）。越大，内部填色区越小，圆圈活动空间略受影响。
 *
 * ADMIN_SW_KNOB_SIZE
 *   中间滑动圆点的目标直径（px），通过 KNOB 负 pad 实现，须 <= ADMIN_SW_SIZE_H。
 *
 * 圆角不单独定义：胶囊形自动为 ADMIN_SW_SIZE_H / 2。
 */
/* 按目标圆圈直径计算 LV_PART_KNOB 四边 pad（通常为负值，用于在固定控件高度下缩小圆圈） */
static lv_coord_t admin_sw_knob_pad_calc(lv_coord_t switch_h, lv_coord_t knob_diam)
{
    if(switch_h <= 0) {
        switch_h = ADMIN_SW_SIZE_H;
    }
    if(knob_diam > switch_h) {
        knob_diam = switch_h;
    }
    return (lv_coord_t)((knob_diam - switch_h) / 2);
}

/* 按目标圆圈直径计算 LV_PART_KNOB 四边 pad（通常为负值，用于在固定控件高度下缩小圆圈） */
static void admin_sw_apply_knob_pad(lv_obj_t * sw, lv_coord_t knob_diam, lv_style_selector_t selector)
{
    lv_coord_t h = lv_obj_get_height(sw);
    lv_coord_t pad = admin_sw_knob_pad_calc(h, knob_diam);
    lv_obj_set_style_pad_left(sw, pad, selector);
    lv_obj_set_style_pad_right(sw, pad, selector);
    lv_obj_set_style_pad_top(sw, pad, selector);
    lv_obj_set_style_pad_bottom(sw, pad, selector);
}

/* 管理员子页开关：仅颜色/描边/圆角（不设置宽高；不调用 remove_style 之后再 set_size） */
static void admin_panel_style_switch(lv_obj_t * sw)
{
    if(sw == NULL) return;

    lv_obj_remove_style_all(sw);

    const lv_coord_t radius_cap = ADMIN_SW_SIZE_H / 2;  /* 圆角用宏高度，apply_layout 里 set_size 后会再校正 */

    lv_obj_set_style_radius(sw, radius_cap, LV_PART_MAIN);
    lv_obj_set_style_pad_all(sw, ADMIN_SW_PAD_MAIN, LV_PART_MAIN);

    lv_obj_set_style_bg_opa(sw, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_color(sw, lv_color_hex(COL_BG), LV_PART_MAIN);
    lv_obj_set_style_border_width(sw, ADMIN_SW_BORDER_W, LV_PART_MAIN);
    lv_obj_set_style_border_color(sw, lv_color_hex(COL_TEXT), LV_PART_MAIN);

    lv_obj_set_style_bg_opa(sw, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(sw, lv_color_hex(COL_BG), LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(sw, ADMIN_SW_BORDER_W, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_border_color(sw, lv_color_hex(COL_ORANGE), LV_PART_MAIN | LV_STATE_CHECKED);

    lv_obj_set_style_bg_opa(sw, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(sw, LV_OPA_COVER, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(sw, lv_color_hex(COL_BG), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(sw, 0, LV_PART_INDICATOR);
    lv_obj_set_style_border_width(sw, 0, LV_PART_INDICATOR | LV_STATE_CHECKED);

    lv_obj_set_style_bg_opa(sw, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_bg_color(sw, lv_color_hex(COL_TEXT), LV_PART_KNOB);
    lv_obj_set_style_radius(sw, LV_RADIUS_CIRCLE, LV_PART_KNOB);
    lv_obj_set_style_border_width(sw, 0, LV_PART_KNOB);
    lv_obj_set_style_bg_color(sw, lv_color_hex(COL_ORANGE), LV_PART_KNOB | LV_STATE_CHECKED);
}

/* 按宏刷新开关尺寸、样式及与标题垂直对齐（右上角 -400） */
static void admin_panel_apply_switch_layout(lv_obj_t * sw, lv_obj_t * title_lbl)
{
    if(sw == NULL) return;

    /* 必须先样式再 set_size：remove_style_all 会清掉 WIDTH/HEIGHT，若在 set_size 之后调用则尺寸失效 */
    admin_panel_style_switch(sw);
    lv_obj_set_size(sw, ADMIN_SW_SIZE_W, ADMIN_SW_SIZE_H);
    lv_obj_refr_size(sw);

    const lv_coord_t radius_cap = lv_obj_get_height(sw) / 2;
    lv_obj_set_style_radius(sw, radius_cap, LV_PART_MAIN);
    lv_obj_update_layout(sw);
    lv_coord_t radius_ind = lv_obj_get_content_height(sw) / 2;
    if(radius_ind < 0) {
        radius_ind = 0;
    }
    lv_obj_set_style_radius(sw, radius_ind, LV_PART_INDICATOR);

    admin_sw_apply_knob_pad(sw, ADMIN_SW_KNOB_SIZE, LV_PART_KNOB);

    lv_obj_align(sw, LV_ALIGN_TOP_RIGHT, -400, 0);
    if(title_lbl != NULL) {
        lv_obj_update_layout(title_lbl);
        lv_coord_t ty = lv_obj_get_y(title_lbl);
        lv_coord_t th = lv_obj_get_height(title_lbl);
        lv_coord_t sh = lv_obj_get_height(sw);
        lv_obj_set_y(sw, ty + (th - sh) / 2);
    }
    lv_obj_invalidate(sw);
}

/* 屏幕亮度页：刷新常亮开关布局 */
static void admin_brightness_apply_switch_layout(void)
{
    admin_data_style_switch(g_admin_sw_run_always_on);
}

static void admin_4g_apply_switch_layout(void)
{
    if(g_admin_sw_4g == NULL) return;

    admin_panel_style_switch(g_admin_sw_4g);
    lv_obj_set_size(g_admin_sw_4g, ADMIN_SW_SIZE_W, ADMIN_SW_SIZE_H);
    lv_obj_refr_size(g_admin_sw_4g);

    const lv_coord_t radius_cap = lv_obj_get_height(g_admin_sw_4g) / 2;
    lv_obj_set_style_radius(g_admin_sw_4g, radius_cap, LV_PART_MAIN);
    lv_obj_update_layout(g_admin_sw_4g);
    lv_coord_t radius_ind = lv_obj_get_content_height(g_admin_sw_4g) / 2;
    if(radius_ind < 0) {
        radius_ind = 0;
    }
    lv_obj_set_style_radius(g_admin_sw_4g, radius_ind, LV_PART_INDICATOR);

    admin_sw_apply_knob_pad(g_admin_sw_4g, ADMIN_SW_KNOB_SIZE, LV_PART_KNOB);

    /* 自由定位：不跟随标题，直接设置 x/y（相对 g_admin_panel_4g 左上角） */
    lv_obj_set_pos(g_admin_sw_4g, 1200, 150);
    lv_obj_invalidate(g_admin_sw_4g);
}


/* 亮度滑条数值对齐到步进 */
static int32_t admin_brightness_snap_slider(int32_t v)
{
    if(v < 0) v = 0;
    if(v > 100) v = 100;
    return (int32_t)ui_screen_brightness_snap((uint8_t)v);
}

static void admin_brightness_sync_switch_ui(void)
{
    if(g_admin_sw_run_always_on == NULL) return;
    if(ui_screen_run_always_on_get()) {
        lv_obj_add_state(g_admin_sw_run_always_on, LV_STATE_CHECKED);
    }
    else {
        lv_obj_remove_state(g_admin_sw_run_always_on, LV_STATE_CHECKED);
    }
}


/* 同步下层渐变动条（与滑条数值一致） */


/* 将编码器焦点态映射到上层焦点框 */




static void admin_brightness_sync_slider_ui(void)
{
    if(g_admin_slider_brightness == NULL) return;
    g_admin_brightness_ui_loading = true;
    int32_t v = (int32_t)ui_screen_brightness_get();
    lv_slider_set_value(g_admin_slider_brightness, v, LV_ANIM_OFF);
    g_admin_brightness_ui_loading = false;
    admin_brightness_fill_sync();
}


static void admin_brightness_sync_ui(void)
{
    admin_brightness_apply_switch_layout();
    admin_brightness_sync_switch_ui();
    admin_brightness_sync_slider_ui();
}


static void cb_admin_brightness_switch_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(g_admin_view != SCREEN_BRIGHTNESS) return;
    lv_obj_t * sw = lv_event_get_target_obj(e);
    bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);
    ui_screen_run_always_on_set(on);
}


static void cb_admin_brightness_slider_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(g_admin_brightness_ui_loading) return;
    if(g_admin_view != SCREEN_BRIGHTNESS) return;
    lv_obj_t * slider = lv_event_get_target_obj(e);
    int32_t snapped = admin_brightness_snap_slider(lv_slider_get_value(slider));
    if(snapped != lv_slider_get_value(slider)) {
        g_admin_brightness_ui_loading = true;
        lv_slider_set_value(slider, snapped, LV_ANIM_OFF);
        g_admin_brightness_ui_loading = false;
    }
    ui_screen_brightness_set((uint8_t)snapped);
    admin_brightness_fill_sync();
}








/* 离开屏幕亮度子页：回管理员 8 宫格菜单 */
static void admin_brightness_back_to_menu1(void)
{
    if(g_group_admin != NULL) {
        if(g_group_admin) lv_group_set_editing(g_group_admin, false);
    }
    admin_panel_show(MENU2);
}

/* 管理员菜单「屏幕亮度」入口 */
static void cb_admin_open_brightness(lv_event_t * e)
{
    (void)e;
    admin_panel_show(SCREEN_BRIGHTNESS);
}

/* 声音页：开关样式 + X 与常亮开关一致，Y 垂直居中于 anchor_lbl */
static void admin_sound_apply_switch_layout(lv_obj_t * sw, lv_obj_t * anchor_lbl)
{
    if(sw == NULL) return;

    admin_panel_style_switch(sw);
    lv_obj_set_size(sw, ADMIN_SW_SIZE_W, ADMIN_SW_SIZE_H);
    lv_obj_refr_size(sw);

    const lv_coord_t radius_cap = lv_obj_get_height(sw) / 2;
    lv_obj_set_style_radius(sw, radius_cap, LV_PART_MAIN);
    lv_obj_update_layout(sw);
    lv_coord_t radius_ind = lv_obj_get_content_height(sw) / 2;
    if(radius_ind < 0) {
        radius_ind = 0;
    }
    lv_obj_set_style_radius(sw, radius_ind, LV_PART_INDICATOR);
    admin_sw_apply_knob_pad(sw, ADMIN_SW_KNOB_SIZE, LV_PART_KNOB);

    lv_obj_align(sw, LV_ALIGN_TOP_RIGHT, -400, 0);
    if(anchor_lbl != NULL) {
        lv_obj_update_layout(anchor_lbl);
        lv_coord_t ty = lv_obj_get_y(anchor_lbl);
        lv_coord_t th = lv_obj_get_height(anchor_lbl);
        lv_coord_t sh = lv_obj_get_height(sw);
        lv_obj_set_y(sw, ty + (th - sh) / 2);
    }
    lv_obj_invalidate(sw);
}

static void admin_sound_sync_touch_switch_ui(void)
{
    if(g_admin_sw_touch_sound == NULL) return;
    if(ui_touch_sound_get()) {
        lv_obj_add_state(g_admin_sw_touch_sound, LV_STATE_CHECKED);
    }
    else {
        lv_obj_remove_state(g_admin_sw_touch_sound, LV_STATE_CHECKED);
    }
}


/* 声音控制页：按 ui_voice_broadcast_get 刷新声音播报开关 */
static void admin_sound_sync_voice_broadcast_switch_ui(void)
{
    if(g_admin_sw_voice_broadcast == NULL) return;
    if(ui_voice_broadcast_get()) {
        lv_obj_add_state(g_admin_sw_voice_broadcast, LV_STATE_CHECKED);
    }
    else {
        lv_obj_remove_state(g_admin_sw_voice_broadcast, LV_STATE_CHECKED);
    }
}


/* 音量滑条数值对齐到步进 */
static int32_t admin_sound_snap_volume_slider(int32_t v)
{
    if(v < 0) v = 0;
    if(v > 100) v = 100;
    return (int32_t)ui_sound_volume_snap((uint8_t)v);
}

/* 触控声音滑条数值对齐到步进 */
static int32_t admin_sound_snap_touch_sound_volume_slider(int32_t v)
{
    if(v < 0) v = 0;
    if(v > 100) v = 100;
    return (int32_t)ui_touch_sound_volume_snap((uint8_t)v);
}

/* 同步播报音量进度填充（亮度样式） */
static void admin_sound_volume_slider_sync_bar(int32_t v)
{
    (void)v;
    admin_bright_style_fill_sync(g_admin_sound_volume_fill, g_admin_sound_volume_fill_grad,
                                 g_admin_slider_sound_volume);
}


/* 同步触控声音进度填充（亮度样式） */
static void admin_sound_touch_sound_volume_slider_sync_bar(int32_t v)
{
    (void)v;
    admin_bright_style_fill_sync(g_admin_touch_sound_volume_fill, g_admin_touch_sound_volume_fill_grad,
                                 g_admin_slider_touch_sound_volume);
}


/* 将编码器焦点态映射到音量滑条上层焦点框 */


/* 将编码器焦点态映射到触控声音滑条上层焦点框 */






/* 声音控制页：按 ui_sound_volume_get 刷新音量滑条（不触发硬件回调） */
static void admin_sound_sync_volume_slider_ui(void)
{
    if(g_admin_slider_sound_volume == NULL) return;
    g_admin_sound_volume_ui_loading = true;
    int32_t v = (int32_t)ui_sound_volume_get();
    lv_slider_set_value(g_admin_slider_sound_volume, v, LV_ANIM_OFF);
    g_admin_sound_volume_ui_loading = false;
    admin_sound_volume_slider_sync_bar(v);
}


/* 声音控制页：按 ui_touch_sound_volume_get 刷新触控声音滑条（不触发硬件回调） */
static void admin_sound_sync_touch_sound_volume_slider_ui(void)
{
    if(g_admin_slider_touch_sound_volume == NULL) return;
    g_admin_touch_sound_volume_ui_loading = true;
    int32_t v = (int32_t)ui_touch_sound_volume_get();
    lv_slider_set_value(g_admin_slider_touch_sound_volume, v, LV_ANIM_OFF);
    g_admin_touch_sound_volume_ui_loading = false;
    admin_sound_touch_sound_volume_slider_sync_bar(v);
}


/* 声音控制页：刷新两个开关与两条滑条 */
static void admin_sound_sync_ui(void)
{
    admin_sound_sync_touch_switch_ui();
    admin_sound_sync_voice_broadcast_switch_ui();
    admin_sound_sync_volume_slider_ui();
    admin_sound_sync_touch_sound_volume_slider_ui();
}


/* 触控声音开关切换：更新状态并通知硬件（本期不播放音效） */
static void cb_admin_sound_touch_switch_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(g_admin_view != SOUND_CONTROL) return;
    lv_obj_t * sw = lv_event_get_target_obj(e);
    bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);
    ui_touch_sound_set(on);
}


/* 声音播报开关切换：更新状态并通知硬件（本期不播放语音） */
static void cb_admin_sound_voice_broadcast_switch_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(g_admin_view != SOUND_CONTROL) return;
    lv_obj_t * sw = lv_event_get_target_obj(e);
    bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);
    ui_voice_broadcast_set(on);
}


/* 音量滑条拖动：按步进 1 对齐并通知硬件 */
static void cb_admin_sound_volume_slider_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(g_admin_sound_volume_ui_loading) return;
    if(g_admin_view != SOUND_CONTROL) return;
    lv_obj_t * slider = lv_event_get_target_obj(e);
    int32_t snapped = admin_sound_snap_volume_slider(lv_slider_get_value(slider));
    if(snapped != lv_slider_get_value(slider)) {
        g_admin_sound_volume_ui_loading = true;
        lv_slider_set_value(slider, snapped, LV_ANIM_OFF);
        g_admin_sound_volume_ui_loading = false;
    }
    ui_sound_volume_set((uint8_t)snapped);
    admin_sound_volume_slider_sync_bar(snapped);
}


/* 触控声音滑条拖动：按步进 1 对齐并通知硬件 */
static void cb_admin_sound_touch_sound_volume_slider_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(g_admin_touch_sound_volume_ui_loading) return;
    if(g_admin_view != SOUND_CONTROL) return;
    lv_obj_t * slider = lv_event_get_target_obj(e);
    int32_t snapped = admin_sound_snap_touch_sound_volume_slider(lv_slider_get_value(slider));
    if(snapped != lv_slider_get_value(slider)) {
        g_admin_touch_sound_volume_ui_loading = true;
        lv_slider_set_value(slider, snapped, LV_ANIM_OFF);
        g_admin_touch_sound_volume_ui_loading = false;
    }
    ui_touch_sound_volume_set((uint8_t)snapped);
    admin_sound_touch_sound_volume_slider_sync_bar(snapped);
}














static void admin_sound_back_to_menu2(void)
{
    admin_panel_show(MENU2);
}


/* 管理员菜单「声音控制」入口 */
static void cb_admin_open_sound(lv_event_t * e)
{
    (void)e;
    admin_panel_show(SOUND_CONTROL);
}

static void admin_lang_btn_set_selected(lv_obj_t * btn, bool selected)
{
    if(btn == NULL) return;
    lv_obj_t * lbl = lv_obj_get_child(btn, 0);
    if(selected) {
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_bg_color(btn, lv_color_hex(COL_ORANGE), LV_PART_MAIN);
        lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN);
        if(lbl != NULL) {
            lv_obj_set_style_text_color(lbl, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        }
    }
    else {
        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(btn, 2, LV_PART_MAIN);
        lv_obj_set_style_border_color(btn, lv_color_hex(COL_ORANGE), LV_PART_MAIN);
        if(lbl != NULL) {
            lv_obj_set_style_text_color(lbl, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        }
    }
}


static void admin_lang_sync_btn_ui(void)
{
    bool zh = (ui_lang_get() == UI_LANG_ZH);
    admin_lang_btn_set_selected(g_admin_btn_lang_zh, zh);
    admin_lang_btn_set_selected(g_admin_btn_lang_en, !zh);
}


/* 离开语言设置页：回管理员 menu2 */
static void admin_lang_back_to_menu2(void)
{
    admin_panel_show(MENU2);
}

/* 管理员菜单「语言设置」入口 */
static void cb_admin_open_language_settings(lv_event_t * e)
{
    (void)e;
    admin_panel_show(LANGUAGE_SETTINGS);
}

static void cb_admin_lang_zh(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != LANGUAGE_SETTINGS) return;
    if(ui_lang_get() == UI_LANG_ZH) return;
    ui_lang_set(UI_LANG_ZH);
    admin_lang_sync_btn_ui();
}


static void cb_admin_lang_en(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != LANGUAGE_SETTINGS) return;
    if(ui_lang_get() == UI_LANG_EN) return;
    ui_lang_set(UI_LANG_EN);
    admin_lang_sync_btn_ui();
}


/* 停止恢复默认页的 2s 完成态定时器（离开页或再次进入前调用） */
static void admin_factory_timer_stop(void)
{
    if(g_admin_factory_timer != NULL) {
        lv_timer_delete(g_admin_factory_timer);
        g_admin_factory_timer = NULL;
    }
}

/* 执行出厂恢复：5 程序配置、待机 5 分钟、机器 ID 000001、语言中文，并刷新主页/支付展示 */
static void admin_factory_run_restore(void)
{
    program_admin_init_factory();
    g_ui_dormancy_timeout_ms = UI_DORMANCY_TIMEOUT_DEFAULT_MS;
    g_ui_dormancy_no_sleep = false;
    ui_idle_apply_dormancy_period();
    g_machine_id = 1u;
    g_ui_lang = UI_LANG_ZH;
    ui_lang_apply_all();
    ui_auto_dispense_set(false);
    ui_fresh_air_care_set(true);
    ui_screen_run_always_on_set(true);
    ui_screen_brightness_set(100);
    ui_touch_sound_set(true);
    ui_voice_broadcast_set(true);
    ui_sound_volume_set(100);
    ui_touch_sound_volume_set(100);
    home_sync_program_labels();
    pay_sync_price_label();
    lv_strcpy(g_admin_pwd, ADMIN_PWD_DEFAULT);
    ui_payment_alipay_set(true);
    ui_payment_wechat_set(true);
    ui_payment_timeout_sec_set(180);
    ui_4g_set(true);
    ui_wifi_set(false);
    ui_data_upload_basic_set(true);
    ui_data_upload_sensor_set(true);
    ui_data_upload_fault_set(true);
    ui_data_upload_auto_dispense_set(false);
    ui_data_upload_payment_order_set(false);
    ui_data_upload_user_op_set(false);
    ui_data_upload_device_set(true);
    ui_data_upload_strategy_reset_all(UI_DATA_STRATEGY_4G_ONLY);
}

static void admin_factory_set_phase(admin_factory_phase_t phase)
{
    g_admin_factory_phase = phase;
    if(g_admin_view != FACTORY_RESET) return;

    const bool prompt = (phase == ADMIN_FACTORY_PHASE_PROMPT);

    if(g_admin_lbl_factory_line1 != NULL) {
        if(prompt) lv_obj_remove_flag(g_admin_lbl_factory_line1, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_lbl_factory_line1, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_lbl_factory_line2 != NULL) {
        if(prompt) lv_obj_remove_flag(g_admin_lbl_factory_line2, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_lbl_factory_line2, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_btn_factory_ok != NULL) {
        if(prompt) lv_obj_remove_flag(g_admin_btn_factory_ok, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_btn_factory_ok, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_btn_factory_cancel != NULL) {
        if(prompt) lv_obj_remove_flag(g_admin_btn_factory_cancel, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_btn_factory_cancel, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_lbl_factory_status != NULL) {
        if(prompt) {
            lv_obj_add_flag(g_admin_lbl_factory_status, LV_OBJ_FLAG_HIDDEN);
        }
        else {
            lv_obj_remove_flag(g_admin_lbl_factory_status, LV_OBJ_FLAG_HIDDEN);
            if(phase == ADMIN_FACTORY_PHASE_RESTORING) {
                lv_label_set_text(g_admin_lbl_factory_status, ui_translation(STR_FACTORY_RESTORING));
            }
            else {
                lv_label_set_text(g_admin_lbl_factory_status, ui_translation(STR_FACTORY_DONE));
            }
            lv_obj_center(g_admin_lbl_factory_status);
        }
    }

}


static void admin_factory_reset_ui_enter(void)
{
    admin_factory_timer_stop();
    admin_factory_set_phase(ADMIN_FACTORY_PHASE_PROMPT);
}


/* 离开恢复默认页：回管理员 menu2 */
static void admin_factory_back_to_menu2(void)
{
    admin_factory_timer_stop();
    if(g_group_admin != NULL) {
        if(g_group_admin) lv_group_set_editing(g_group_admin, false);
    }
    admin_panel_show(MENU2);
}

/* 恢复默认「取消」：返回管理员菜单 */
static void cb_admin_factory_cancel(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    admin_factory_back_to_menu2();
}

static void cb_admin_factory_timer(lv_timer_t * t)
{
    (void)t;
    g_admin_factory_timer = NULL;
    if(g_admin_view == FACTORY_RESET) {
        admin_factory_set_phase(ADMIN_FACTORY_PHASE_DONE);
    }
}


static void cb_admin_factory_ok(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != FACTORY_RESET) return;

    admin_factory_timer_stop();
    admin_factory_set_phase(ADMIN_FACTORY_PHASE_RESTORING);
    admin_factory_run_restore();

    g_admin_factory_timer = lv_timer_create(cb_admin_factory_timer, 2000, NULL);
    lv_timer_set_repeat_count(g_admin_factory_timer, 1);
}


/* 管理员菜单「恢复默认」入口 */
static void cb_admin_open_factory_reset(lv_event_t * e)
{
    (void)e;
    admin_panel_show(FACTORY_RESET);
}

/* 离开联系我们页：回管理员 menu2 */
static void admin_contact_back_to_menu2(void)
{
    admin_panel_show(MENU2);
}

/* 管理员菜单「联系我们」入口 */
static void cb_admin_open_contact_us(lv_event_t * e)
{
    (void)e;
    admin_panel_show(CONTACT_US);
}

/* 防缠绕功能页：按 ui_auto_dispense_get 刷新「开启」「关闭」橙色填充/描边 */
static void admin_auto_dispense_sync_btn_ui(void)
{
    bool on = ui_auto_dispense_get();
    admin_lang_btn_set_selected(g_admin_btn_auto_dispense_on, on);
    admin_lang_btn_set_selected(g_admin_btn_auto_dispense_off, !on);
}

/* 离开防缠绕功能页：回管理员 menu1 */
static void admin_auto_dispense_back_to_menu1(void)
{
    admin_panel_show(MENU1);
}

/* 管理员 menu1「防缠绕功能」入口 */
static void cb_admin_open_auto_dispense(lv_event_t * e)
{
    (void)e;
    admin_panel_show(AUTO_DISPENSE);
}

/* 防缠绕功能「开启」：已是开启则无操作，否则 set 并通知硬件 */
static void cb_admin_auto_dispense_on(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != AUTO_DISPENSE) return;
    (void)ui_auto_dispense_set(true);
}

/* 防缠绕功能「关闭」：已是关闭则无操作，否则 set 并通知硬件 */
static void cb_admin_auto_dispense_off(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != AUTO_DISPENSE) return;
    (void)ui_auto_dispense_set(false);
}

/* 新风护理页：按 ui_fresh_air_care_get 刷新「开启」「关闭」橙色填充/描边 */
static void admin_fresh_air_care_sync_btn_ui(void)
{
    bool on = ui_fresh_air_care_get();
    admin_lang_btn_set_selected(g_admin_btn_fresh_air_care_on, on);
    admin_lang_btn_set_selected(g_admin_btn_fresh_air_care_off, !on);
}

/* 离开新风护理页：回管理员 menu1 */
static void admin_fresh_air_care_back_to_menu1(void)
{
    admin_panel_show(MENU1);
}

/* 管理员 menu2「新风护理」入口 */
static void cb_admin_open_fresh_air_care(lv_event_t * e)
{
    (void)e;
    admin_panel_show(FRESH_AIR_CARE);
}

/* 新风护理「开启」：已是开启则无操作，否则 set 并通知硬件 */
static void cb_admin_fresh_air_care_on(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != FRESH_AIR_CARE) return;
    (void)ui_fresh_air_care_set(true);
}

/* 新风护理「关闭」：已是关闭则无操作，否则 set 并通知硬件 */
static void cb_admin_fresh_air_care_off(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != FRESH_AIR_CARE) return;
    (void)ui_fresh_air_care_set(false);
}

/* 停止系统升级页的 2s 完成态定时器（离开页或再次进入前调用） */
static void admin_system_upgrade_timer_stop(void)
{
    if(g_admin_system_upgrade_timer != NULL) {
        lv_timer_delete(g_admin_system_upgrade_timer);
        g_admin_system_upgrade_timer = NULL;
    }
}

static void admin_system_upgrade_set_phase(admin_system_upgrade_phase_t phase)
{
    g_admin_system_upgrade_phase = phase;
    if(g_admin_view != SYSTEM_UPGRADE) return;

    const bool prompt = (phase == ADMIN_SYSTEM_UPGRADE_PHASE_PROMPT);

    if(g_admin_lbl_system_upgrade_line1 != NULL) {
        if(prompt) lv_obj_remove_flag(g_admin_lbl_system_upgrade_line1, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_lbl_system_upgrade_line1, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_btn_system_upgrade_ok != NULL) {
        if(prompt) lv_obj_remove_flag(g_admin_btn_system_upgrade_ok, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_btn_system_upgrade_ok, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_lbl_system_upgrade_status != NULL) {
        if(prompt) {
            lv_obj_add_flag(g_admin_lbl_system_upgrade_status, LV_OBJ_FLAG_HIDDEN);
        }
        else {
            lv_obj_remove_flag(g_admin_lbl_system_upgrade_status, LV_OBJ_FLAG_HIDDEN);
            if(phase == ADMIN_SYSTEM_UPGRADE_PHASE_UPGRADING) {
                lv_label_set_text(g_admin_lbl_system_upgrade_status, ui_translation(STR_UPGRADE_IN_PROGRESS));
            }
            else {
                lv_label_set_text(g_admin_lbl_system_upgrade_status, ui_translation(STR_UPGRADE_LATEST));
            }
            lv_obj_center(g_admin_lbl_system_upgrade_status);
        }
    }

}


static void admin_system_upgrade_ui_enter(void)
{
    admin_system_upgrade_timer_stop();
    admin_system_upgrade_set_phase(ADMIN_SYSTEM_UPGRADE_PHASE_PROMPT);
}


/* 离开系统升级页：回管理员 menu2 */
static void admin_system_upgrade_back_to_menu2(void)
{
    admin_system_upgrade_timer_stop();
    if(g_group_admin != NULL) {
        if(g_group_admin) lv_group_set_editing(g_group_admin, false);
    }
    admin_panel_show(MENU2);
}

static void cb_admin_system_upgrade_timer(lv_timer_t * t)
{
    (void)t;
    g_admin_system_upgrade_timer = NULL;
    if(g_admin_view == SYSTEM_UPGRADE) {
        admin_system_upgrade_set_phase(ADMIN_SYSTEM_UPGRADE_PHASE_DONE);
    }
}


static void cb_admin_system_upgrade_ok(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != SYSTEM_UPGRADE) return;

    admin_system_upgrade_timer_stop();
    admin_system_upgrade_set_phase(ADMIN_SYSTEM_UPGRADE_PHASE_UPGRADING);

    g_admin_system_upgrade_timer = lv_timer_create(cb_admin_system_upgrade_timer, 2000, NULL);
    lv_timer_set_repeat_count(g_admin_system_upgrade_timer, 1);
}


/* 管理员 menu2「系统升级」入口 */
static void cb_admin_open_system_upgrade(lv_event_t * e)
{
    (void)e;
    admin_panel_show(SYSTEM_UPGRADE);
}

static void admin_4g_timer_stop(void)
{
    if(g_admin_4g_timer != NULL) {
        lv_timer_delete(g_admin_4g_timer);
        g_admin_4g_timer = NULL;
    }
}


static void admin_4g_sync_switch_ui(void)
{
    if(g_admin_sw_4g == NULL) return;
    g_admin_4g_ui_loading = true;
    if(ui_4g_get()) {
        lv_obj_add_state(g_admin_sw_4g, LV_STATE_CHECKED);
    }
    else {
        lv_obj_remove_state(g_admin_sw_4g, LV_STATE_CHECKED);
    }
    g_admin_4g_ui_loading = false;
}


static void admin_4g_set_phase(admin_4g_phase_t phase)
{
    g_admin_4g_phase = phase;
    if(g_admin_view != SETTINGS_4G) return;

    const bool prompt = (phase == ADMIN_4G_PHASE_PROMPT);
    const bool provisioning = (phase == ADMIN_4G_PHASE_PROVISIONING);
    const bool result_page = (phase == ADMIN_4G_PHASE_SUCCESS || phase == ADMIN_4G_PHASE_FAILURE);

    if(g_admin_lbl_4g_title != NULL) {
        if(result_page) lv_obj_add_flag(g_admin_lbl_4g_title, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_remove_flag(g_admin_lbl_4g_title, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_img_4g_title_box != NULL) {
        if(result_page) lv_obj_add_flag(g_admin_img_4g_title_box, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_remove_flag(g_admin_img_4g_title_box, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_sw_4g != NULL) {
        if(result_page) lv_obj_add_flag(g_admin_sw_4g, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_remove_flag(g_admin_sw_4g, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_lbl_4g_prompt != NULL) {
        if(prompt) lv_obj_remove_flag(g_admin_lbl_4g_prompt, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_lbl_4g_prompt, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_lbl_4g_status != NULL) {
        if(provisioning) {
            lv_label_set_text(g_admin_lbl_4g_status, ui_translation(STR_4G_PROVISIONING));
            lv_obj_remove_flag(g_admin_lbl_4g_status, LV_OBJ_FLAG_HIDDEN);
        }
        else {
            lv_obj_add_flag(g_admin_lbl_4g_status, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if(g_admin_4g_done_center != NULL) {
        if(result_page) {
            lv_obj_remove_flag(g_admin_4g_done_center, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(g_admin_4g_done_center);
            if(g_admin_img_4g_done != NULL) {
                lv_image_set_src(g_admin_img_4g_done,
                    (phase == ADMIN_4G_PHASE_SUCCESS) ? &success : &failure);
            }
            if(g_admin_lbl_4g_done != NULL) {
                lv_label_set_text(g_admin_lbl_4g_done,
                    ui_translation((phase == ADMIN_4G_PHASE_SUCCESS) ? STR_4G_SUCCESS : STR_4G_FAIL));
            }
        } else {
            lv_obj_add_flag(g_admin_4g_done_center, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if(g_admin_4g_set_box_wrap != NULL) {
        if(result_page) lv_obj_add_flag(g_admin_4g_set_box_wrap, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_remove_flag(g_admin_4g_set_box_wrap, LV_OBJ_FLAG_HIDDEN);
    }

}


/* 待机/支付 roller 选项随语言切换（保持当前选中项） */
static void ui_lang_refresh_rollers(void)
{
}

/* 程序设置页：字段名与程序 Tab 文案 */
static void program_admin_refresh_i18n(void)
{
	unsigned i;
	for(i = 0; i < TOTAL_PROGRAMS; i++) {
		if(g_admin_prog_btns[i] != NULL) {
			if(g_admin_prog_btn_lbls[i] != NULL) {
				lv_label_set_text(g_admin_prog_btn_lbls[i], ui_translation(g_mode_name_ids[i]));
			}
		}
	}
	for(i = 0; i < PROG_ADMIN_FIELD_CNT; i++) {
		if(g_admin_prog_field_lbl[i] != NULL && g_admin_prog_field_box[i] != NULL) {
			lv_obj_align_to(g_admin_prog_field_lbl[i], g_admin_prog_field_box[i],
			                LV_ALIGN_OUT_TOP_MID, 0, -PROG_ADMIN_LBL_BOX_GAP);
		}
	}
	if(g_admin_prog_sel >= 0 && g_admin_prog_sel < TOTAL_PROGRAMS) {
		program_admin_update_cool_display(&g_prog_cfg[g_admin_prog_sel]);
	}
}

/* 恢复出厂/系统升级/4G 等可见动态状态文案 */
static void admin_refresh_visible_status_text(void)
{
    if(g_admin_lbl_msg_pwd != NULL && g_admin_pwd_err_id < STR_COUNT
       && !lv_obj_has_flag(g_admin_lbl_msg_pwd, LV_OBJ_FLAG_HIDDEN)) {
        lv_label_set_text(g_admin_lbl_msg_pwd, ui_translation(g_admin_pwd_err_id));
    }
    if(g_admin_lbl_pwd_chg_result != NULL && g_admin_pwd_chg_new_msg_id < STR_COUNT
       && !lv_obj_has_flag(g_admin_pwd_chg_result, LV_OBJ_FLAG_HIDDEN)) {
        lv_label_set_text(g_admin_lbl_pwd_chg_result, ui_translation(g_admin_pwd_chg_new_msg_id));
    }
    if(g_admin_view == FACTORY_RESET) {
        admin_factory_set_phase(g_admin_factory_phase);
    }
    if(g_admin_view == SYSTEM_UPGRADE) {
        admin_system_upgrade_set_phase(g_admin_system_upgrade_phase);
    }
    if(g_admin_view == SETTINGS_4G) {
        admin_4g_set_phase(g_admin_4g_phase);
    }
    if(g_admin_view == WIFI_SETTINGS) {
        admin_wifi_set_phase(g_admin_wifi_phase);
    }
}

/* 切换语言后刷新：已绑定标签 + 轮播名 + 运行页 + 管理员子页 */
static void ui_lang_apply_all(void)
{
    unsigned i;
    for(i = 0; i < g_lang_bind_count; i++) {
        if(g_lang_binds[i].lbl != NULL) {
            lv_label_set_text(g_lang_binds[i].lbl, ui_translation(g_lang_binds[i].id));
        }
    }
    ui_lang_refresh_rollers();
    admin_machine_id_label_update();
    program_admin_refresh_i18n();
    admin_payment_sync_list_ui();
    admin_payment_sync_method_ui();
    admin_data_sync_upload_items_ui();
    admin_data_sync_strategy_ui();
    admin_refresh_visible_status_text();
    carousel_update_card_images();
    running_screen_sync_mode_name();
    running_status_sync_labels();
    alarm_content_update();
    alarm_fault_panels_relayout_content();                                 //切换语言后重排故障页内容
    home_sync_program_labels();
    pay_sync_pay_ui();
}

/* 进入 4G 设置页：停止定时器、复位为说明态并同步开关 */
static void admin_4g_ui_enter(void)
{
    admin_4g_timer_stop();
    admin_4g_apply_switch_layout();
    admin_4g_sync_switch_ui();
    admin_4g_set_phase(ADMIN_4G_PHASE_PROMPT);
}


/* 离开 4G 设置页：回 menu1 */
static void admin_4g_back_to_menu1(void)
{
    admin_4g_timer_stop();
    if(g_group_admin != NULL) {
        if(g_group_admin) lv_group_set_editing(g_group_admin, false);
    }
    admin_panel_show(MENU1);
}

/* 2s 定时器：配网中 → 成功/失败态 */
static void cb_admin_4g_timer(lv_timer_t * t)
{
    (void)t;
    g_admin_4g_timer = NULL;
    if(g_admin_view == SETTINGS_4G) {
        if(g_ui_4g_connect_result == 1) {                   //判断是否配网成功
            admin_4g_set_phase(ADMIN_4G_PHASE_SUCCESS);
        } else {
            admin_4g_set_phase(ADMIN_4G_PHASE_FAILURE);
        }
    }
}


/* 4G 开关 OFF→ON：进入配网中 UI，2s 后显示成功 */
static void admin_4g_start_provisioning(void)
{
    admin_4g_timer_stop();
    g_ui_4g_connect_result = 1;//PC默认值，2秒后读取硬件反馈的值
    admin_4g_set_phase(ADMIN_4G_PHASE_PROVISIONING);
    g_admin_4g_timer = lv_timer_create(cb_admin_4g_timer, 2000, NULL);
    lv_timer_set_repeat_count(g_admin_4g_timer, 1);
}


/* 4G 开关切换：更新状态；OFF→ON 触发配网，OFF 时中止流程 */
static void cb_admin_4g_switch_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(g_admin_view != SETTINGS_4G) return;
    if(g_admin_4g_ui_loading) return;

    lv_obj_t * sw = lv_event_get_target_obj(e);
    bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);

    if(on) {
        if(g_admin_4g_phase == ADMIN_4G_PHASE_PROMPT) {
            ui_4g_set(true);
            admin_4g_start_provisioning();
        }
    }
    else {
        ui_4g_set(false);
        if(g_admin_4g_phase != ADMIN_4G_PHASE_PROMPT) {
            admin_4g_timer_stop();
            admin_4g_set_phase(ADMIN_4G_PHASE_PROMPT);
        }
    }
}


/* menu1「网络设置」入口 */
static void cb_admin_open_network_settings(lv_event_t * e)
{
    (void)e;
    admin_panel_show(NETWORK_SETTINGS);
}

/* 网络设置「WIFI设置」入口 */
static void cb_admin_open_wifi_settings(lv_event_t * e)
{
    (void)e;
    admin_panel_show(WIFI_SETTINGS);
}

/* 网络设置「4G设置」入口 */
static void cb_admin_open_4g_settings(lv_event_t * e)
{
    (void)e;
    admin_panel_show(SETTINGS_4G);
}

/* 离开网络设置页：回 menu1 */
static void admin_network_back_to_menu1(void)
{
    if(g_group_admin != NULL) {
        if(g_group_admin) lv_group_set_editing(g_group_admin, false);
    }
    admin_panel_show(MENU1);
}

/* 离开 WIFI 设置页：回网络设置 */
static void admin_wifi_back_to_network(void)
{
    admin_wifi_timer_stop();
    admin_panel_show(NETWORK_SETTINGS);
}


/* 支付设置页：复选框统一样式（未选空心橙框，已选橙色对号） */
static void admin_payment_style_checkbox(lv_obj_t * cb)
{
    if(cb == NULL) return;
    ui_set_obj_font(cb, s_font_sc_30);
    lv_obj_set_style_text_color(cb, lv_color_hex(COL_TEXT), LV_PART_MAIN);

    /* 未选中：透明底 + 橙色边框 */
    lv_obj_set_style_border_color(cb, lv_color_hex(COL_ORANGE), LV_PART_INDICATOR);
    lv_obj_set_style_border_width(cb, 2, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(cb, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_radius(cb, 4, LV_PART_INDICATOR);

    /* 已选中：透明底 + 橙色边框 + 橙色对号 */
    lv_obj_set_style_border_color(cb, lv_color_hex(COL_ORANGE), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(cb, 2, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(cb, LV_OPA_TRANSP, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_image_src(cb, LV_SYMBOL_OK, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(cb, lv_color_hex(COL_ORANGE), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_text_font(cb, &lv_font_montserrat_30, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_pad_all(cb, 2, LV_PART_INDICATOR | LV_STATE_CHECKED);
}

/* 支付设置页：竖向分隔线 */
static void admin_payment_add_divider(lv_obj_t * parent, lv_coord_t x)
{
    lv_obj_t * div = lv_obj_create(parent);
    lv_obj_set_size(div, 1, 280);
    lv_obj_set_pos(div, x, 70);
    lv_obj_set_style_bg_color(div, lv_color_hex(COL_DIM), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(div, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(div, 0, LV_PART_MAIN);
    lv_obj_remove_flag(div, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
}

/* 已保存的支付超时秒数 → roller 选项下标 */


static void admin_payment_sync_method_ui(void)
{
    g_admin_payment_ui_loading = true;
    if(g_admin_lbl_payment_wechat != NULL)
        lv_label_set_text(g_admin_lbl_payment_wechat, ui_translation(STR_PAYMENT_WECHAT));
    if(g_admin_lbl_payment_alipay != NULL)
        lv_label_set_text(g_admin_lbl_payment_alipay, ui_translation(STR_PAYMENT_ALIPAY));
    if(g_admin_sw_payment_wechat != NULL) {
        if(g_ui_payment_wechat_enabled)
            lv_obj_add_state(g_admin_sw_payment_wechat, LV_STATE_CHECKED);
        else
            lv_obj_remove_state(g_admin_sw_payment_wechat, LV_STATE_CHECKED);
    }
    if(g_admin_sw_payment_alipay != NULL) {
        if(g_ui_payment_alipay_enabled)
            lv_obj_add_state(g_admin_sw_payment_alipay, LV_STATE_CHECKED);
        else
            lv_obj_remove_state(g_admin_sw_payment_alipay, LV_STATE_CHECKED);
    }
    g_admin_payment_ui_loading = false;
}


static void cb_admin_payment_alipay_changed(lv_event_t * e)
{
    if(g_admin_payment_ui_loading) return;
    lv_obj_t * sw = lv_event_get_target_obj(e);
    ui_payment_alipay_set(lv_obj_has_state(sw, LV_STATE_CHECKED));
    pay_sync_pay_ui();
}


static void cb_admin_payment_wechat_changed(lv_event_t * e)
{
    if(g_admin_payment_ui_loading) return;
    lv_obj_t * sw = lv_event_get_target_obj(e);
    ui_payment_wechat_set(lv_obj_has_state(sw, LV_STATE_CHECKED));
    pay_sync_pay_ui();
}


/* 支付设置页：按 g_ui_payment_timeout_sec 刷新支付超时 roller */


static const ui_str_id_t g_data_upload_item_str_ids[7] = {
    STR_DATA_BASIC, STR_DATA_SENSOR, STR_DATA_FAULT, STR_DATA_AUTO_DISPENSE,
    STR_DATA_PAYMENT_ORDER, STR_DATA_USER_OP, STR_DATA_DEVICE
};

static const ui_str_id_t g_data_strategy_str_ids[5] = {
    STR_DATA_REALTIME, STR_DATA_SCHEDULED, STR_DATA_WIFI_ONLY,
    STR_DATA_4G_ONLY, STR_DATA_FORBIDDEN
};

/* 数据设置页：复选框统一样式（含禁用态灰字） */
static void admin_data_style_checkbox(lv_obj_t * cb)
{
    admin_payment_style_checkbox(cb);
    lv_obj_set_style_text_color(cb, lv_color_hex(COL_DIM), LV_PART_MAIN | LV_STATE_DISABLED);
}

/* 数据设置页：按 g_ui_data_upload_* 刷新左栏上传项复选框与文案 */
static void admin_data_sync_upload_items_ui(void)
{
    static bool * const vars[7] = {
        &g_ui_data_upload_basic, &g_ui_data_upload_sensor, &g_ui_data_upload_fault,
        &g_ui_data_upload_auto_dispense, &g_ui_data_upload_payment_order,
        &g_ui_data_upload_user_op, &g_ui_data_upload_device
    };
    unsigned i;

    g_admin_data_upload_ui_loading = true;
    for(i = 0; i < 7; i++) {
        if(g_admin_lbl_data_upload[i] != NULL) {
            lv_label_set_text(g_admin_lbl_data_upload[i], ui_translation(g_data_upload_item_str_ids[i]));
        }
        if(g_admin_cb_data_upload[i] != NULL) {
            if(*vars[i]) {
                lv_obj_add_state(g_admin_cb_data_upload[i], LV_STATE_CHECKED);
            }
            else {
                lv_obj_remove_state(g_admin_cb_data_upload[i], LV_STATE_CHECKED);
            }
        }
    }
    g_admin_data_upload_ui_loading = false;
}


/* 数据设置页：切换策略前临时解除右栏全部 DISABLED，便于点选其它项 */
static void admin_data_strategy_enable_all(void)
{
    unsigned i;

    for(i = 0; i < 5; i++) {
        if(g_admin_cb_data_strategy[i] != NULL) {
            lv_obj_remove_state(g_admin_cb_data_strategy[i], LV_STATE_DISABLED);
        }
    }
}

/* 数据设置页：按当前上传项已保存的策略刷新策略页互斥开关 */
static void admin_data_sync_strategy_ui(void)
{
    unsigned i;
    ui_data_upload_strategy_t cur = ui_data_upload_strategy_get();

    g_admin_data_strategy_ui_loading = true;
    for(i = 0; i < 5; i++) {
        if(g_admin_cb_data_strategy[i] == NULL) continue;
        if(g_admin_lbl_data_strategy[i] != NULL) {
            lv_label_set_text(g_admin_lbl_data_strategy[i], ui_translation(g_data_strategy_str_ids[i]));
        }
        if((ui_data_upload_strategy_t)i == cur) {
            lv_obj_add_state(g_admin_cb_data_strategy[i], LV_STATE_CHECKED);
        }
        else {
            lv_obj_remove_state(g_admin_cb_data_strategy[i], LV_STATE_CHECKED);
        }
    }
    g_admin_data_strategy_ui_loading = false;
}


/* 由策略复选框对象反查下标；未找到返回 -1 */

static int admin_data_upload_index_from_cb(lv_obj_t * cb)
{
    unsigned i;
    for(i = 0; i < 7; i++) {
        if(g_admin_cb_data_upload[i] == cb) return (int)i;
    }
    return -1;
}

static int admin_data_strategy_index_from_cb(lv_obj_t * cb)
{
    unsigned i;

    for(i = 0; i < 5; i++) {
        if(g_admin_cb_data_strategy[i] == cb) return (int)i;
    }
    return -1;
}

/* 数据设置页：策略项按下时先解除禁用，便于切换到其它策略 */


static void cb_admin_data_strategy_changed(lv_event_t * e)
{
    lv_obj_t * cb;
    int idx;

    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(g_admin_data_strategy_ui_loading) return;
    if(g_admin_view != DATA_SETTINGS) return;

    cb = lv_event_get_target_obj(e);
    idx = admin_data_strategy_index_from_cb(cb);
    if(idx < 0) return;

    if(lv_obj_has_state(cb, LV_STATE_CHECKED)) {
        ui_data_upload_strategy_set((ui_data_upload_strategy_t)idx);
        for(unsigned i = 0; i < 5; i++) {
            if(g_admin_cb_data_strategy[i] != NULL && i != (unsigned)idx) {
                lv_obj_remove_state(g_admin_cb_data_strategy[i], LV_STATE_CHECKED);
            }
        }
    }
    else if((ui_data_upload_strategy_t)idx == ui_data_upload_strategy_get()) {
        g_admin_data_strategy_ui_loading = true;
        lv_obj_add_state(cb, LV_STATE_CHECKED);
        g_admin_data_strategy_ui_loading = false;
    }
}




/*
 * 支付超时 roller：外设编码器短按发 LV_EVENT_CLICKED，在编辑/导航间切换。
 * 短按进入编辑（旋转改值），再按退回选择（旋转切焦点）；失焦时自动退出编辑。
 */


/* 离开支付设置页：回管理员 menu2 */
static void admin_payment_back_to_menu2(void)
{
    admin_panel_show(MENU2);
}

/* 管理员 menu2「支付设置」入口 */
static void cb_admin_open_payment_settings(lv_event_t * e)
{
    (void)e;
    admin_panel_show(PAYMENT_SETTINGS);
}

static void admin_data_back_to_menu1(void)
{
    if(g_admin_data_page == ADMIN_DATA_PAGE_STRATEGY) {
        admin_data_set_page(ADMIN_DATA_PAGE_LIST);
        admin_data_sync_upload_items_ui();
        return;
    }
    admin_panel_show(MENU1);
}


/* 管理员 menu2「厂商维护」入口 */
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

/* 管理员 menu1「数据设置」入口 */
static void cb_admin_open_data_settings(lv_event_t * e)
{
    (void)e;
    admin_panel_show(DATA_SETTINGS);
}

/* 管理员菜单按钮内文字 label（button → inner → label） */
static lv_obj_t * admin_menu_btn_get_label(lv_obj_t * btn)
{
    if(btn == NULL) return NULL;
    uint32_t n = lv_obj_get_child_cnt(btn);
    if(n == 0) return NULL;
    return lv_obj_get_child(btn, (int)(n - 1));
}

static void cb_admin_err_blocker(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    lv_obj_t * blk = lv_event_get_current_target(e);
    if(g_admin_lbl_msg_pwd != NULL) lv_obj_add_flag(g_admin_lbl_msg_pwd, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_ta_pwd != NULL) lv_textarea_set_text(g_admin_ta_pwd, "");
    if(g_admin_lbl_serial_title != NULL) lv_obj_remove_flag(g_admin_lbl_serial_title, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_input_wrap != NULL) lv_obj_remove_flag(g_admin_input_wrap, LV_OBJ_FLAG_HIDDEN);
    if(blk != NULL) lv_obj_del(blk);
}

static void cb_admin_kb_btn(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_kb_ta == NULL) return;
    const char * txt = (const char *)lv_event_get_user_data(e);
    if(txt == NULL) return;
    if(lv_strcmp(txt, "DEL") == 0) {
        lv_textarea_delete_char(g_admin_kb_ta);
    } else if(lv_strcmp(txt, "ENT") == 0) {
        lv_obj_send_event(g_admin_kb_ta, LV_EVENT_READY, NULL);
    } else {
        lv_textarea_add_text(g_admin_kb_ta, txt);
    }
}

static void cb_admin_menu_gesture(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_GESTURE) return;
    if(g_admin_view != MENU1 && g_admin_view != MENU2) return;
    lv_indev_t * indev = lv_indev_active();
    if(indev == NULL) return;
    lv_dir_t dir = lv_indev_get_gesture_dir(indev);
    if(g_admin_view == MENU1 && dir == LV_DIR_LEFT) {
        lv_indev_wait_release(indev);
        admin_panel_show(MENU2);
    } else if(g_admin_view == MENU2 && dir == LV_DIR_RIGHT) {
        lv_indev_wait_release(indev);
        admin_panel_show(MENU1);
    }
}

static lv_obj_t * admin_panel_apply_shell(lv_obj_t * panel, lv_obj_t * title_lbl)
{
    if(panel == NULL) return NULL;
    lv_obj_t * img_title = lv_image_create(panel);
    lv_image_set_src(img_title, &title_box);
    lv_obj_align(img_title, LV_ALIGN_TOP_MID, 0, 25);
    lv_obj_t * wrap = lv_obj_create(panel);
    lv_obj_set_size(wrap, 1117, 409);
    lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(wrap, 0, LV_PART_MAIN);
    lv_obj_remove_flag(wrap, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t * img_box = lv_image_create(wrap);
    lv_image_set_src(img_box, &set_box);
    lv_obj_center(img_box);
    lv_obj_move_background(wrap);
    lv_obj_move_background(img_title);
    if(title_lbl != NULL) {
        lv_obj_align(title_lbl, LV_ALIGN_TOP_MID, 0, 25);
        lv_obj_move_foreground(title_lbl);
    }
    return wrap;
}

/* 管理员菜单按钮文字绑定 i18n，随 ui_lang_apply_all 刷新 */
static void admin_menu_btn_bind_i18n(lv_obj_t * btn, ui_str_id_t id)
{
    lv_obj_t * lbl = admin_menu_btn_get_label(btn);
    ui_lang_bind_label(lbl, id);
}

/* 创建管理员菜单渐变风格按钮（双层：外环渐变描边 + 内层渐变底） */
static void admin_group_edge_cb(lv_group_t * group, bool forward)
{
    (void)group;
    if(!forward || g_admin_view != MENU1 || g_group_admin == NULL) return;
    lv_obj_t * focused = (g_group_admin ? lv_group_get_focused(g_group_admin) : NULL);
    if(focused == g_admin_menu1_btns[7]) {
        admin_panel_show(MENU2);
    }
}

/* menu2 第 1 钮左转落到电源时：回到 menu1 第 8 钮 */
static void admin_group_focus_cb(lv_group_t * group)
{
    (void)group;
    if(g_group_admin == NULL) return;
    lv_obj_t * focused = (g_group_admin ? lv_group_get_focused(g_group_admin) : NULL);
    if(g_admin_view == MENU2 && focused == g_admin_btn_power &&
       s_admin_group_prev_focus == g_admin_menu2_btns[0]) {
        admin_panel_show(MENU1);
        if(g_admin_menu1_btns[7] != NULL) {
            lv_group_focus_obj(g_admin_menu1_btns[7]);
        }
        s_admin_group_prev_focus = g_admin_menu1_btns[7];
        return;
    }
    s_admin_group_prev_focus = focused;
}

static void admin_pwd_chg_enter(void)
{
    g_admin_pwd_chg_pending[0] = '\0';
    g_admin_pwd_chg_page1_step = 0;
    admin_panel_show(PASSWORD_CHANGE_OLD);
}


/* 密码修改步骤一：键盘 OK 校验原密码，成功进入双新密码页 */


/* 密码修改步骤二：切换到双新密码输入页 */


/* 密码修改步骤二：第一框 OK 进第二框；第二框 OK 比对并保存 */


static void admin_pwd_chg_back_to_menu2(void)
{
    if(g_admin_ta_pwd_chg_old != NULL) lv_textarea_set_text(g_admin_ta_pwd_chg_old, "");
    if(g_admin_ta_pwd_chg_new1 != NULL) lv_textarea_set_text(g_admin_ta_pwd_chg_new1, "");
    if(g_admin_ta_pwd_chg_new2 != NULL) lv_textarea_set_text(g_admin_ta_pwd_chg_new2, "");
    g_admin_pwd_chg_pending[0] = '\0';
    g_admin_pwd_chg_page1_step = 0;
    if(g_admin_pwd_chg_result != NULL) {
        lv_obj_add_flag(g_admin_pwd_chg_result, LV_OBJ_FLAG_HIDDEN);
    }
    admin_panel_show(MENU2);
}


/* menu2「密码修改」入口 */
static void cb_admin_open_password_change(lv_event_t * e)
{
    (void)e;
    admin_pwd_chg_enter();
}

//构建管理员页，顶栏返回/启停/电源 + 状态栏；密码 888888，进入管理员设置页面
//密码子面板：密码标题/密码输入框/密码错误提示
//管理员设置页1（8 宫格）：机器ID设置/网络设置/数据设置/程序设置/防缠绕/新风护理/待机时间/厂商维护
//管理员设置页2（8 宫格）：屏幕亮度/声音控制/语言设置/系统升级/恢复默认/联系我们/支付设置/密码修改
static void build_admin(void)
{
    lv_obj_t * root = lv_obj_create(g_scr_admin);
    lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(root, 0, 0);
    lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(root, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(root, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(root, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(root, LV_LAYOUT_NONE, LV_PART_MAIN);

    lv_obj_t * top = create_top_bar(root, &g_lbl_clock_admin, NULL, NULL);

    g_admin_btn_back = make_top_back_btn(top, cb_admin_back);

    g_admin_btn_runpause = add_top_text_btn(top, "启停", 100);
    g_admin_btn_power = add_top_text_btn(top, "电源", 180);
    lv_obj_add_event_cb(g_admin_btn_power, cb_power_long, LV_EVENT_LONG_PRESSED, NULL);

    const lv_coord_t body_y = 60;
    const lv_coord_t body_h = 540;

    /* 密码子面板 */
    g_admin_panel_pwd = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_pwd, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_pwd, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_pwd, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_pwd, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_pwd, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_pwd, LV_LAYOUT_NONE, LV_PART_MAIN);

    lv_obj_t * lbl_admin_title = lv_label_create(g_admin_panel_pwd);
    ui_lang_bind_label(lbl_admin_title, STR_ADMIN_MENU_TITLE);
    lv_obj_set_style_text_color(lbl_admin_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_admin_title, s_font_sc_30);
    lv_obj_align(lbl_admin_title, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t * img_title_bar = lv_image_create(g_admin_panel_pwd);
    lv_image_set_src(img_title_bar, &title_bar);
    lv_obj_align(img_title_bar, LV_ALIGN_TOP_MID, 0, 50);

    lv_obj_t * lbl_serial_title = lv_label_create(g_admin_panel_pwd);
    g_admin_lbl_serial_title = lbl_serial_title;
    ui_lang_bind_label(lbl_serial_title, STR_VENDOR_SERIAL_HINT);
    lv_obj_set_style_text_color(lbl_serial_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_serial_title, s_font_sc_40);
    lv_obj_align(lbl_serial_title, LV_ALIGN_TOP_MID, 0, 170);

    {
        lv_obj_t * input_wrap = lv_obj_create(g_admin_panel_pwd);
        g_admin_input_wrap = input_wrap;
        lv_obj_set_size(input_wrap, 433, 149);
        lv_obj_align(input_wrap, LV_ALIGN_TOP_MID, 0, 240);
        lv_obj_set_style_bg_opa(input_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(input_wrap, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(input_wrap, 0, LV_PART_MAIN);

        lv_obj_t * img_input_bg = lv_image_create(input_wrap);
        lv_image_set_src(img_input_bg, &input_box);
        lv_obj_center(img_input_bg);

        g_admin_ta_pwd = lv_textarea_create(input_wrap);
        lv_obj_set_size(g_admin_ta_pwd, 350, 60);
        lv_obj_align(g_admin_ta_pwd, LV_ALIGN_CENTER, 0, 0);
        lv_textarea_set_one_line(g_admin_ta_pwd, true);
        lv_textarea_set_max_length(g_admin_ta_pwd, 6);
        lv_textarea_set_accepted_chars(g_admin_ta_pwd, "0123456789");
        lv_obj_set_style_text_font(g_admin_ta_pwd, s_font_sc_40, LV_PART_MAIN);
        lv_obj_set_style_text_color(g_admin_ta_pwd, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(g_admin_ta_pwd, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(g_admin_ta_pwd, 0, LV_PART_MAIN);
        lv_obj_set_style_text_align(g_admin_ta_pwd, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_style_text_letter_space(g_admin_ta_pwd, 8, LV_PART_MAIN);
        lv_obj_add_event_cb(g_admin_ta_pwd, cb_admin_ta_ready, LV_EVENT_READY, NULL);
        lv_obj_add_event_cb(g_admin_ta_pwd, cb_admin_ta_kb_focus, LV_EVENT_ALL, NULL);
    }

    g_admin_lbl_msg_pwd = lv_label_create(g_admin_panel_pwd);
    lv_obj_set_width(g_admin_lbl_msg_pwd, LV_PCT(80));
    lv_obj_set_style_text_color(g_admin_lbl_msg_pwd, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_align(g_admin_lbl_msg_pwd, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_msg_pwd, s_font_sc_50);
    lv_obj_align(g_admin_lbl_msg_pwd, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(g_admin_lbl_msg_pwd, LV_OBJ_FLAG_HIDDEN);

    /* 管理员设置页1（8 宫格） */
    g_admin_panel_menu1 = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_menu1, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_menu1, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_menu1, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_menu1, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_menu1, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_menu1, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_panel_menu1, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_admin_panel_menu1, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(g_admin_panel_menu1, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(g_admin_panel_menu1, cb_admin_menu_gesture, LV_EVENT_GESTURE, NULL);
    lv_obj_add_flag(g_admin_panel_menu1, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_menu1_title = lv_label_create(g_admin_panel_menu1);
    lv_obj_set_style_text_color(g_admin_lbl_menu1_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_menu1_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_menu1_title, LV_ALIGN_TOP_MID, 0, 10);
    ui_lang_bind_label(g_admin_lbl_menu1_title, STR_ADMIN_MENU_TITLE);
    lv_obj_add_flag(g_admin_lbl_menu1_title, LV_OBJ_FLAG_GESTURE_BUBBLE);

    lv_obj_t * img_menu1_bar = lv_image_create(g_admin_panel_menu1);
    lv_image_set_src(img_menu1_bar, &title_bar);
    lv_obj_align(img_menu1_bar, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_add_flag(img_menu1_bar, LV_OBJ_FLAG_GESTURE_BUBBLE);

    static const ui_str_id_t menu1_ids[8] = {
        STR_ADMIN_M1_MACHINE_ID, STR_ADMIN_M2_NETWORK, STR_ADMIN_M2_DATA, STR_ADMIN_M1_PROGRAM,
        STR_ADMIN_M2_AUTO_DISPENSE, STR_ADMIN_M2_FRESH_AIR, STR_ADMIN_M1_STANDBY, STR_ADMIN_M2_VENDOR_MAINT
    };
    static const lv_image_dsc_t * const menu1_icons[8] = {
        &id_set, &net_set, &data_set, &program_set,
        &auto_put_set, &fresh_air_set, &standby_time_set, NULL
    };
    const lv_coord_t btn_w = ADMIN_MENU_BTN_W;
    const lv_coord_t btn_h = ADMIN_MENU_BTN_H;
    const lv_coord_t gap_x = ADMIN_MENU_BTN_GAP_X;
    const lv_coord_t gap_y = ADMIN_MENU_BTN_GAP_Y;
    const lv_coord_t grid_w = btn_w * 4 + gap_x * 3;
    const lv_coord_t grid_x0 = (lv_coord_t)((UI_FIXED_W - grid_w) / 2);
    const lv_coord_t grid_y0 = ADMIN_MENU_BTN_GRID_Y0;

    for(int i = 0; i < 8; i++) {
        int row = i / 4;
        int col = i % 4;
        g_admin_menu1_btns[i] = make_admin_menu_btn(g_admin_panel_menu1, ui_translation(menu1_ids[i]), menu1_icons[i]);
        lv_obj_set_size(g_admin_menu1_btns[i], btn_w, btn_h);
        lv_obj_set_pos(g_admin_menu1_btns[i],
            grid_x0 + col * (btn_w + gap_x),
            grid_y0 + row * (btn_h + gap_y));
        admin_menu_btn_bind_i18n(g_admin_menu1_btns[i], menu1_ids[i]);
        if(i == 0) lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_machine_id, LV_EVENT_CLICKED, NULL);
        else if(i == 1) lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_network_settings, LV_EVENT_CLICKED, NULL);
        else if(i == 2) lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_data_settings, LV_EVENT_CLICKED, NULL);
        else if(i == 3) lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_program_settings, LV_EVENT_CLICKED, NULL);
        else if(i == 4) lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_auto_dispense, LV_EVENT_CLICKED, NULL);
        else if(i == 5) lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_fresh_air_care, LV_EVENT_CLICKED, NULL);
        else if(i == 6) lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_dormancy_standby, LV_EVENT_CLICKED, NULL);
        else if(i == 7) lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_vendor_maint, LV_EVENT_CLICKED, NULL);
        lv_obj_add_flag(g_admin_menu1_btns[i], LV_OBJ_FLAG_GESTURE_BUBBLE);
    }

    /* 管理员设置页2（8 宫格） */
    g_admin_panel_menu2 = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_menu2, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_menu2, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_menu2, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_menu2, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_menu2, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_menu2, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_panel_menu2, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_admin_panel_menu2, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(g_admin_panel_menu2, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(g_admin_panel_menu2, cb_admin_menu_gesture, LV_EVENT_GESTURE, NULL);
    lv_obj_add_flag(g_admin_panel_menu2, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_menu2_title = lv_label_create(g_admin_panel_menu2);
    lv_obj_set_style_text_color(g_admin_lbl_menu2_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_menu2_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_menu2_title, LV_ALIGN_TOP_MID, 0, 10);
    ui_lang_bind_label(g_admin_lbl_menu2_title, STR_ADMIN_MENU_TITLE);
    lv_obj_add_flag(g_admin_lbl_menu2_title, LV_OBJ_FLAG_GESTURE_BUBBLE);

    lv_obj_t * img_menu2_bar = lv_image_create(g_admin_panel_menu2);
    lv_image_set_src(img_menu2_bar, &title_bar);
    lv_obj_align(img_menu2_bar, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_add_flag(img_menu2_bar, LV_OBJ_FLAG_GESTURE_BUBBLE);

    static const ui_str_id_t menu2_ids[8] = {
        STR_ADMIN_M1_BRIGHTNESS, STR_ADMIN_M1_SOUND, STR_ADMIN_M1_LANGUAGE, STR_ADMIN_M2_UPGRADE,
        STR_ADMIN_M1_FACTORY_RESET, STR_ADMIN_M1_CONTACT, STR_ADMIN_M2_PAYMENT, STR_ADMIN_M2_PASSWORD
    };
    static const lv_image_dsc_t * const menu2_icons[8] = {
        &bright_set, &sound_set, &language_set, &upgrade_set,
        &default_set, &service_set, &pay_set, &password_set
    };

    for(int i = 0; i < 8; i++) {
        int row = i / 4;
        int col = i % 4;
        g_admin_menu2_btns[i] = make_admin_menu_btn(g_admin_panel_menu2, ui_translation(menu2_ids[i]), menu2_icons[i]);
        lv_obj_set_size(g_admin_menu2_btns[i], btn_w, btn_h);
        lv_obj_set_pos(g_admin_menu2_btns[i],
            grid_x0 + col * (btn_w + gap_x),
            grid_y0 + row * (btn_h + gap_y));
        admin_menu_btn_bind_i18n(g_admin_menu2_btns[i], menu2_ids[i]);
        if(i == 0) lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_brightness, LV_EVENT_CLICKED, NULL);
        else if(i == 1) lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_sound, LV_EVENT_CLICKED, NULL);
        else if(i == 2) lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_language_settings, LV_EVENT_CLICKED, NULL);
        else if(i == 3) lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_system_upgrade, LV_EVENT_CLICKED, NULL);
        else if(i == 4) lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_factory_reset, LV_EVENT_CLICKED, NULL);
        else if(i == 5) lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_contact_us, LV_EVENT_CLICKED, NULL);
        else if(i == 6) lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_payment_settings, LV_EVENT_CLICKED, NULL);
        else if(i == 7) lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_password_change, LV_EVENT_CLICKED, NULL);
        lv_obj_add_flag(g_admin_menu2_btns[i], LV_OBJ_FLAG_GESTURE_BUBBLE);
    }

    /* 厂商维护：出厂序列号 */
    g_admin_panel_vendor_serial = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_vendor_serial, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_vendor_serial, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_vendor_serial, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_vendor_serial, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_vendor_serial, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_vendor_serial, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_vendor_serial, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_vendor_serial_title = lv_label_create(g_admin_panel_vendor_serial);
    ui_lang_bind_label(g_admin_lbl_vendor_serial_title, STR_ADMIN_M2_VENDOR_MAINT);
    lv_obj_set_style_text_color(g_admin_lbl_vendor_serial_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_vendor_serial_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_vendor_serial_title, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t * img_vendor_bar = lv_image_create(g_admin_panel_vendor_serial);
    lv_image_set_src(img_vendor_bar, &title_bar);
    lv_obj_align(img_vendor_bar, LV_ALIGN_TOP_MID, 0, 50);

    lv_obj_t * lbl_vendor_hint = lv_label_create(g_admin_panel_vendor_serial);
    ui_lang_bind_label(lbl_vendor_hint, STR_VENDOR_SERIAL_HINT);
    lv_obj_set_style_text_color(lbl_vendor_hint, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_vendor_hint, s_font_sc_40);
    lv_obj_align(lbl_vendor_hint, LV_ALIGN_TOP_MID, 0, 170);

    {
        lv_obj_t * vwrap = lv_obj_create(g_admin_panel_vendor_serial);
        lv_obj_set_size(vwrap, 433, 149);
        lv_obj_align(vwrap, LV_ALIGN_TOP_MID, 0, 240);
        lv_obj_set_style_bg_opa(vwrap, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(vwrap, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(vwrap, 0, LV_PART_MAIN);
        lv_obj_t * vbg = lv_image_create(vwrap);
        lv_image_set_src(vbg, &input_box);
        lv_obj_center(vbg);
        g_admin_ta_vendor_serial = lv_textarea_create(vwrap);
        lv_obj_set_size(g_admin_ta_vendor_serial, 350, 60);
        lv_obj_align(g_admin_ta_vendor_serial, LV_ALIGN_CENTER, 0, 0);
        lv_textarea_set_one_line(g_admin_ta_vendor_serial, true);
        lv_textarea_set_max_length(g_admin_ta_vendor_serial, 6);
        lv_textarea_set_accepted_chars(g_admin_ta_vendor_serial, "0123456789");
        lv_obj_set_style_text_font(g_admin_ta_vendor_serial, s_font_sc_40, LV_PART_MAIN);
        lv_obj_set_style_text_color(g_admin_ta_vendor_serial, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(g_admin_ta_vendor_serial, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(g_admin_ta_vendor_serial, 0, LV_PART_MAIN);
        lv_obj_set_style_text_align(g_admin_ta_vendor_serial, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_style_text_letter_space(g_admin_ta_vendor_serial, 8, LV_PART_MAIN);
        lv_obj_add_event_cb(g_admin_ta_vendor_serial, cb_admin_ta_ready, LV_EVENT_READY, NULL);
        lv_obj_add_event_cb(g_admin_ta_vendor_serial, cb_admin_ta_kb_focus, LV_EVENT_ALL, NULL);
    }

    /* 厂商维护：功能选择 */
    g_admin_panel_vendor_menu = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_vendor_menu, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_vendor_menu, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_vendor_menu, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_vendor_menu, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_vendor_menu, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_vendor_menu, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_vendor_menu, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t * lbl_vendor_menu_title = lv_label_create(g_admin_panel_vendor_menu);
    ui_lang_bind_label(lbl_vendor_menu_title, STR_ADMIN_M2_VENDOR_MAINT);
    lv_obj_set_style_text_color(lbl_vendor_menu_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_vendor_menu_title, s_font_sc_30);
    lv_obj_align(lbl_vendor_menu_title, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t * img_vendor_menu_bar = lv_image_create(g_admin_panel_vendor_menu);
    lv_image_set_src(img_vendor_menu_bar, &title_bar);
    lv_obj_align(img_vendor_menu_bar, LV_ALIGN_TOP_MID, 0, 50);

    g_admin_btn_vendor_self_check = make_admin_menu_btn(g_admin_panel_vendor_menu, ui_translation(STR_VENDOR_SELF_CHECK), NULL);
    lv_obj_set_size(g_admin_btn_vendor_self_check, btn_w, btn_h);
    lv_obj_align(g_admin_btn_vendor_self_check, LV_ALIGN_CENTER, 0, -30);
    admin_menu_btn_bind_i18n(g_admin_btn_vendor_self_check, STR_VENDOR_SELF_CHECK);
    lv_obj_add_event_cb(g_admin_btn_vendor_self_check, cb_admin_open_selfcheck, LV_EVENT_CLICKED, NULL);

    g_admin_btn_vendor_self_learn = make_admin_menu_btn(g_admin_panel_vendor_menu, ui_translation(STR_VENDOR_SELF_LEARN), NULL);
    lv_obj_set_size(g_admin_btn_vendor_self_learn, btn_w, btn_h);
    lv_obj_align(g_admin_btn_vendor_self_learn, LV_ALIGN_CENTER, 0, 110);
    admin_menu_btn_bind_i18n(g_admin_btn_vendor_self_learn, STR_VENDOR_SELF_LEARN);
    lv_obj_add_event_cb(g_admin_btn_vendor_self_learn, cb_admin_open_cycle, LV_EVENT_CLICKED, NULL);

    /* 待机时间子面板（标题页：title_box + set_box + 两项互斥开关） */
    g_admin_panel_dormancy = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_dormancy, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_dormancy, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_dormancy, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_dormancy, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_dormancy, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_dormancy, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_dormancy, LV_OBJ_FLAG_HIDDEN);

    g_admin_img_dormancy_title_box = lv_image_create(g_admin_panel_dormancy);
    lv_image_set_src(g_admin_img_dormancy_title_box, &title_box);
    lv_obj_align(g_admin_img_dormancy_title_box, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_lbl_dormancy_title = lv_label_create(g_admin_panel_dormancy);
    ui_lang_bind_label(g_admin_lbl_dormancy_title, STR_DORMANCY_TITLE);
    lv_obj_set_style_text_color(g_admin_lbl_dormancy_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_dormancy_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_dormancy_title, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_dormancy_set_box_wrap = lv_obj_create(g_admin_panel_dormancy);
    lv_obj_set_size(g_admin_dormancy_set_box_wrap, 1117, 409);
    lv_obj_align(g_admin_dormancy_set_box_wrap, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(g_admin_dormancy_set_box_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_dormancy_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_dormancy_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_dormancy_set_box_wrap, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * img_dormancy_set_box = lv_image_create(g_admin_dormancy_set_box_wrap);
    lv_image_set_src(img_dormancy_set_box, &set_box);
    lv_obj_center(img_dormancy_set_box);

    /* ===== 待机时间页尺寸/间距集中配置（改这里即可统一调整） ===== */
    const lv_coord_t dorm_row_w   = 800;   /* 行宽（横线、说明文字宽度同此） */
    const lv_coord_t dorm_row1_y  = 85;    /* 行1 顶部 Y（相对 set_box 顶） */
    const lv_coord_t dorm_sep_y   = 200;   /* 横线 Y（行1 与行2 之间） */
    const lv_coord_t dorm_row2_y  = 240;   /* 行2 顶部 Y */
    const lv_coord_t dorm_row_pad = 20;     /* 主文字与小文字的间距 */

    /* 行1：时间设置（主文字+开关同排垂直居中，说明小字在下） */
    g_admin_dormancy_row_time = lv_obj_create(g_admin_dormancy_set_box_wrap);
    lv_obj_set_size(g_admin_dormancy_row_time, dorm_row_w, 150);
    lv_obj_align(g_admin_dormancy_row_time, LV_ALIGN_TOP_MID, 0, dorm_row1_y);
    lv_obj_set_style_bg_opa(g_admin_dormancy_row_time, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_dormancy_row_time, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_dormancy_row_time, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_dormancy_row_time, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(g_admin_dormancy_row_time, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(g_admin_dormancy_row_time, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(g_admin_dormancy_row_time, dorm_row_pad, LV_PART_MAIN);
    lv_obj_clear_flag(g_admin_dormancy_row_time, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    /* 主文字 + 开关 同一行（文字左、开关右、垂直居中） */
    lv_obj_t * dormancy_time_line = lv_obj_create(g_admin_dormancy_row_time);
    lv_obj_set_size(dormancy_time_line, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(dormancy_time_line, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(dormancy_time_line, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(dormancy_time_line, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(dormancy_time_line, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(dormancy_time_line, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(dormancy_time_line, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    g_admin_lbl_dormancy_time = lv_label_create(dormancy_time_line);
    ui_lang_bind_label(g_admin_lbl_dormancy_time, STR_DORM_TIME_SET);
    lv_obj_set_style_text_color(g_admin_lbl_dormancy_time, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_dormancy_time, s_font_sc_30);

    g_admin_dormancy_sw_time = lv_switch_create(dormancy_time_line);
    admin_data_style_switch(g_admin_dormancy_sw_time);
    lv_obj_add_event_cb(g_admin_dormancy_sw_time, cb_admin_dormancy_sw_changed, LV_EVENT_VALUE_CHANGED, NULL);

    /* 说明小字 */
    g_admin_lbl_dormancy_time_hint = lv_label_create(g_admin_dormancy_row_time);
    ui_lang_bind_label(g_admin_lbl_dormancy_time_hint, STR_DORM_TIME_HINT);
    lv_obj_set_style_text_color(g_admin_lbl_dormancy_time_hint, lv_color_hex(COL_SETTING_HINT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_dormancy_time_hint, s_font_sc_27);
    lv_obj_set_width(g_admin_lbl_dormancy_time_hint, dorm_row_w);
    lv_label_set_long_mode(g_admin_lbl_dormancy_time_hint, LV_LABEL_LONG_DOT);

    /* 两项之间的灰色横线 */
    lv_obj_t * dormancy_sep = lv_obj_create(g_admin_dormancy_set_box_wrap);
    lv_obj_set_size(dormancy_sep, dorm_row_w, 1);
    lv_obj_align(dormancy_sep, LV_ALIGN_TOP_MID, 0, dorm_sep_y);
    lv_obj_set_style_bg_color(dormancy_sep, lv_color_hex(COL_DIM), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(dormancy_sep, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(dormancy_sep, 0, LV_PART_MAIN);
    lv_obj_clear_flag(dormancy_sep, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    /* 行2：不息屏（主文字+开关同排垂直居中，说明小字在下） */
    g_admin_dormancy_row_no_sleep = lv_obj_create(g_admin_dormancy_set_box_wrap);
    lv_obj_set_size(g_admin_dormancy_row_no_sleep, dorm_row_w, 150);
    lv_obj_align(g_admin_dormancy_row_no_sleep, LV_ALIGN_TOP_MID, 0, dorm_row2_y);
    lv_obj_set_style_bg_opa(g_admin_dormancy_row_no_sleep, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_dormancy_row_no_sleep, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_dormancy_row_no_sleep, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_dormancy_row_no_sleep, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(g_admin_dormancy_row_no_sleep, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(g_admin_dormancy_row_no_sleep, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(g_admin_dormancy_row_no_sleep, dorm_row_pad, LV_PART_MAIN);
    lv_obj_clear_flag(g_admin_dormancy_row_no_sleep, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    /* 主文字 + 开关 同一行（文字左、开关右、垂直居中） */
    lv_obj_t * dormancy_nosleep_line = lv_obj_create(g_admin_dormancy_row_no_sleep);
    lv_obj_set_size(dormancy_nosleep_line, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(dormancy_nosleep_line, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(dormancy_nosleep_line, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(dormancy_nosleep_line, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(dormancy_nosleep_line, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(dormancy_nosleep_line, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(dormancy_nosleep_line, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    g_admin_lbl_dormancy_no_sleep = lv_label_create(dormancy_nosleep_line);
    ui_lang_bind_label(g_admin_lbl_dormancy_no_sleep, STR_DORM_NO_SLEEP);
    lv_obj_set_style_text_color(g_admin_lbl_dormancy_no_sleep, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_dormancy_no_sleep, s_font_sc_30);

    g_admin_dormancy_sw_no_sleep = lv_switch_create(dormancy_nosleep_line);
    admin_data_style_switch(g_admin_dormancy_sw_no_sleep);
    lv_obj_add_event_cb(g_admin_dormancy_sw_no_sleep, cb_admin_dormancy_sw_changed, LV_EVENT_VALUE_CHANGED, NULL);

    /* 说明小字 */
    g_admin_lbl_dormancy_no_sleep_hint = lv_label_create(g_admin_dormancy_row_no_sleep);
    ui_lang_bind_label(g_admin_lbl_dormancy_no_sleep_hint, STR_DORM_NO_SLEEP_HINT);
    lv_obj_set_style_text_color(g_admin_lbl_dormancy_no_sleep_hint, lv_color_hex(COL_SETTING_HINT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_dormancy_no_sleep_hint, s_font_sc_27);
    lv_obj_set_width(g_admin_lbl_dormancy_no_sleep_hint, dorm_row_w);
    lv_label_set_long_mode(g_admin_lbl_dormancy_no_sleep_hint, LV_LABEL_LONG_DOT);

    admin_dormancy_sync_switches();

    /* ===== 时间选择子页（无 title_box/set_box，纯文字+数字瓦片+滚动条+按钮） ===== */
    g_admin_dormancy_tp_wrap = lv_obj_create(g_admin_panel_dormancy);
    lv_obj_set_size(g_admin_dormancy_tp_wrap, LV_PCT(100), body_h);
    lv_obj_align(g_admin_dormancy_tp_wrap, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(g_admin_dormancy_tp_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_dormancy_tp_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_dormancy_tp_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_dormancy_tp_wrap, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_dormancy_tp_wrap, LV_OBJ_FLAG_HIDDEN);

    /* ---- 数字瓦片行："机器将在" + 4×time_set_box + ":" + "后熄屏"，同一行水平居中 ---- */
    {
        const lv_coord_t tile_w   = 78;
        const lv_coord_t tile_h   = 101;
        const lv_coord_t gap      = 8;
        const lv_coord_t colon_w  = 24;   /* 冒号占宽 */
        /* 总宽 = 前缀 + gap + 4*tile + 3*gap + colon + gap + 后缀 */
        /* 先算中间组宽：4*78 + 3*8 + 24 = 360 */
        const lv_coord_t group_w  = tile_w * 4 + gap * 3 + colon_w;  /* 360 */
        const lv_coord_t row_y    = 180;  /* 行 Y（相对 wrap 顶） */

        /* "机器将在" */
        g_admin_dormancy_tp_lbl_prefix = lv_label_create(g_admin_dormancy_tp_wrap);
        ui_lang_bind_label(g_admin_dormancy_tp_lbl_prefix, STR_DORM_TIME_PREFIX);
        lv_obj_set_style_text_color(g_admin_dormancy_tp_lbl_prefix, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_dormancy_tp_lbl_prefix, s_font_sc_50);

        /* "后熄屏" */
        g_admin_dormancy_tp_lbl_suffix = lv_label_create(g_admin_dormancy_tp_wrap);
        ui_lang_bind_label(g_admin_dormancy_tp_lbl_suffix, STR_DORM_TIME_SUFFIX);
        lv_obj_set_style_text_color(g_admin_dormancy_tp_lbl_suffix, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_dormancy_tp_lbl_suffix, s_font_sc_50);

        /* 先让标签自适应大小后再计算居中偏移 */
        lv_obj_update_layout(g_admin_dormancy_tp_lbl_prefix);
        lv_obj_update_layout(g_admin_dormancy_tp_lbl_suffix);
        const lv_coord_t prefix_w = lv_obj_get_width(g_admin_dormancy_tp_lbl_prefix);
        const lv_coord_t suffix_w = lv_obj_get_width(g_admin_dormancy_tp_lbl_suffix);
        const lv_coord_t row_total_w = prefix_w + gap + group_w + gap + suffix_w;
        const lv_coord_t row_start_x = (UI_FIXED_W - row_total_w) / 2 - 80;  /* 整体偏移 */

        /* 放置前缀 */
        lv_obj_set_pos(g_admin_dormancy_tp_lbl_prefix, row_start_x - 15,
            row_y + (tile_h - lv_obj_get_height(g_admin_dormancy_tp_lbl_prefix)) / 2);

        /* 放置后缀 */
        lv_obj_set_pos(g_admin_dormancy_tp_lbl_suffix,
            row_start_x + prefix_w + gap + group_w + gap + 15,
            row_y + (tile_h - lv_obj_get_height(g_admin_dormancy_tp_lbl_suffix)) / 2);

        /* 数字瓦片组起始 x */
        const lv_coord_t tiles_x0 = row_start_x + prefix_w + gap;

        /* 4 个 time_set_box 背景 + 独立数字滚动条（0~9） */
        for(int i = 0; i < 4; i++) {
            lv_coord_t tx;
            if(i == 0) tx = 0;
            else if(i == 1) tx = tile_w + gap;
            else if(i == 2) tx = (tile_w + gap) * 2 + colon_w;
            else tx = (tile_w + gap) * 3 + colon_w;

            /* time_set_box 背景图 */
            g_admin_dormancy_tp_digit_imgs[i] = lv_image_create(g_admin_dormancy_tp_wrap);
            lv_image_set_src(g_admin_dormancy_tp_digit_imgs[i], &time_set_box);
            lv_obj_set_size(g_admin_dormancy_tp_digit_imgs[i], tile_w, tile_h);
            lv_obj_set_pos(g_admin_dormancy_tp_digit_imgs[i], tiles_x0 + tx, row_y);

            /* 数字滚动条叠在图上 */
            g_admin_dormancy_tp_digit_rollers[i] = lv_roller_create(g_admin_dormancy_tp_wrap);
            lv_roller_set_visible_row_count(g_admin_dormancy_tp_digit_rollers[i], 1);
            lv_obj_set_style_bg_opa(g_admin_dormancy_tp_digit_rollers[i], LV_OPA_TRANSP, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(g_admin_dormancy_tp_digit_rollers[i], LV_OPA_TRANSP, LV_PART_SELECTED);
            lv_obj_set_style_text_color(g_admin_dormancy_tp_digit_rollers[i], lv_color_hex(COL_TEXT), LV_PART_MAIN);
            lv_obj_set_style_text_color(g_admin_dormancy_tp_digit_rollers[i], lv_color_hex(COL_TEXT), LV_PART_SELECTED);
            lv_obj_set_style_border_width(g_admin_dormancy_tp_digit_rollers[i], 0, LV_PART_MAIN);
            lv_obj_set_style_pad_all(g_admin_dormancy_tp_digit_rollers[i], 0, LV_PART_MAIN);
            lv_obj_set_style_text_line_space(g_admin_dormancy_tp_digit_rollers[i], 30, LV_PART_MAIN);
            lv_obj_set_style_text_line_space(g_admin_dormancy_tp_digit_rollers[i], 30, LV_PART_SELECTED);
            lv_obj_set_style_radius(g_admin_dormancy_tp_digit_rollers[i], 0, LV_PART_MAIN);
            lv_obj_set_style_text_font(g_admin_dormancy_tp_digit_rollers[i], s_font_sc_70, LV_PART_MAIN);
            lv_obj_set_style_text_font(g_admin_dormancy_tp_digit_rollers[i], s_font_sc_70, LV_PART_SELECTED);
            lv_obj_set_style_text_align(g_admin_dormancy_tp_digit_rollers[i], LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
            lv_obj_set_style_text_align(g_admin_dormancy_tp_digit_rollers[i], LV_TEXT_ALIGN_CENTER, LV_PART_SELECTED);
            /* 十分位 0~5、个位 0~9 → 分钟/秒最大 59 */
            lv_roller_set_options(g_admin_dormancy_tp_digit_rollers[i],
                (i == 0 || i == 2) ? TP_ROLLER_OPTS_0_5 : TP_ROLLER_OPTS_0_9,
                LV_ROLLER_MODE_NORMAL);
            lv_obj_set_size(g_admin_dormancy_tp_digit_rollers[i], tile_w, tile_h);
            lv_obj_set_pos(g_admin_dormancy_tp_digit_rollers[i], tiles_x0 + tx, row_y);
            lv_obj_add_event_cb(g_admin_dormancy_tp_digit_rollers[i],
                cb_admin_dormancy_tp_digit_roller_changed, LV_EVENT_VALUE_CHANGED, NULL);
        }

        /* 冒号 ":" */
        g_admin_dormancy_tp_lbl_colon = lv_label_create(g_admin_dormancy_tp_wrap);
        lv_label_set_text(g_admin_dormancy_tp_lbl_colon, ":");
        lv_obj_set_style_text_color(g_admin_dormancy_tp_lbl_colon, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_dormancy_tp_lbl_colon, s_font_sc_70);
        lv_obj_update_layout(g_admin_dormancy_tp_lbl_colon);
        lv_obj_set_pos(g_admin_dormancy_tp_lbl_colon,
            tiles_x0 + (tile_w + gap) * 2 + (colon_w - lv_obj_get_width(g_admin_dormancy_tp_lbl_colon)) / 2 + (-2),
            row_y + (tile_h - lv_obj_get_height(g_admin_dormancy_tp_lbl_colon)) / 2 + (-10));
    }

    /* ---- 右侧按钮：确定（填充）+ 取消（描边） ---- */
    {
        const lv_coord_t btn_w = 110;
        const lv_coord_t btn_h = 50;
        const lv_coord_t btn_y = -40;//y偏移，越大越往下
        g_admin_dormancy_tp_btn_ok = make_orange_fill_btn(g_admin_dormancy_tp_wrap,
            ui_translation(STR_BTN_CONFIRM), btn_w, btn_h);
        lv_obj_align(g_admin_dormancy_tp_btn_ok, LV_ALIGN_RIGHT_MID, -300, -50 + btn_y);
        ui_set_obj_font(lv_obj_get_child(g_admin_dormancy_tp_btn_ok, 0), s_font_sc_30);
        orange_btn_bind_i18n(g_admin_dormancy_tp_btn_ok, STR_BTN_CONFIRM);
        lv_obj_add_event_cb(g_admin_dormancy_tp_btn_ok, cb_admin_dormancy_tp_confirm, LV_EVENT_CLICKED, NULL);

        g_admin_dormancy_tp_btn_cancel = make_orange_outline_btn(g_admin_dormancy_tp_wrap,
            ui_translation(STR_BTN_CANCEL), btn_w, btn_h);
        lv_obj_align(g_admin_dormancy_tp_btn_cancel, LV_ALIGN_RIGHT_MID, -300, 50 + btn_y);
        ui_set_obj_font(lv_obj_get_child(g_admin_dormancy_tp_btn_cancel, 0), s_font_sc_30);
        orange_btn_bind_i18n(g_admin_dormancy_tp_btn_cancel, STR_BTN_CANCEL);
        lv_obj_add_event_cb(g_admin_dormancy_tp_btn_cancel, cb_admin_dormancy_tp_cancel, LV_EVENT_CLICKED, NULL);
    }

    /* 屏幕亮度子面板（同待机时间：title_box + set_box；主文字+开关+说明+滑条） */
    g_admin_panel_brightness = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_brightness, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_brightness, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_brightness, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_brightness, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_brightness, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_brightness, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_brightness, LV_OBJ_FLAG_HIDDEN);

    g_admin_img_brightness_title_box = lv_image_create(g_admin_panel_brightness);
    lv_image_set_src(g_admin_img_brightness_title_box, &title_box);
    lv_obj_align(g_admin_img_brightness_title_box, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_lbl_brightness_title = lv_label_create(g_admin_panel_brightness);
    ui_lang_bind_label(g_admin_lbl_brightness_title, STR_ADMIN_M1_BRIGHTNESS);
    lv_obj_set_style_text_color(g_admin_lbl_brightness_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_brightness_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_brightness_title, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_brightness_set_box_wrap = lv_obj_create(g_admin_panel_brightness);
    lv_obj_set_size(g_admin_brightness_set_box_wrap, 1117, 409);
    lv_obj_align(g_admin_brightness_set_box_wrap, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(g_admin_brightness_set_box_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_brightness_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_brightness_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_brightness_set_box_wrap, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_admin_brightness_set_box_wrap, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    lv_obj_t * img_brightness_set_box = lv_image_create(g_admin_brightness_set_box_wrap);
    lv_image_set_src(img_brightness_set_box, &set_box);
    lv_obj_center(img_brightness_set_box);

    /* ===== 与待机时间页相同的行位置/间距；滑条右端对齐标题行（方案二） ===== */
    const lv_coord_t bright_row_w   = 960;
    const lv_coord_t bright_row1_y  = 85;   /* 同待机时间「时间设置」行距 set_box 顶 */
    const lv_coord_t bright_row_pad = 50;   /* 同待机时间：主文字与小字间距 */
    const lv_coord_t bright_icon_gap = 16;  /* 太阳图标与滑条间距 */
    const lv_coord_t bright_area_h = BRIGHT_PAGE_TRACK_H + 8;
    /* 行容器右侧多 SIDE_PAD：轨道右端对齐标题/开关右缘，旋钮防裁切区落在外侧 */
    const lv_coord_t bright_row_outer_w = bright_row_w + BRIGHT_PAGE_SIDE_PAD;
    const lv_coord_t bright_row_x_ofs = BRIGHT_PAGE_SIDE_PAD / 2;

    lv_obj_t * bright_row = lv_obj_create(g_admin_brightness_set_box_wrap);
    lv_obj_set_size(bright_row, bright_row_outer_w, LV_SIZE_CONTENT);
    lv_obj_align(bright_row, LV_ALIGN_TOP_MID, bright_row_x_ofs, bright_row1_y);
    lv_obj_set_style_bg_opa(bright_row, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(bright_row, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(bright_row, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(bright_row, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(bright_row, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(bright_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(bright_row, bright_row_pad, LV_PART_MAIN);
    lv_obj_clear_flag(bright_row, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(bright_row, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    /* 主文字 + 开关 同一行（文字左、开关右、垂直居中） */
    lv_obj_t * bright_title_line = lv_obj_create(bright_row);
    lv_obj_set_size(bright_title_line, bright_row_w, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(bright_title_line, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(bright_title_line, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(bright_title_line, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(bright_title_line, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(bright_title_line, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bright_title_line, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(bright_title_line, LV_OBJ_FLAG_SCROLLABLE);

    g_admin_lbl_brightness_main = lv_label_create(bright_title_line);
    ui_lang_bind_label(g_admin_lbl_brightness_main, STR_ADMIN_M1_BRIGHTNESS);
    lv_obj_set_style_text_color(g_admin_lbl_brightness_main, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_brightness_main, s_font_sc_30);

    g_admin_sw_run_always_on = lv_switch_create(bright_title_line);
    admin_data_style_switch(g_admin_sw_run_always_on);
    lv_obj_add_event_cb(g_admin_sw_run_always_on, cb_admin_brightness_switch_changed, LV_EVENT_VALUE_CHANGED, NULL);

    /* 说明小字：一整段，随显示框自动换行 */
    g_admin_lbl_brightness_line1 = lv_label_create(bright_row);
    ui_lang_bind_label(g_admin_lbl_brightness_line1, STR_BRIGHTNESS_LINE1);
    lv_obj_set_style_text_color(g_admin_lbl_brightness_line1, lv_color_hex(COL_SETTING_HINT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_brightness_line1, s_font_sc_27);
    lv_obj_set_width(g_admin_lbl_brightness_line1, bright_row_w - 100);
    lv_label_set_long_mode(g_admin_lbl_brightness_line1, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_line_space(g_admin_lbl_brightness_line1, 20, LV_PART_MAIN);
    g_admin_lbl_brightness_line2 = NULL;

    /* 太阳图标 + 滑条（行宽含右侧 SIDE_PAD，轨道右端对齐开关右缘） */
    lv_obj_t * bright_slider_row = lv_obj_create(bright_row);
    lv_obj_set_size(bright_slider_row, bright_row_outer_w, bright_area_h);
    lv_obj_set_style_bg_opa(bright_slider_row, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(bright_slider_row, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(bright_slider_row, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(bright_slider_row, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(bright_slider_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bright_slider_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(bright_slider_row, bright_icon_gap, LV_PART_MAIN);
    lv_obj_clear_flag(bright_slider_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(bright_slider_row, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    lv_obj_t * img_bright_logo = lv_image_create(bright_slider_row);
    lv_image_set_src(img_bright_logo, &bright_logo);
    lv_obj_remove_flag(img_bright_logo, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t * brightness_slider_area = lv_obj_create(bright_slider_row);
    lv_obj_set_flex_grow(brightness_slider_area, 1);
    lv_obj_set_height(brightness_slider_area, bright_area_h);
    lv_obj_set_style_bg_opa(brightness_slider_area, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(brightness_slider_area, 0, LV_PART_MAIN);
    /* 左/右均留 SIDE_PAD；外宽多出右侧 SIDE_PAD → 轨道右端对齐标题行右缘 */
    lv_obj_set_style_pad_hor(brightness_slider_area, BRIGHT_PAGE_SIDE_PAD, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(brightness_slider_area, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(brightness_slider_area, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_clear_flag(brightness_slider_area, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(brightness_slider_area, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    /* 底层：深灰胶囊轨（与滑条同宽同高） */
    lv_obj_t * brightness_track = lv_obj_create(brightness_slider_area);
    lv_obj_set_size(brightness_track, LV_PCT(100), BRIGHT_PAGE_TRACK_H);
    lv_obj_align(brightness_track, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(brightness_track, lv_color_hex(BRIGHT_PAGE_TRACK_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(brightness_track, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(brightness_track, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(brightness_track, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(brightness_track, 0, LV_PART_MAIN);
    lv_obj_clear_flag(brightness_track, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    /* 中层裁剪窗：宽度随数值变；内层整轨固定黑→蓝渐变（右端蓝不随滑钮变色） */
    g_admin_brightness_fill = lv_obj_create(brightness_slider_area);
    lv_obj_set_size(g_admin_brightness_fill, 0, BRIGHT_PAGE_TRACK_H);
    lv_obj_align(g_admin_brightness_fill, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_opa(g_admin_brightness_fill, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_radius(g_admin_brightness_fill, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_clip_corner(g_admin_brightness_fill, true, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_brightness_fill, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_brightness_fill, 0, LV_PART_MAIN);
    lv_obj_clear_flag(g_admin_brightness_fill, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    g_admin_brightness_fill_grad = lv_obj_create(g_admin_brightness_fill);
    lv_obj_set_size(g_admin_brightness_fill_grad, LV_PCT(100), BRIGHT_PAGE_TRACK_H);
    lv_obj_align(g_admin_brightness_fill_grad, LV_ALIGN_LEFT_MID, 0, 0);
    {
        const lv_color_t stops_c[] = {
            lv_color_hex(BRIGHT_PAGE_FILL_0),
            lv_color_hex(BRIGHT_PAGE_FILL_43),
            lv_color_hex(BRIGHT_PAGE_FILL_77),
            lv_color_hex(BRIGHT_PAGE_FILL_100),
        };
        const uint8_t stops_frac[] = {
            0,                              /* 0% */
            (uint8_t)(255 * 43 / 100),      /* 43% */
            (uint8_t)(255 * 77 / 100),      /* 77% */
            255,                            /* 100% */
        };
        lv_grad_init_stops(&s_bright_page_fill_grad, stops_c, NULL, stops_frac, 4);
        lv_grad_horizontal_init(&s_bright_page_fill_grad);
        lv_obj_set_style_bg_grad(g_admin_brightness_fill_grad, &s_bright_page_fill_grad, LV_PART_MAIN);
    }
    lv_obj_set_style_bg_opa(g_admin_brightness_fill_grad, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(g_admin_brightness_fill_grad, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_brightness_fill_grad, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_brightness_fill_grad, 0, LV_PART_MAIN);
    lv_obj_clear_flag(g_admin_brightness_fill_grad, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    /* 顶层：透明轨滑条（仅旋钮可见 + 触摸） */
    g_admin_slider_brightness = lv_slider_create(brightness_slider_area);
    lv_obj_set_size(g_admin_slider_brightness, LV_PCT(100), BRIGHT_PAGE_TRACK_H);
    lv_obj_align(g_admin_slider_brightness, LV_ALIGN_LEFT_MID, 0, 0);
    lv_slider_set_range(g_admin_slider_brightness, 0, 100);
    /*
     * 左右 pad=半个旋钮：旋钮圆心在内容区两端移动，
     * 底层灰轨仍铺满整控件 → 最左/最右时轨边与旋钮外沿对齐。
     */
    lv_obj_set_style_pad_hor(g_admin_slider_brightness, BRIGHT_PAGE_TRACK_H / 2, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(g_admin_slider_brightness, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_admin_slider_brightness, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_radius(g_admin_slider_brightness, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_slider_brightness, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_admin_slider_brightness, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_pad_all(g_admin_slider_brightness, 0, LV_PART_INDICATOR);
    /* 旋钮：直径=轨高；握纹见 BRIGHT_GRIP_LINE_* */
    lv_obj_set_style_bg_color(g_admin_slider_brightness, lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    lv_obj_set_style_bg_opa(g_admin_slider_brightness, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_width(g_admin_slider_brightness, 0, LV_PART_KNOB);
    lv_obj_set_style_radius(g_admin_slider_brightness, LV_RADIUS_CIRCLE, LV_PART_KNOB);
    lv_obj_set_style_pad_all(g_admin_slider_brightness, BRIGHT_PAGE_KNOB_PAD, LV_PART_KNOB);
    lv_obj_set_style_shadow_width(g_admin_slider_brightness, 0, LV_PART_KNOB);

    lv_obj_add_flag(g_admin_slider_brightness, LV_OBJ_FLAG_SEND_DRAW_TASK_EVENTS | LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_obj_add_event_cb(g_admin_slider_brightness, cb_admin_brightness_slider_draw_grip,
                        LV_EVENT_DRAW_TASK_ADDED, NULL);
    lv_obj_add_event_cb(g_admin_slider_brightness, cb_admin_brightness_slider_changed, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(g_admin_slider_brightness, cb_admin_brightness_slider_size_changed, LV_EVENT_SIZE_CHANGED, NULL);
    lv_obj_add_event_cb(g_admin_slider_brightness, cb_admin_brightness_slider_ext_draw_size,
                        LV_EVENT_REFR_EXT_DRAW_SIZE, NULL);
    lv_obj_refresh_ext_draw_size(g_admin_slider_brightness);

    admin_brightness_sync_ui();

    /* 声音控制子面板（同待机时间：title_box + set_box；两项开关 + 亮度样式滑条，仅触摸） */
    g_admin_panel_sound = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_sound, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_sound, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_sound, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_sound, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_sound, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_sound, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_sound, LV_OBJ_FLAG_HIDDEN);

    g_admin_img_sound_title_box = lv_image_create(g_admin_panel_sound);
    lv_image_set_src(g_admin_img_sound_title_box, &title_box);
    lv_obj_align(g_admin_img_sound_title_box, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_lbl_sound_title = lv_label_create(g_admin_panel_sound);
    ui_lang_bind_label(g_admin_lbl_sound_title, STR_ADMIN_M1_SOUND);
    lv_obj_set_style_text_color(g_admin_lbl_sound_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_sound_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_sound_title, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_sound_set_box_wrap = lv_obj_create(g_admin_panel_sound);
    lv_obj_set_size(g_admin_sound_set_box_wrap, 1117, 409);
    lv_obj_align(g_admin_sound_set_box_wrap, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(g_admin_sound_set_box_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_sound_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_sound_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_sound_set_box_wrap, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_admin_sound_set_box_wrap, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    lv_obj_t * img_sound_set_box = lv_image_create(g_admin_sound_set_box_wrap);
    lv_image_set_src(img_sound_set_box, &set_box);
    lv_obj_center(img_sound_set_box);

    /* ===== 与待机时间页相同的行位置；行宽同亮度页（含图标+滑条） ===== */
    const lv_coord_t sound_row_w   = 800;
    const lv_coord_t sound_row1_y  = 85;   /* 同待机时间「时间设置」行距 set_box 顶 */
    const lv_coord_t sound_sep_y   = 220;  /* 同待机时间横线 */
    const lv_coord_t sound_row2_y  = 250;  /* 同待机时间「不熄屏」行 */
    const lv_coord_t sound_row_pad = 30;   /* 主文字行与图标/滑条间距（无小字） */
    const lv_coord_t sound_icon_gap = 16;  /* 同亮度页：图标与滑条间距 */
    const lv_coord_t sound_area_h = BRIGHT_PAGE_TRACK_H + 8;
    /* 行容器右侧多 SIDE_PAD：轨道右端对齐白线，旋钮防裁切区落在线外 */
    const lv_coord_t sound_row_outer_w = sound_row_w + BRIGHT_PAGE_SIDE_PAD;
    const lv_coord_t sound_row_x_ofs = BRIGHT_PAGE_SIDE_PAD / 2; /* TOP_MID 偏移，使左缘仍对齐白线 */

    /* 行1：触控声音（主文字+开关，下方 touch_sound_logo + 滑条） */
    lv_obj_t * sound_row1 = lv_obj_create(g_admin_sound_set_box_wrap);
    lv_obj_set_size(sound_row1, sound_row_outer_w, LV_SIZE_CONTENT);
    lv_obj_align(sound_row1, LV_ALIGN_TOP_MID, sound_row_x_ofs, sound_row1_y);
    lv_obj_set_style_bg_opa(sound_row1, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(sound_row1, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(sound_row1, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(sound_row1, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(sound_row1, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(sound_row1, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(sound_row1, sound_row_pad, LV_PART_MAIN);
    lv_obj_clear_flag(sound_row1, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(sound_row1, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    lv_obj_t * sound_title_line1 = lv_obj_create(sound_row1);
    lv_obj_set_size(sound_title_line1, sound_row_w, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(sound_title_line1, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(sound_title_line1, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(sound_title_line1, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(sound_title_line1, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(sound_title_line1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(sound_title_line1, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(sound_title_line1, LV_OBJ_FLAG_SCROLLABLE);

    g_admin_lbl_sound_line1 = lv_label_create(sound_title_line1);
    ui_lang_bind_label(g_admin_lbl_sound_line1, STR_SOUND_TOUCH);
    lv_obj_set_style_text_color(g_admin_lbl_sound_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_sound_line1, s_font_sc_30);

    g_admin_sw_touch_sound = lv_switch_create(sound_title_line1);
    admin_data_style_switch(g_admin_sw_touch_sound);
    lv_obj_add_event_cb(g_admin_sw_touch_sound, cb_admin_sound_touch_switch_changed, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t * sound_slider_row1 = lv_obj_create(sound_row1);
    lv_obj_set_size(sound_slider_row1, sound_row_outer_w, sound_area_h);
    lv_obj_set_style_bg_opa(sound_slider_row1, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(sound_slider_row1, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(sound_slider_row1, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(sound_slider_row1, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(sound_slider_row1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(sound_slider_row1, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(sound_slider_row1, sound_icon_gap, LV_PART_MAIN);
    lv_obj_clear_flag(sound_slider_row1, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(sound_slider_row1, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    lv_obj_t * img_touch_sound_logo = lv_image_create(sound_slider_row1);
    lv_image_set_src(img_touch_sound_logo, &touch_sound_logo);
    lv_obj_remove_flag(img_touch_sound_logo, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t * touch_sound_slider_area = lv_obj_create(sound_slider_row1);
    lv_obj_set_flex_grow(touch_sound_slider_area, 1);
    lv_obj_set_height(touch_sound_slider_area, sound_area_h);
    lv_obj_set_style_bg_opa(touch_sound_slider_area, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(touch_sound_slider_area, 0, LV_PART_MAIN);
    /* 左/右均留 SIDE_PAD；行宽多出右侧 SIDE_PAD → 轨道右端对齐白线 */
    lv_obj_set_style_pad_hor(touch_sound_slider_area, BRIGHT_PAGE_SIDE_PAD, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(touch_sound_slider_area, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(touch_sound_slider_area, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_clear_flag(touch_sound_slider_area, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(touch_sound_slider_area, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    admin_bright_style_slider_build(touch_sound_slider_area,
                                    &g_admin_touch_sound_volume_fill,
                                    &g_admin_touch_sound_volume_fill_grad,
                                    &g_admin_slider_touch_sound_volume,
                                    cb_admin_sound_touch_sound_volume_slider_changed);
    lv_obj_add_event_cb(g_admin_slider_touch_sound_volume, cb_admin_sound_slider_size_changed,
                        LV_EVENT_SIZE_CHANGED, NULL);

    /* 两项之间的白色横线 */
    lv_obj_t * sound_sep = lv_obj_create(g_admin_sound_set_box_wrap);
    lv_obj_set_size(sound_sep, sound_row_w, 1);
    lv_obj_align(sound_sep, LV_ALIGN_TOP_MID, 0, sound_sep_y);
    lv_obj_set_style_bg_color(sound_sep, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sound_sep, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(sound_sep, 0, LV_PART_MAIN);
    lv_obj_clear_flag(sound_sep, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    /* 行2：声音播报（主文字+开关，下方 sound_logo + 滑条） */
    lv_obj_t * sound_row2 = lv_obj_create(g_admin_sound_set_box_wrap);
    lv_obj_set_size(sound_row2, sound_row_outer_w, LV_SIZE_CONTENT);
    lv_obj_align(sound_row2, LV_ALIGN_TOP_MID, sound_row_x_ofs, sound_row2_y);
    lv_obj_set_style_bg_opa(sound_row2, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(sound_row2, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(sound_row2, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(sound_row2, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(sound_row2, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(sound_row2, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(sound_row2, sound_row_pad, LV_PART_MAIN);
    lv_obj_clear_flag(sound_row2, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(sound_row2, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    lv_obj_t * sound_title_line2 = lv_obj_create(sound_row2);
    lv_obj_set_size(sound_title_line2, sound_row_w, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(sound_title_line2, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(sound_title_line2, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(sound_title_line2, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(sound_title_line2, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(sound_title_line2, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(sound_title_line2, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(sound_title_line2, LV_OBJ_FLAG_SCROLLABLE);

    g_admin_lbl_sound_line2 = lv_label_create(sound_title_line2);
    ui_lang_bind_label(g_admin_lbl_sound_line2, STR_SOUND_VOICE);
    lv_obj_set_style_text_color(g_admin_lbl_sound_line2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_sound_line2, s_font_sc_30);

    g_admin_sw_voice_broadcast = lv_switch_create(sound_title_line2);
    admin_data_style_switch(g_admin_sw_voice_broadcast);
    lv_obj_add_event_cb(g_admin_sw_voice_broadcast, cb_admin_sound_voice_broadcast_switch_changed,
                        LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t * sound_slider_row2 = lv_obj_create(sound_row2);
    lv_obj_set_size(sound_slider_row2, sound_row_outer_w, sound_area_h);
    lv_obj_set_style_bg_opa(sound_slider_row2, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(sound_slider_row2, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(sound_slider_row2, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(sound_slider_row2, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(sound_slider_row2, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(sound_slider_row2, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(sound_slider_row2, sound_icon_gap, LV_PART_MAIN);
    lv_obj_clear_flag(sound_slider_row2, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(sound_slider_row2, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    lv_obj_t * img_sound_logo = lv_image_create(sound_slider_row2);
    lv_image_set_src(img_sound_logo, &sound_logo);
    lv_obj_remove_flag(img_sound_logo, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t * voice_sound_slider_area = lv_obj_create(sound_slider_row2);
    lv_obj_set_flex_grow(voice_sound_slider_area, 1);
    lv_obj_set_height(voice_sound_slider_area, sound_area_h);
    lv_obj_set_style_bg_opa(voice_sound_slider_area, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(voice_sound_slider_area, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(voice_sound_slider_area, BRIGHT_PAGE_SIDE_PAD, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(voice_sound_slider_area, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(voice_sound_slider_area, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_clear_flag(voice_sound_slider_area, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(voice_sound_slider_area, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    admin_bright_style_slider_build(voice_sound_slider_area,
                                    &g_admin_sound_volume_fill,
                                    &g_admin_sound_volume_fill_grad,
                                    &g_admin_slider_sound_volume,
                                    cb_admin_sound_volume_slider_changed);
    lv_obj_add_event_cb(g_admin_slider_sound_volume, cb_admin_sound_slider_size_changed,
                        LV_EVENT_SIZE_CHANGED, NULL);

    admin_sound_sync_ui();

    /* 语言设置子面板（title_box/set_box 同待机时间；右侧按钮同 ID；说明文案同 WIFI 样式） */
    g_admin_panel_lang = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_lang, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_lang, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_lang, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_lang, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_lang, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_lang, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_lang, LV_OBJ_FLAG_HIDDEN);

    g_admin_img_lang_title_box = lv_image_create(g_admin_panel_lang);
    lv_image_set_src(g_admin_img_lang_title_box, &title_box);
    lv_obj_align(g_admin_img_lang_title_box, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_lbl_lang_title = lv_label_create(g_admin_panel_lang);
    ui_lang_bind_label(g_admin_lbl_lang_title, STR_ADMIN_LANG_TITLE);
    lv_obj_set_style_text_color(g_admin_lbl_lang_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_lang_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_lang_title, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_lang_set_box_wrap = lv_obj_create(g_admin_panel_lang);
    lv_obj_set_size(g_admin_lang_set_box_wrap, 1117, 409);
    lv_obj_align(g_admin_lang_set_box_wrap, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(g_admin_lang_set_box_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_lang_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_lang_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_lang_set_box_wrap, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * img_lang_set_box = lv_image_create(g_admin_lang_set_box_wrap);
    lv_image_set_src(img_lang_set_box, &set_box);
    lv_obj_center(img_lang_set_box);

    /* 说明文字：与 WIFI prompt 相同宽/字号/居中（在 set_box 内） */
    g_admin_lbl_lang_line1 = lv_label_create(g_admin_lang_set_box_wrap);
    ui_lang_bind_label(g_admin_lbl_lang_line1, STR_ADMIN_LANG_HINT);
    lv_obj_set_style_text_color(g_admin_lbl_lang_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_lang_line1, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_lang_line1, 900);
    lv_label_set_long_mode(g_admin_lbl_lang_line1, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(g_admin_lbl_lang_line1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_center(g_admin_lbl_lang_line1);

    /* 右侧按钮：与 ID 设置 mid_btn 完全一致（110×50，CENTER 400/±35），文字改为中文/英文 */
    const lv_coord_t lang_btn_w = 110;
    const lv_coord_t lang_btn_h = 50;
    g_admin_btn_lang_zh = make_orange_fill_btn(g_admin_lang_set_box_wrap, ui_translation(STR_ADMIN_LANG_BTN_ZH), lang_btn_w, lang_btn_h);
    lv_obj_align(g_admin_btn_lang_zh, LV_ALIGN_CENTER, 400, -35);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_lang_zh, 0), s_font_sc_30);
    ui_lang_bind_label(lv_obj_get_child(g_admin_btn_lang_zh, 0), STR_ADMIN_LANG_BTN_ZH);
    lv_obj_add_event_cb(g_admin_btn_lang_zh, cb_admin_lang_zh, LV_EVENT_CLICKED, NULL);

    g_admin_btn_lang_en = make_orange_fill_btn(g_admin_lang_set_box_wrap, ui_translation(STR_ADMIN_LANG_BTN_EN), lang_btn_w, lang_btn_h);
    lv_obj_align(g_admin_btn_lang_en, LV_ALIGN_CENTER, 400, 35);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_lang_en, 0), s_font_sc_30);
    ui_lang_bind_label(lv_obj_get_child(g_admin_btn_lang_en, 0), STR_ADMIN_LANG_BTN_EN);
    lv_obj_add_event_cb(g_admin_btn_lang_en, cb_admin_lang_en, LV_EVENT_CLICKED, NULL);

    if(g_admin_btn_lang_zh != NULL) lv_obj_move_foreground(g_admin_btn_lang_zh);
    if(g_admin_btn_lang_en != NULL) lv_obj_move_foreground(g_admin_btn_lang_en);
    admin_lang_sync_btn_ui();

    /* 恢复默认子面板（title_box/set_box 同待机时间；确定/取消同 ID） */
    g_admin_panel_factory = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_factory, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_factory, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_factory, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_factory, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_factory, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_factory, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_factory, LV_OBJ_FLAG_HIDDEN);

    g_admin_img_factory_title_box = lv_image_create(g_admin_panel_factory);
    lv_image_set_src(g_admin_img_factory_title_box, &title_box);
    lv_obj_align(g_admin_img_factory_title_box, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_lbl_factory_title = lv_label_create(g_admin_panel_factory);
    ui_lang_bind_label(g_admin_lbl_factory_title, STR_ADMIN_M1_FACTORY_RESET);
    lv_obj_set_style_text_color(g_admin_lbl_factory_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_factory_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_factory_title, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_factory_set_box_wrap = lv_obj_create(g_admin_panel_factory);
    lv_obj_set_size(g_admin_factory_set_box_wrap, 1117, 409);
    lv_obj_align(g_admin_factory_set_box_wrap, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(g_admin_factory_set_box_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_factory_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_factory_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_factory_set_box_wrap, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * img_factory_set_box = lv_image_create(g_admin_factory_set_box_wrap);
    lv_image_set_src(img_factory_set_box, &set_box);
    lv_obj_center(img_factory_set_box);

    /* 第一页：左侧两行确认文案（左对齐，与右侧按钮垂直居中） */
    g_admin_lbl_factory_line1 = lv_label_create(g_admin_factory_set_box_wrap);
    ui_lang_bind_label(g_admin_lbl_factory_line1, STR_FACTORY_CONFIRM_Q);
    lv_obj_set_style_text_color(g_admin_lbl_factory_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_factory_line1, s_font_sc_30);
    lv_obj_set_style_text_align(g_admin_lbl_factory_line1, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_align(g_admin_lbl_factory_line1, LV_ALIGN_LEFT_MID, 100, -30);

    g_admin_lbl_factory_line2 = lv_label_create(g_admin_factory_set_box_wrap);
    ui_lang_bind_label(g_admin_lbl_factory_line2, STR_FACTORY_CONFIRM_HINT);
    lv_obj_set_style_text_color(g_admin_lbl_factory_line2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_factory_line2, s_font_sc_30);
    lv_obj_set_style_text_align(g_admin_lbl_factory_line2, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_align(g_admin_lbl_factory_line2, LV_ALIGN_LEFT_MID, 100, 30);

    /* 第二/三页：状态文案居中（恢复中 / 完成） */
    g_admin_lbl_factory_status = lv_label_create(g_admin_factory_set_box_wrap);
    lv_label_set_text(g_admin_lbl_factory_status, ui_translation(STR_FACTORY_RESTORING));
    lv_obj_set_style_text_color(g_admin_lbl_factory_status, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_factory_status, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_factory_status, 900);
    lv_label_set_long_mode(g_admin_lbl_factory_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(g_admin_lbl_factory_status, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_center(g_admin_lbl_factory_status);
    lv_obj_add_flag(g_admin_lbl_factory_status, LV_OBJ_FLAG_HIDDEN);

    /* 确定/取消：与 ID 设置 mid_btn 完全一致（110×50，CENTER 400/±35） */
    const lv_coord_t factory_btn_w = 110;
    const lv_coord_t factory_btn_h = 50;
    g_admin_btn_factory_ok = make_orange_fill_btn(g_admin_factory_set_box_wrap,
        ui_translation(STR_BTN_OK), factory_btn_w, factory_btn_h);
    lv_obj_align(g_admin_btn_factory_ok, LV_ALIGN_CENTER, 400, -35);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_factory_ok, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_factory_ok, STR_BTN_OK);
    lv_obj_add_event_cb(g_admin_btn_factory_ok, cb_admin_factory_ok, LV_EVENT_CLICKED, NULL);

    g_admin_btn_factory_cancel = make_orange_outline_btn(g_admin_factory_set_box_wrap,
        ui_translation(STR_BTN_CANCEL), factory_btn_w, factory_btn_h);
    lv_obj_align(g_admin_btn_factory_cancel, LV_ALIGN_CENTER, 400, 35);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_factory_cancel, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_factory_cancel, STR_BTN_CANCEL);
    lv_obj_add_event_cb(g_admin_btn_factory_cancel, cb_admin_factory_cancel, LV_EVENT_CLICKED, NULL);

    if(g_admin_btn_factory_ok != NULL) lv_obj_move_foreground(g_admin_btn_factory_ok);
    if(g_admin_btn_factory_cancel != NULL) lv_obj_move_foreground(g_admin_btn_factory_cancel);

    /* 联系我们子面板（title_box/set_box 同待机时间；文案/二维码坐标保持原值） */
    g_admin_panel_contact = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_contact, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_contact, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_contact, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_contact, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_contact, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_contact, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_contact, LV_OBJ_FLAG_HIDDEN);

    g_admin_img_contact_title_box = lv_image_create(g_admin_panel_contact);
    lv_image_set_src(g_admin_img_contact_title_box, &title_box);
    lv_obj_align(g_admin_img_contact_title_box, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_lbl_contact_title = lv_label_create(g_admin_panel_contact);
    ui_lang_bind_label(g_admin_lbl_contact_title, STR_ADMIN_M1_CONTACT);
    lv_obj_set_style_text_color(g_admin_lbl_contact_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_contact_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_contact_title, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_contact_set_box_wrap = lv_obj_create(g_admin_panel_contact);
    lv_obj_set_size(g_admin_contact_set_box_wrap, 1117, 409);
    lv_obj_align(g_admin_contact_set_box_wrap, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(g_admin_contact_set_box_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_contact_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_contact_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_contact_set_box_wrap, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * img_contact_set_box = lv_image_create(g_admin_contact_set_box_wrap);
    lv_image_set_src(img_contact_set_box, &set_box);
    lv_obj_center(img_contact_set_box);

    /* 两行文案 + 二维码：整体相对原坐标下移 100px */
    g_admin_lbl_contact_line1 = lv_label_create(g_admin_panel_contact);
    ui_lang_bind_label(g_admin_lbl_contact_line1, STR_CONTACT_HOTLINE);
    lv_obj_set_style_text_color(g_admin_lbl_contact_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_contact_line1, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_contact_line1, 400, 230);

    g_admin_lbl_contact_line2 = lv_label_create(g_admin_panel_contact);
    ui_lang_bind_label(g_admin_lbl_contact_line2, STR_CONTACT_SLOGAN);
    lv_obj_set_style_text_color(g_admin_lbl_contact_line2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_contact_line2, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_contact_line2, 400, 285);

    g_admin_img_contact_qr = lv_image_create(g_admin_panel_contact);
    lv_image_set_src(g_admin_img_contact_qr, &QR_code_xiaoya);
    lv_obj_set_size(g_admin_img_contact_qr, 160, 160); /* 小鸭二维码大小 */
    lv_obj_set_pos(g_admin_img_contact_qr, 1040, 190);
    lv_image_set_inner_align(g_admin_img_contact_qr, LV_IMAGE_ALIGN_STRETCH);

    if(g_admin_lbl_contact_line1 != NULL) lv_obj_move_foreground(g_admin_lbl_contact_line1);
    if(g_admin_lbl_contact_line2 != NULL) lv_obj_move_foreground(g_admin_lbl_contact_line2);
    if(g_admin_img_contact_qr != NULL) lv_obj_move_foreground(g_admin_img_contact_qr);

    /* 防缠绕功能子面板（1600×400，左文右钮竖排，布局同恢复默认/语言设置） */
    g_admin_panel_auto_dispense = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_auto_dispense, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_auto_dispense, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_auto_dispense, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_auto_dispense, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_auto_dispense, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_auto_dispense, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_auto_dispense, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_auto_dispense_title = lv_label_create(g_admin_panel_auto_dispense);
    ui_lang_bind_label(g_admin_lbl_auto_dispense_title, STR_ADMIN_M2_AUTO_DISPENSE);
    lv_obj_set_style_text_color(g_admin_lbl_auto_dispense_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_auto_dispense_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_auto_dispense_title, LV_ALIGN_TOP_LEFT, 350, 30);
    lv_obj_t * auto_dispense_box = admin_panel_apply_shell(g_admin_panel_auto_dispense, g_admin_lbl_auto_dispense_title);

    g_admin_lbl_auto_dispense_line1 = lv_label_create(auto_dispense_box);
    ui_lang_bind_label(g_admin_lbl_auto_dispense_line1, STR_AUTO_DISP_LINE1);
    lv_obj_set_style_text_color(g_admin_lbl_auto_dispense_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_auto_dispense_line1, s_font_sc_30);
    lv_obj_set_style_text_align(g_admin_lbl_auto_dispense_line1, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_align(g_admin_lbl_auto_dispense_line1, LV_ALIGN_LEFT_MID, 100, 0);

    const lv_coord_t auto_disp_btn_w = 110;
    const lv_coord_t auto_disp_btn_h = 50;
    g_admin_btn_auto_dispense_on = make_orange_fill_btn(auto_dispense_box, ui_translation(STR_BTN_ON), auto_disp_btn_w, auto_disp_btn_h);
    lv_obj_align(g_admin_btn_auto_dispense_on, LV_ALIGN_CENTER, 400, -35);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_auto_dispense_on, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_auto_dispense_on, STR_BTN_ON);
    lv_obj_add_event_cb(g_admin_btn_auto_dispense_on, cb_admin_auto_dispense_on, LV_EVENT_CLICKED, NULL);

    g_admin_btn_auto_dispense_off = make_orange_fill_btn(auto_dispense_box, ui_translation(STR_BTN_OFF), auto_disp_btn_w, auto_disp_btn_h);
    lv_obj_align(g_admin_btn_auto_dispense_off, LV_ALIGN_CENTER, 400, 35);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_auto_dispense_off, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_auto_dispense_off, STR_BTN_OFF);
    lv_obj_add_event_cb(g_admin_btn_auto_dispense_off, cb_admin_auto_dispense_off, LV_EVENT_CLICKED, NULL);

    if(g_admin_btn_auto_dispense_on != NULL) lv_obj_move_foreground(g_admin_btn_auto_dispense_on);
    if(g_admin_btn_auto_dispense_off != NULL) lv_obj_move_foreground(g_admin_btn_auto_dispense_off);

    admin_auto_dispense_sync_btn_ui();

    /* 新风护理子面板（1600×400，左文右钮竖排，布局同防缠绕功能） */
    g_admin_panel_fresh_air_care = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_fresh_air_care, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_fresh_air_care, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_fresh_air_care, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_fresh_air_care, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_fresh_air_care, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_fresh_air_care, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_fresh_air_care, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_fresh_air_care_title = lv_label_create(g_admin_panel_fresh_air_care);
    ui_lang_bind_label(g_admin_lbl_fresh_air_care_title, STR_ADMIN_M2_FRESH_AIR);
    lv_obj_set_style_text_color(g_admin_lbl_fresh_air_care_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_fresh_air_care_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_fresh_air_care_title, LV_ALIGN_TOP_LEFT, 350, 30);
    lv_obj_t * fresh_air_box = admin_panel_apply_shell(g_admin_panel_fresh_air_care, g_admin_lbl_fresh_air_care_title);

    g_admin_lbl_fresh_air_care_line1 = lv_label_create(fresh_air_box);
    ui_lang_bind_label(g_admin_lbl_fresh_air_care_line1, STR_FRESH_AIR_LINE1);
    lv_obj_set_style_text_color(g_admin_lbl_fresh_air_care_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_fresh_air_care_line1, s_font_sc_30);
    lv_obj_set_style_text_align(g_admin_lbl_fresh_air_care_line1, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_align(g_admin_lbl_fresh_air_care_line1, LV_ALIGN_LEFT_MID, 100, 0);

    const lv_coord_t fresh_air_btn_w = 110;
    const lv_coord_t fresh_air_btn_h = 50;
    g_admin_btn_fresh_air_care_on = make_orange_fill_btn(fresh_air_box, ui_translation(STR_BTN_ON), fresh_air_btn_w, fresh_air_btn_h);
    lv_obj_align(g_admin_btn_fresh_air_care_on, LV_ALIGN_CENTER, 400, -35);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_fresh_air_care_on, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_fresh_air_care_on, STR_BTN_ON);
    lv_obj_add_event_cb(g_admin_btn_fresh_air_care_on, cb_admin_fresh_air_care_on, LV_EVENT_CLICKED, NULL);

    g_admin_btn_fresh_air_care_off = make_orange_fill_btn(fresh_air_box, ui_translation(STR_BTN_OFF), fresh_air_btn_w, fresh_air_btn_h);
    lv_obj_align(g_admin_btn_fresh_air_care_off, LV_ALIGN_CENTER, 400, 35);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_fresh_air_care_off, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_fresh_air_care_off, STR_BTN_OFF);
    lv_obj_add_event_cb(g_admin_btn_fresh_air_care_off, cb_admin_fresh_air_care_off, LV_EVENT_CLICKED, NULL);

    if(g_admin_btn_fresh_air_care_on != NULL) lv_obj_move_foreground(g_admin_btn_fresh_air_care_on);
    if(g_admin_btn_fresh_air_care_off != NULL) lv_obj_move_foreground(g_admin_btn_fresh_air_care_off);

    admin_fresh_air_care_sync_btn_ui();

    /* 系统升级子面板（title_box/set_box 同待机时间；确定钮同 ID 取消） */
    g_admin_panel_system_upgrade = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_system_upgrade, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_system_upgrade, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_system_upgrade, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_system_upgrade, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_system_upgrade, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_system_upgrade, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_system_upgrade, LV_OBJ_FLAG_HIDDEN);

    g_admin_img_system_upgrade_title_box = lv_image_create(g_admin_panel_system_upgrade);
    lv_image_set_src(g_admin_img_system_upgrade_title_box, &title_box);
    lv_obj_align(g_admin_img_system_upgrade_title_box, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_lbl_system_upgrade_title = lv_label_create(g_admin_panel_system_upgrade);
    ui_lang_bind_label(g_admin_lbl_system_upgrade_title, STR_ADMIN_M2_UPGRADE);
    lv_obj_set_style_text_color(g_admin_lbl_system_upgrade_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_system_upgrade_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_system_upgrade_title, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_system_upgrade_set_box_wrap = lv_obj_create(g_admin_panel_system_upgrade);
    lv_obj_set_size(g_admin_system_upgrade_set_box_wrap, 1117, 409);
    lv_obj_align(g_admin_system_upgrade_set_box_wrap, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(g_admin_system_upgrade_set_box_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_system_upgrade_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_system_upgrade_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_system_upgrade_set_box_wrap, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * img_system_upgrade_set_box = lv_image_create(g_admin_system_upgrade_set_box_wrap);
    lv_image_set_src(img_system_upgrade_set_box, &set_box);
    lv_obj_center(img_system_upgrade_set_box);

    /* 第一页：左侧确认文案（与右侧按钮垂直居中） */
    g_admin_lbl_system_upgrade_line1 = lv_label_create(g_admin_system_upgrade_set_box_wrap);
    ui_lang_bind_label(g_admin_lbl_system_upgrade_line1, STR_UPGRADE_CONFIRM_Q);
    lv_obj_set_style_text_color(g_admin_lbl_system_upgrade_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_system_upgrade_line1, s_font_sc_30);
    lv_obj_align(g_admin_lbl_system_upgrade_line1, LV_ALIGN_CENTER, -200, 0);

    /* 第二/三页：状态文案居中（升级中 / 已是最新） */
    g_admin_lbl_system_upgrade_status = lv_label_create(g_admin_system_upgrade_set_box_wrap);
    lv_label_set_text(g_admin_lbl_system_upgrade_status, ui_translation(STR_UPGRADE_IN_PROGRESS));
    lv_obj_set_style_text_color(g_admin_lbl_system_upgrade_status, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_system_upgrade_status, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_system_upgrade_status, 900);
    lv_label_set_long_mode(g_admin_lbl_system_upgrade_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(g_admin_lbl_system_upgrade_status, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_center(g_admin_lbl_system_upgrade_status);
    lv_obj_add_flag(g_admin_lbl_system_upgrade_status, LV_OBJ_FLAG_HIDDEN);

    /* 确定：与 ID 取消按钮完全一致（outline、110×50、右侧 x=400），y=0 与文案垂直居中 */
    const lv_coord_t upgrade_btn_w = 110;
    const lv_coord_t upgrade_btn_h = 50;
    g_admin_btn_system_upgrade_ok = make_orange_outline_btn(g_admin_system_upgrade_set_box_wrap,
        ui_translation(STR_BTN_OK), upgrade_btn_w, upgrade_btn_h);
    lv_obj_align(g_admin_btn_system_upgrade_ok, LV_ALIGN_CENTER, 400, 0);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_system_upgrade_ok, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_system_upgrade_ok, STR_BTN_OK);
    lv_obj_add_event_cb(g_admin_btn_system_upgrade_ok, cb_admin_system_upgrade_ok, LV_EVENT_CLICKED, NULL);

    if(g_admin_btn_system_upgrade_ok != NULL) {
        lv_obj_move_foreground(g_admin_btn_system_upgrade_ok);
    }

    /* 支付设置子面板（同待机时间：title_box + set_box；首页3项 / 方式2项 / 超时数字页） */
    g_admin_panel_payment = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_payment, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_payment, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_payment, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_payment, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_payment, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_payment, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_payment, LV_OBJ_FLAG_HIDDEN);

    g_admin_img_payment_title_box = lv_image_create(g_admin_panel_payment);
    lv_image_set_src(g_admin_img_payment_title_box, &title_box);
    lv_obj_align(g_admin_img_payment_title_box, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_lbl_payment_title = lv_label_create(g_admin_panel_payment);
    ui_lang_bind_label(g_admin_lbl_payment_title, STR_ADMIN_M2_PAYMENT);
    lv_obj_set_style_text_color(g_admin_lbl_payment_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_payment_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_payment_title, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_payment_set_box_wrap = lv_obj_create(g_admin_panel_payment);
    lv_obj_set_size(g_admin_payment_set_box_wrap, 1117, 409);
    lv_obj_align(g_admin_payment_set_box_wrap, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(g_admin_payment_set_box_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_payment_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_payment_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_payment_set_box_wrap, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * img_payment_set_box = lv_image_create(g_admin_payment_set_box_wrap);
    lv_image_set_src(img_payment_set_box, &set_box);
    lv_obj_center(img_payment_set_box);

    /* 首页：与数据设置相同的行高60 + 1px横线 + pad_row=0；3项+3线，整体居中于 set_box */
    const lv_coord_t pay_row_w = 800;
    const lv_coord_t pay_list_h = 60 * 3 + 1 * 3; /* 183 */

    g_admin_payment_list_view = lv_obj_create(g_admin_payment_set_box_wrap);
    lv_obj_set_size(g_admin_payment_list_view, pay_row_w, pay_list_h);
    lv_obj_align(g_admin_payment_list_view, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(g_admin_payment_list_view, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_payment_list_view, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_payment_list_view, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(g_admin_payment_list_view, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(g_admin_payment_list_view, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_payment_list_view, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(g_admin_payment_list_view, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(g_admin_payment_list_view, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_set_scroll_dir(g_admin_payment_list_view, LV_DIR_NONE);
    lv_obj_set_scrollbar_mode(g_admin_payment_list_view, LV_SCROLLBAR_MODE_OFF);

    {
        static const ui_str_id_t pay_list_ids[3] = {
            STR_PAYMENT_METHOD, STR_PAYMENT_TIMEOUT, STR_PAYMENT_ORDER
        };
        lv_obj_t ** pay_lbls[3] = {
            &g_admin_lbl_payment_method, &g_admin_lbl_payment_timeout, &g_admin_lbl_payment_order
        };
        lv_obj_t ** pay_sws[3] = {
            &g_admin_sw_payment_method, &g_admin_sw_payment_timeout, &g_admin_sw_payment_order
        };
        for(int i = 0; i < 3; i++) {
            lv_obj_t * row = lv_obj_create(g_admin_payment_list_view);
            lv_obj_set_size(row, pay_row_w, 60);
            lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, LV_PART_MAIN);
            lv_obj_set_style_border_width(row, 0, LV_PART_MAIN);
            lv_obj_set_style_pad_all(row, 0, LV_PART_MAIN);
            lv_obj_set_style_layout(row, LV_LAYOUT_FLEX, LV_PART_MAIN);
            lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

            *pay_lbls[i] = lv_label_create(row);
            ui_lang_bind_label(*pay_lbls[i], pay_list_ids[i]);
            lv_obj_set_style_text_color(*pay_lbls[i], lv_color_hex(COL_TEXT), LV_PART_MAIN);
            ui_set_obj_font(*pay_lbls[i], s_font_sc_30);

            *pay_sws[i] = lv_switch_create(row);
            admin_data_style_switch(*pay_sws[i]);
            lv_obj_add_event_cb(*pay_sws[i], cb_admin_payment_list_sw_changed, LV_EVENT_VALUE_CHANGED, NULL);

            /* 每项下方一条横线（共 3 条） */
            lv_obj_t * sep = lv_obj_create(g_admin_payment_list_view);
            lv_obj_set_size(sep, pay_row_w, 1);
            lv_obj_set_style_bg_color(sep, lv_color_hex(COL_DIM), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(sep, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_border_width(sep, 0, LV_PART_MAIN);
            lv_obj_clear_flag(sep, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        }
    }

    /* 支付方式页：微信/支付宝 + logo；文字与横线间隔大于首页（参考自投行距） */
    g_admin_payment_method_view = lv_obj_create(g_admin_payment_set_box_wrap);
    lv_obj_set_size(g_admin_payment_method_view, LV_PCT(100), LV_PCT(100));
    lv_obj_align(g_admin_payment_method_view, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(g_admin_payment_method_view, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_payment_method_view, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_payment_method_view, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_payment_method_view, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_payment_method_view, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_admin_payment_method_view, LV_OBJ_FLAG_HIDDEN);

    {
        const lv_coord_t pay_m_row_w = 800;
        const lv_coord_t pay_m_title_h = 48;
        const lv_coord_t pay_m_row1_y = 120;  /* 整体居中于 set_box */
        const lv_coord_t pay_m_sep_y  = 200;  /* 比首页文字-横线间隔更大 */
        const lv_coord_t pay_m_row2_y = 240;
        const lv_coord_t pay_m_logo_w = 29;   /* wechat_logo / alipay_logo 原图 */
        const lv_coord_t pay_m_logo_h = 30;
        const lv_coord_t pay_m_icon_text_gap = 10;

        lv_obj_t * pay_wechat_row = lv_obj_create(g_admin_payment_method_view);
        lv_obj_set_size(pay_wechat_row, pay_m_row_w, pay_m_title_h);
        lv_obj_align(pay_wechat_row, LV_ALIGN_TOP_MID, 0, pay_m_row1_y);
        lv_obj_set_style_bg_opa(pay_wechat_row, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(pay_wechat_row, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(pay_wechat_row, 0, LV_PART_MAIN);
        lv_obj_set_style_layout(pay_wechat_row, LV_LAYOUT_FLEX, LV_PART_MAIN);
        lv_obj_set_flex_flow(pay_wechat_row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(pay_wechat_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_add_flag(pay_wechat_row, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
        lv_obj_clear_flag(pay_wechat_row, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

        lv_obj_t * img_wechat = lv_image_create(pay_wechat_row);
        lv_image_set_src(img_wechat, &wechat_logo);
        lv_obj_set_size(img_wechat, pay_m_logo_w, pay_m_logo_h);
        lv_obj_add_flag(img_wechat, LV_OBJ_FLAG_IGNORE_LAYOUT);
        lv_obj_align(img_wechat, LV_ALIGN_LEFT_MID, 0, 0);

        g_admin_lbl_payment_wechat = lv_label_create(pay_wechat_row);
        ui_lang_bind_label(g_admin_lbl_payment_wechat, STR_PAYMENT_WECHAT);
        lv_obj_set_style_text_color(g_admin_lbl_payment_wechat, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_payment_wechat, s_font_sc_30);
        lv_obj_set_style_margin_left(g_admin_lbl_payment_wechat,
            pay_m_logo_w + pay_m_icon_text_gap, LV_PART_MAIN);

        g_admin_sw_payment_wechat = lv_switch_create(pay_wechat_row);
        admin_data_style_switch(g_admin_sw_payment_wechat);
        lv_obj_add_event_cb(g_admin_sw_payment_wechat, cb_admin_payment_wechat_changed,
            LV_EVENT_VALUE_CHANGED, NULL);

        lv_obj_t * pay_method_sep = lv_obj_create(g_admin_payment_method_view);
        lv_obj_set_size(pay_method_sep, pay_m_row_w, 1);
        lv_obj_align(pay_method_sep, LV_ALIGN_TOP_MID, 0, pay_m_sep_y);
        lv_obj_set_style_bg_color(pay_method_sep, lv_color_hex(COL_DIM), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(pay_method_sep, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(pay_method_sep, 0, LV_PART_MAIN);
        lv_obj_clear_flag(pay_method_sep, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * pay_alipay_row = lv_obj_create(g_admin_payment_method_view);
        lv_obj_set_size(pay_alipay_row, pay_m_row_w, pay_m_title_h);
        lv_obj_align(pay_alipay_row, LV_ALIGN_TOP_MID, 0, pay_m_row2_y);
        lv_obj_set_style_bg_opa(pay_alipay_row, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(pay_alipay_row, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(pay_alipay_row, 0, LV_PART_MAIN);
        lv_obj_set_style_layout(pay_alipay_row, LV_LAYOUT_FLEX, LV_PART_MAIN);
        lv_obj_set_flex_flow(pay_alipay_row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(pay_alipay_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_add_flag(pay_alipay_row, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
        lv_obj_clear_flag(pay_alipay_row, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

        lv_obj_t * img_alipay = lv_image_create(pay_alipay_row);
        lv_image_set_src(img_alipay, &alipay_logo);
        lv_obj_set_size(img_alipay, pay_m_logo_w, pay_m_logo_h);
        lv_obj_add_flag(img_alipay, LV_OBJ_FLAG_IGNORE_LAYOUT);
        lv_obj_align(img_alipay, LV_ALIGN_LEFT_MID, 0, 0);

        g_admin_lbl_payment_alipay = lv_label_create(pay_alipay_row);
        ui_lang_bind_label(g_admin_lbl_payment_alipay, STR_PAYMENT_ALIPAY);
        lv_obj_set_style_text_color(g_admin_lbl_payment_alipay, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_payment_alipay, s_font_sc_30);
        lv_obj_set_style_margin_left(g_admin_lbl_payment_alipay,
            pay_m_logo_w + pay_m_icon_text_gap, LV_PART_MAIN);

        g_admin_sw_payment_alipay = lv_switch_create(pay_alipay_row);
        admin_data_style_switch(g_admin_sw_payment_alipay);
        lv_obj_add_event_cb(g_admin_sw_payment_alipay, cb_admin_payment_alipay_changed,
            LV_EVENT_VALUE_CHANGED, NULL);
    }

    /* 订单查询列表（图1：5 行门店+状态，居中于 set_box） */
    g_admin_payment_orders_view = lv_obj_create(g_admin_payment_set_box_wrap);
    lv_obj_set_size(g_admin_payment_orders_view, LV_PCT(100), LV_PCT(100));
    lv_obj_align(g_admin_payment_orders_view, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(g_admin_payment_orders_view, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_payment_orders_view, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_payment_orders_view, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_payment_orders_view, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_payment_orders_view, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_admin_payment_orders_view, LV_OBJ_FLAG_HIDDEN);

    {
        const lv_coord_t ord_row_w = 960;
        const lv_coord_t ord_row_h = 60;
        const lv_coord_t ord_list_h = ord_row_h * ADMIN_PAYMENT_ORDER_CNT + 1 * (ADMIN_PAYMENT_ORDER_CNT - 1);
        lv_obj_t * ord_list = lv_obj_create(g_admin_payment_orders_view);
        lv_obj_set_size(ord_list, ord_row_w, ord_list_h);
        lv_obj_align(ord_list, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_opa(ord_list, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(ord_list, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(ord_list, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_row(ord_list, 0, LV_PART_MAIN);
        lv_obj_set_style_layout(ord_list, LV_LAYOUT_FLEX, LV_PART_MAIN);
        lv_obj_set_flex_flow(ord_list, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(ord_list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
        lv_obj_set_scroll_dir(ord_list, LV_DIR_NONE);
        lv_obj_set_scrollbar_mode(ord_list, LV_SCROLLBAR_MODE_OFF);

        for(int i = 0; i < ADMIN_PAYMENT_ORDER_CNT; i++) {
            const admin_payment_order_demo_t * o = &g_admin_payment_order_demo[i];
            g_admin_payment_order_rows[i] = lv_obj_create(ord_list);
            lv_obj_set_size(g_admin_payment_order_rows[i], ord_row_w, ord_row_h);
            lv_obj_set_style_bg_opa(g_admin_payment_order_rows[i], LV_OPA_TRANSP, LV_PART_MAIN);
            lv_obj_set_style_border_width(g_admin_payment_order_rows[i], 0, LV_PART_MAIN);
            lv_obj_set_style_pad_all(g_admin_payment_order_rows[i], 0, LV_PART_MAIN);
            lv_obj_set_style_layout(g_admin_payment_order_rows[i], LV_LAYOUT_NONE, LV_PART_MAIN);
            lv_obj_clear_flag(g_admin_payment_order_rows[i], LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(g_admin_payment_order_rows[i], LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(g_admin_payment_order_rows[i], cb_admin_payment_order_row_clicked,
                LV_EVENT_CLICKED, (void *)(intptr_t)i);

            lv_obj_t * title = lv_label_create(g_admin_payment_order_rows[i]);
            lv_label_set_text(title, o->title);
            lv_obj_set_style_text_color(title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
            ui_set_obj_font(title, s_font_sc_30);
            lv_obj_align(title, LV_ALIGN_LEFT_MID, 0, 0);
            lv_obj_add_flag(title, LV_OBJ_FLAG_EVENT_BUBBLE);

            if(o->running) {
                lv_obj_t * badge = lv_obj_create(g_admin_payment_order_rows[i]);
                lv_obj_set_size(badge, LV_SIZE_CONTENT, 36);
                lv_obj_set_style_bg_color(badge, lv_color_hex(COL_ORANGE), LV_PART_MAIN);
                lv_obj_set_style_bg_opa(badge, LV_OPA_COVER, LV_PART_MAIN);
                lv_obj_set_style_radius(badge, 6, LV_PART_MAIN);
                lv_obj_set_style_border_width(badge, 0, LV_PART_MAIN);
                lv_obj_set_style_pad_hor(badge, 14, LV_PART_MAIN);
                lv_obj_set_style_pad_ver(badge, 4, LV_PART_MAIN);
                lv_obj_clear_flag(badge, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_flag(badge, LV_OBJ_FLAG_EVENT_BUBBLE);
                lv_obj_align(badge, LV_ALIGN_RIGHT_MID, 0, 0);

                lv_obj_t * st = lv_label_create(badge);
                ui_lang_bind_label(st, STR_ORDER_STATUS_RUNNING);
                lv_obj_set_style_text_color(st, lv_color_hex(COL_TEXT), LV_PART_MAIN);
                ui_set_obj_font(st, s_font_sc_20);
                lv_obj_center(st);
                lv_obj_add_flag(st, LV_OBJ_FLAG_EVENT_BUBBLE);
            }
            else {
                lv_obj_t * st = lv_label_create(g_admin_payment_order_rows[i]);
                ui_lang_bind_label(st, STR_ORDER_STATUS_DONE);
                lv_obj_set_style_text_color(st, lv_color_hex(COL_TEXT), LV_PART_MAIN);
                ui_set_obj_font(st, s_font_sc_20);
                lv_obj_align(st, LV_ALIGN_RIGHT_MID, -13, 0);
                lv_obj_add_flag(st, LV_OBJ_FLAG_EVENT_BUBBLE);
            }

            if(i < ADMIN_PAYMENT_ORDER_CNT - 1) {
                lv_obj_t * sep = lv_obj_create(ord_list);
                lv_obj_set_size(sep, ord_row_w, 1);
                lv_obj_set_style_bg_color(sep, lv_color_hex(COL_DIM), LV_PART_MAIN);
                lv_obj_set_style_bg_opa(sep, LV_OPA_COVER, LV_PART_MAIN);
                lv_obj_set_style_border_width(sep, 0, LV_PART_MAIN);
                lv_obj_clear_flag(sep, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
            }
        }
    }

    /* 订单摘要中间页（图2：门店/程序概要 + 开发票/订单详情） */
    g_admin_payment_order_summary_view = lv_obj_create(g_admin_payment_set_box_wrap);
    lv_obj_set_size(g_admin_payment_order_summary_view, LV_PCT(100), LV_PCT(100));
    lv_obj_align(g_admin_payment_order_summary_view, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(g_admin_payment_order_summary_view, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_payment_order_summary_view, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_payment_order_summary_view, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_payment_order_summary_view, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_payment_order_summary_view, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_admin_payment_order_summary_view, LV_OBJ_FLAG_HIDDEN);

    {
        const lv_coord_t sum_w = 960;
        const lv_coord_t sum_x = (1117 - sum_w) / 2;
        const lv_coord_t sum_y0 = 20;
        const lv_coord_t sum_btn_w = 110;
        const lv_coord_t sum_btn_h = 36;
        const lv_coord_t sum_btn_gap = 16;

        /* 顶行：门店标题 + 状态徽章（同详情页） */
        lv_obj_t * sum_hdr = lv_obj_create(g_admin_payment_order_summary_view);
        lv_obj_set_size(sum_hdr, sum_w, 48);
        lv_obj_set_pos(sum_hdr, sum_x, 20 + sum_y0);
        lv_obj_set_style_bg_opa(sum_hdr, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(sum_hdr, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(sum_hdr, 0, LV_PART_MAIN);
        lv_obj_set_style_layout(sum_hdr, LV_LAYOUT_FLEX, LV_PART_MAIN);
        lv_obj_set_flex_flow(sum_hdr, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(sum_hdr, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(sum_hdr, LV_OBJ_FLAG_SCROLLABLE);

        g_admin_lbl_order_sum_title = lv_label_create(sum_hdr);
        lv_obj_set_style_text_color(g_admin_lbl_order_sum_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_order_sum_title, s_font_sc_30);

        g_admin_order_sum_status_badge = lv_obj_create(sum_hdr);
        lv_obj_set_size(g_admin_order_sum_status_badge, LV_SIZE_CONTENT, 36);
        lv_obj_set_style_bg_color(g_admin_order_sum_status_badge, lv_color_hex(COL_ORANGE), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(g_admin_order_sum_status_badge, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_radius(g_admin_order_sum_status_badge, 6, LV_PART_MAIN);
        lv_obj_set_style_border_width(g_admin_order_sum_status_badge, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_hor(g_admin_order_sum_status_badge, 14, LV_PART_MAIN);
        lv_obj_set_style_pad_ver(g_admin_order_sum_status_badge, 4, LV_PART_MAIN);
        lv_obj_clear_flag(g_admin_order_sum_status_badge, LV_OBJ_FLAG_SCROLLABLE);

        g_admin_lbl_order_sum_status = lv_label_create(g_admin_order_sum_status_badge);
        lv_obj_set_style_text_color(g_admin_lbl_order_sum_status, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_order_sum_status, s_font_sc_20);
        lv_obj_center(g_admin_lbl_order_sum_status);

        lv_obj_t * sum_sep1 = lv_obj_create(g_admin_payment_order_summary_view);
        lv_obj_set_size(sum_sep1, sum_w, 1);
        lv_obj_set_pos(sum_sep1, sum_x, 78 + sum_y0);
        lv_obj_set_style_bg_color(sum_sep1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(sum_sep1, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(sum_sep1, 0, LV_PART_MAIN);
        lv_obj_clear_flag(sum_sep1, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * img_sum_wm = lv_image_create(g_admin_payment_order_summary_view);
        lv_image_set_src(img_sum_wm, &washing_machine);
        lv_obj_set_pos(img_sum_wm, sum_x, 95 + sum_y0);

        g_admin_lbl_order_sum_prog = lv_label_create(g_admin_payment_order_summary_view);
        lv_obj_set_style_text_color(g_admin_lbl_order_sum_prog, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_order_sum_prog, s_font_sc_30);
        lv_obj_set_pos(g_admin_lbl_order_sum_prog, sum_x + 220, 130 + sum_y0);

        g_admin_lbl_order_sum_sub = lv_label_create(g_admin_payment_order_summary_view);
        lv_obj_set_style_text_color(g_admin_lbl_order_sum_sub, lv_color_hex(COL_SETTING_HINT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_order_sum_sub, s_font_sc_30);
        lv_obj_set_pos(g_admin_lbl_order_sum_sub, sum_x + 220, 175 + sum_y0);

        lv_obj_t * sum_sep2 = lv_obj_create(g_admin_payment_order_summary_view);
        lv_obj_set_size(sum_sep2, sum_w, 1);
        lv_obj_set_pos(sum_sep2, sum_x, 250 + sum_y0);
        lv_obj_set_style_bg_color(sum_sep2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(sum_sep2, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(sum_sep2, 0, LV_PART_MAIN);
        lv_obj_clear_flag(sum_sep2, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * sum_foot = lv_obj_create(g_admin_payment_order_summary_view);
        lv_obj_set_size(sum_foot, sum_w, 40);
        lv_obj_set_pos(sum_foot, sum_x, 265 + sum_y0);
        lv_obj_set_style_bg_opa(sum_foot, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(sum_foot, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(sum_foot, 0, LV_PART_MAIN);
        lv_obj_set_style_layout(sum_foot, LV_LAYOUT_FLEX, LV_PART_MAIN);
        lv_obj_set_flex_flow(sum_foot, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(sum_foot, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(sum_foot, LV_OBJ_FLAG_SCROLLABLE);

        g_admin_lbl_order_sum_time = lv_label_create(sum_foot);
        lv_obj_set_style_text_color(g_admin_lbl_order_sum_time, lv_color_hex(COL_SETTING_HINT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_order_sum_time, s_font_sc_30);

        g_admin_lbl_order_sum_total = lv_label_create(sum_foot);
        lv_obj_set_style_text_color(g_admin_lbl_order_sum_total, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_order_sum_total, s_font_sc_30);

        /* 宽度随文字（中/英文差异大），左右留 18px 内边距，位置由布局函数右对齐 */
        g_admin_btn_order_invoice = make_orange_outline_btn(g_admin_payment_order_summary_view,
            ui_translation(STR_ORDER_INVOICE), LV_SIZE_CONTENT, sum_btn_h);
        lv_obj_set_style_radius(g_admin_btn_order_invoice, 18, LV_PART_MAIN);
        lv_obj_set_style_border_width(g_admin_btn_order_invoice, 2, LV_PART_MAIN);
        lv_obj_set_style_pad_hor(g_admin_btn_order_invoice, 18, LV_PART_MAIN);
        lv_obj_set_pos(g_admin_btn_order_invoice,
            sum_x + sum_w - sum_btn_w * 2 - sum_btn_gap + 20, 325 + sum_y0);
        ui_set_obj_font(lv_obj_get_child(g_admin_btn_order_invoice, 0), s_font_sc_20);
        lv_obj_set_style_text_color(lv_obj_get_child(g_admin_btn_order_invoice, 0),
            lv_color_hex(COL_ORANGE), LV_PART_MAIN);
        orange_btn_bind_i18n(g_admin_btn_order_invoice, STR_ORDER_INVOICE);
        lv_obj_add_event_cb(g_admin_btn_order_invoice, cb_admin_payment_order_invoice, LV_EVENT_CLICKED, NULL);

        g_admin_btn_order_detail = make_orange_outline_btn(g_admin_payment_order_summary_view,
            ui_translation(STR_ORDER_DETAIL_BTN), LV_SIZE_CONTENT, sum_btn_h);
        lv_obj_set_style_radius(g_admin_btn_order_detail, 18, LV_PART_MAIN);
        lv_obj_set_style_border_width(g_admin_btn_order_detail, 2, LV_PART_MAIN);
        lv_obj_set_style_pad_hor(g_admin_btn_order_detail, 18, LV_PART_MAIN);
        lv_obj_set_pos(g_admin_btn_order_detail, sum_x + sum_w - sum_btn_w, 325 + sum_y0);
        ui_set_obj_font(lv_obj_get_child(g_admin_btn_order_detail, 0), s_font_sc_20);
        lv_obj_set_style_text_color(lv_obj_get_child(g_admin_btn_order_detail, 0),
            lv_color_hex(COL_ORANGE), LV_PART_MAIN);
        orange_btn_bind_i18n(g_admin_btn_order_detail, STR_ORDER_DETAIL_BTN);
        lv_obj_add_event_cb(g_admin_btn_order_detail, cb_admin_payment_order_detail_btn, LV_EVENT_CLICKED, NULL);

        admin_payment_order_summary_btns_layout();
    }

    /* 订单详情（图3：标题条 + washing_machine + 程序/金额/时间） */
    g_admin_payment_order_detail_view = lv_obj_create(g_admin_payment_set_box_wrap);
    lv_obj_set_size(g_admin_payment_order_detail_view, LV_PCT(100), LV_PCT(100));
    lv_obj_align(g_admin_payment_order_detail_view, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(g_admin_payment_order_detail_view, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_payment_order_detail_view, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_payment_order_detail_view, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_payment_order_detail_view, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_payment_order_detail_view, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_admin_payment_order_detail_view, LV_OBJ_FLAG_HIDDEN);

    {
        const lv_coord_t det_w = 960;
        const lv_coord_t det_x = (1117 - det_w) / 2;
        const lv_coord_t det_y0 = 15;  /* 订单详情区整体下移，越大越往下 */

        /* 顶行：橙色竖条 + 订单详情 + 状态徽章 */
        lv_obj_t * hdr = lv_obj_create(g_admin_payment_order_detail_view);
        lv_obj_set_size(hdr, det_w, 40);
        lv_obj_set_pos(hdr, det_x, 28 + det_y0);
        lv_obj_set_style_bg_opa(hdr, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(hdr, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(hdr, 0, LV_PART_MAIN);
        lv_obj_set_style_layout(hdr, LV_LAYOUT_FLEX, LV_PART_MAIN);
        lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * hdr_left = lv_obj_create(hdr);
        lv_obj_set_size(hdr_left, LV_SIZE_CONTENT, LV_PCT(100));
        lv_obj_set_style_bg_opa(hdr_left, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(hdr_left, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(hdr_left, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_column(hdr_left, 10, LV_PART_MAIN);
        lv_obj_set_style_layout(hdr_left, LV_LAYOUT_FLEX, LV_PART_MAIN);
        lv_obj_set_flex_flow(hdr_left, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(hdr_left, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(hdr_left, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * bar = lv_obj_create(hdr_left);
        lv_obj_set_size(bar, 5, 26);
        lv_obj_set_style_bg_color(bar, lv_color_hex(COL_ORANGE), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_radius(bar, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(bar, 0, LV_PART_MAIN);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

        g_admin_lbl_order_detail_id = lv_label_create(hdr_left);
        lv_obj_set_style_text_color(g_admin_lbl_order_detail_id, lv_color_hex(COL_DIM), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_order_detail_id, s_font_sc_30);

        g_admin_order_detail_status_badge = lv_obj_create(hdr);
        lv_obj_set_size(g_admin_order_detail_status_badge, LV_SIZE_CONTENT, 36);
        lv_obj_set_style_bg_color(g_admin_order_detail_status_badge, lv_color_hex(COL_ORANGE), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(g_admin_order_detail_status_badge, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_radius(g_admin_order_detail_status_badge, 6, LV_PART_MAIN);
        lv_obj_set_style_border_width(g_admin_order_detail_status_badge, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_hor(g_admin_order_detail_status_badge, 14, LV_PART_MAIN);
        lv_obj_set_style_pad_ver(g_admin_order_detail_status_badge, 4, LV_PART_MAIN);
        lv_obj_clear_flag(g_admin_order_detail_status_badge, LV_OBJ_FLAG_SCROLLABLE);

        g_admin_lbl_order_detail_status = lv_label_create(g_admin_order_detail_status_badge);
        lv_obj_set_style_text_color(g_admin_lbl_order_detail_status, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_order_detail_status, s_font_sc_20);
        lv_obj_center(g_admin_lbl_order_detail_status);

        /* 中部：洗衣机图 + 程序名/参数 */
        lv_obj_t * img_wm = lv_image_create(g_admin_payment_order_detail_view);
        lv_image_set_src(img_wm, &washing_machine);
        lv_obj_set_pos(img_wm, det_x, 78 + det_y0);

        g_admin_lbl_order_detail_prog = lv_label_create(g_admin_payment_order_detail_view);
        lv_obj_set_style_text_color(g_admin_lbl_order_detail_prog, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_order_detail_prog, s_font_sc_30);
        lv_obj_set_pos(g_admin_lbl_order_detail_prog, det_x + 220, 120 + det_y0);

        g_admin_lbl_order_detail_params = lv_label_create(g_admin_payment_order_detail_view);
        lv_obj_set_style_text_color(g_admin_lbl_order_detail_params, lv_color_hex(COL_DIM), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_order_detail_params, s_font_sc_30);
        lv_obj_set_pos(g_admin_lbl_order_detail_params, det_x + 220, 165 + det_y0);

        /* 分隔线 */
        lv_obj_t * det_sep = lv_obj_create(g_admin_payment_order_detail_view);
        lv_obj_set_size(det_sep, det_w, 1);
        lv_obj_set_pos(det_sep, det_x, 240 + det_y0);
        lv_obj_set_style_bg_color(det_sep, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(det_sep, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(det_sep, 0, LV_PART_MAIN);
        lv_obj_clear_flag(det_sep, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

        /* 衣物件数 / 实付 */
        lv_obj_t * pay_row = lv_obj_create(g_admin_payment_order_detail_view);
        lv_obj_set_size(pay_row, det_w, 40);
        lv_obj_set_pos(pay_row, det_x, 255 + det_y0);
        lv_obj_set_style_bg_opa(pay_row, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(pay_row, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(pay_row, 0, LV_PART_MAIN);
        lv_obj_set_style_layout(pay_row, LV_LAYOUT_FLEX, LV_PART_MAIN);
        lv_obj_set_flex_flow(pay_row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(pay_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(pay_row, LV_OBJ_FLAG_SCROLLABLE);

        g_admin_lbl_order_detail_items = lv_label_create(pay_row);
        lv_obj_set_style_text_color(g_admin_lbl_order_detail_items, lv_color_hex(COL_DIM), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_order_detail_items, s_font_sc_30);

        g_admin_lbl_order_detail_paid = lv_label_create(pay_row);
        lv_obj_set_style_text_color(g_admin_lbl_order_detail_paid, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_order_detail_paid, s_font_sc_30);

        g_admin_lbl_order_detail_t_start = lv_label_create(g_admin_payment_order_detail_view);
        lv_obj_set_style_text_color(g_admin_lbl_order_detail_t_start, lv_color_hex(COL_DIM), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_order_detail_t_start, s_font_sc_30);
        lv_obj_set_pos(g_admin_lbl_order_detail_t_start, det_x, 305 + det_y0);

        g_admin_lbl_order_detail_t_end = lv_label_create(g_admin_payment_order_detail_view);
        lv_obj_set_style_text_color(g_admin_lbl_order_detail_t_end, lv_color_hex(COL_DIM), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_order_detail_t_end, s_font_sc_30);
        lv_obj_set_pos(g_admin_lbl_order_detail_t_end, det_x, 340 + det_y0);
    }

    /* 支付超时时间选择（布局同待机时间选择；文案「支付将在」「分钟后取消」） */
    g_admin_payment_tp_wrap = lv_obj_create(g_admin_panel_payment);
    lv_obj_set_size(g_admin_payment_tp_wrap, LV_PCT(100), body_h);
    lv_obj_align(g_admin_payment_tp_wrap, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(g_admin_payment_tp_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_payment_tp_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_payment_tp_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_payment_tp_wrap, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_payment_tp_wrap, LV_OBJ_FLAG_HIDDEN);

    {
        const lv_coord_t tile_w   = 78;
        const lv_coord_t tile_h   = 101;
        const lv_coord_t gap      = 8;
        const lv_coord_t colon_w  = 24;
        const lv_coord_t group_w  = tile_w * 4 + gap * 3 + colon_w;
        const lv_coord_t row_y    = 180;

        g_admin_payment_tp_lbl_prefix = lv_label_create(g_admin_payment_tp_wrap);
        ui_lang_bind_label(g_admin_payment_tp_lbl_prefix, STR_PAYMENT_TIME_PREFIX);
        lv_obj_set_style_text_color(g_admin_payment_tp_lbl_prefix, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_payment_tp_lbl_prefix, s_font_sc_50);

        g_admin_payment_tp_lbl_suffix = lv_label_create(g_admin_payment_tp_wrap);
        ui_lang_bind_label(g_admin_payment_tp_lbl_suffix, STR_PAYMENT_TIME_SUFFIX);
        lv_obj_set_style_text_color(g_admin_payment_tp_lbl_suffix, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_payment_tp_lbl_suffix, s_font_sc_50);

        lv_obj_update_layout(g_admin_payment_tp_lbl_prefix);
        lv_obj_update_layout(g_admin_payment_tp_lbl_suffix);
        const lv_coord_t prefix_w = lv_obj_get_width(g_admin_payment_tp_lbl_prefix);
        const lv_coord_t suffix_w = lv_obj_get_width(g_admin_payment_tp_lbl_suffix);
        const lv_coord_t row_total_w = prefix_w + gap + group_w + gap + suffix_w;
        const lv_coord_t row_start_x = (UI_FIXED_W - row_total_w) / 2 - 80;

        lv_obj_set_pos(g_admin_payment_tp_lbl_prefix, row_start_x - 15,
            row_y + (tile_h - lv_obj_get_height(g_admin_payment_tp_lbl_prefix)) / 2);
        lv_obj_set_pos(g_admin_payment_tp_lbl_suffix,
            row_start_x + prefix_w + gap + group_w + gap + 15,
            row_y + (tile_h - lv_obj_get_height(g_admin_payment_tp_lbl_suffix)) / 2);

        const lv_coord_t tiles_x0 = row_start_x + prefix_w + gap;
        for(int i = 0; i < 4; i++) {
            lv_coord_t tx;
            if(i == 0) tx = 0;
            else if(i == 1) tx = tile_w + gap;
            else if(i == 2) tx = (tile_w + gap) * 2 + colon_w;
            else tx = (tile_w + gap) * 3 + colon_w;

            g_admin_payment_tp_digit_imgs[i] = lv_image_create(g_admin_payment_tp_wrap);
            lv_image_set_src(g_admin_payment_tp_digit_imgs[i], &time_set_box);
            lv_obj_set_size(g_admin_payment_tp_digit_imgs[i], tile_w, tile_h);
            lv_obj_set_pos(g_admin_payment_tp_digit_imgs[i], tiles_x0 + tx, row_y);

            g_admin_payment_tp_digit_rollers[i] = lv_roller_create(g_admin_payment_tp_wrap);
            lv_roller_set_visible_row_count(g_admin_payment_tp_digit_rollers[i], 1);
            lv_obj_set_style_bg_opa(g_admin_payment_tp_digit_rollers[i], LV_OPA_TRANSP, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(g_admin_payment_tp_digit_rollers[i], LV_OPA_TRANSP, LV_PART_SELECTED);
            lv_obj_set_style_text_color(g_admin_payment_tp_digit_rollers[i], lv_color_hex(COL_TEXT), LV_PART_MAIN);
            lv_obj_set_style_text_color(g_admin_payment_tp_digit_rollers[i], lv_color_hex(COL_TEXT), LV_PART_SELECTED);
            lv_obj_set_style_border_width(g_admin_payment_tp_digit_rollers[i], 0, LV_PART_MAIN);
            lv_obj_set_style_pad_all(g_admin_payment_tp_digit_rollers[i], 0, LV_PART_MAIN);
            lv_obj_set_style_text_line_space(g_admin_payment_tp_digit_rollers[i], 30, LV_PART_MAIN);
            lv_obj_set_style_text_line_space(g_admin_payment_tp_digit_rollers[i], 30, LV_PART_SELECTED);
            lv_obj_set_style_radius(g_admin_payment_tp_digit_rollers[i], 0, LV_PART_MAIN);
            lv_obj_set_style_text_font(g_admin_payment_tp_digit_rollers[i], s_font_sc_70, LV_PART_MAIN);
            lv_obj_set_style_text_font(g_admin_payment_tp_digit_rollers[i], s_font_sc_70, LV_PART_SELECTED);
            lv_obj_set_style_text_align(g_admin_payment_tp_digit_rollers[i], LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
            lv_obj_set_style_text_align(g_admin_payment_tp_digit_rollers[i], LV_TEXT_ALIGN_CENTER, LV_PART_SELECTED);
            /* 十分位 0~5、个位 0~9 → 分钟/秒最大 59 */
            lv_roller_set_options(g_admin_payment_tp_digit_rollers[i],
                (i == 0 || i == 2) ? TP_ROLLER_OPTS_0_5 : TP_ROLLER_OPTS_0_9,
                LV_ROLLER_MODE_NORMAL);
            lv_obj_set_size(g_admin_payment_tp_digit_rollers[i], tile_w, tile_h);
            lv_obj_set_pos(g_admin_payment_tp_digit_rollers[i], tiles_x0 + tx, row_y);
            lv_obj_add_event_cb(g_admin_payment_tp_digit_rollers[i],
                cb_admin_payment_tp_digit_roller_changed, LV_EVENT_VALUE_CHANGED, NULL);
        }

        g_admin_payment_tp_lbl_colon = lv_label_create(g_admin_payment_tp_wrap);
        lv_label_set_text(g_admin_payment_tp_lbl_colon, ":");
        lv_obj_set_style_text_color(g_admin_payment_tp_lbl_colon, lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_payment_tp_lbl_colon, s_font_sc_70);
        lv_obj_update_layout(g_admin_payment_tp_lbl_colon);
        lv_obj_set_pos(g_admin_payment_tp_lbl_colon,
            tiles_x0 + (tile_w + gap) * 2 + (colon_w - lv_obj_get_width(g_admin_payment_tp_lbl_colon)) / 2 + (-2),
            row_y + (tile_h - lv_obj_get_height(g_admin_payment_tp_lbl_colon)) / 2 + (-10));
    }

    {
        const lv_coord_t btn_w = 110;
        const lv_coord_t btn_h = 50;
        const lv_coord_t btn_y = -40;
        g_admin_payment_tp_btn_ok = make_orange_fill_btn(g_admin_payment_tp_wrap,
            ui_translation(STR_BTN_CONFIRM), btn_w, btn_h);
        lv_obj_align(g_admin_payment_tp_btn_ok, LV_ALIGN_RIGHT_MID, -300, -50 + btn_y);
        ui_set_obj_font(lv_obj_get_child(g_admin_payment_tp_btn_ok, 0), s_font_sc_30);
        orange_btn_bind_i18n(g_admin_payment_tp_btn_ok, STR_BTN_CONFIRM);
        lv_obj_add_event_cb(g_admin_payment_tp_btn_ok, cb_admin_payment_tp_confirm, LV_EVENT_CLICKED, NULL);

        g_admin_payment_tp_btn_cancel = make_orange_outline_btn(g_admin_payment_tp_wrap,
            ui_translation(STR_BTN_CANCEL), btn_w, btn_h);
        lv_obj_align(g_admin_payment_tp_btn_cancel, LV_ALIGN_RIGHT_MID, -300, 50 + btn_y);
        ui_set_obj_font(lv_obj_get_child(g_admin_payment_tp_btn_cancel, 0), s_font_sc_30);
        orange_btn_bind_i18n(g_admin_payment_tp_btn_cancel, STR_BTN_CANCEL);
        lv_obj_add_event_cb(g_admin_payment_tp_btn_cancel, cb_admin_payment_tp_cancel, LV_EVENT_CLICKED, NULL);
    }

    admin_payment_set_page(ADMIN_PAYMENT_PAGE_LIST);
    admin_payment_sync_list_ui();
    admin_payment_sync_list_ui();
    admin_payment_sync_method_ui();

    /* 数据设置子面板（标题页：title_box + set_box + 7 行滚动开关） */
    g_admin_panel_data = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_data, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_data, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_data, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_data, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_data, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_data, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_data, LV_OBJ_FLAG_HIDDEN);

    g_admin_img_data_title_box = lv_image_create(g_admin_panel_data);
    lv_image_set_src(g_admin_img_data_title_box, &title_box);
    lv_obj_align(g_admin_img_data_title_box, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_lbl_data_title = lv_label_create(g_admin_panel_data);
    ui_lang_bind_label(g_admin_lbl_data_title, STR_ADMIN_M2_DATA);
    lv_obj_set_style_text_color(g_admin_lbl_data_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_data_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_data_title, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_data_set_box_wrap = lv_obj_create(g_admin_panel_data);
    lv_obj_set_size(g_admin_data_set_box_wrap, 1117, 409);
    lv_obj_align(g_admin_data_set_box_wrap, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(g_admin_data_set_box_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_data_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_data_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_data_set_box_wrap, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * img_data_set_box = lv_image_create(g_admin_data_set_box_wrap);
    lv_image_set_src(img_data_set_box, &set_box);
    lv_obj_center(img_data_set_box);

    /* 数据设置页统一宽度（列表/策略行、视图、横线共用，改这个值可整体缩放） */
    const lv_coord_t data_row_w = 800;

    g_admin_data_list_view = lv_obj_create(g_admin_data_set_box_wrap);
    lv_obj_set_size(g_admin_data_list_view, data_row_w, 300);
    lv_obj_align(g_admin_data_list_view, LV_ALIGN_TOP_MID, 0, 70);
    lv_obj_set_style_bg_opa(g_admin_data_list_view, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_data_list_view, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_data_list_view, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(g_admin_data_list_view, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(g_admin_data_list_view, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_data_list_view, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(g_admin_data_list_view, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(g_admin_data_list_view, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_set_scroll_dir(g_admin_data_list_view, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(g_admin_data_list_view, LV_SCROLLBAR_MODE_OFF);

    g_admin_data_strategy_view = lv_obj_create(g_admin_data_set_box_wrap);
    lv_obj_set_size(g_admin_data_strategy_view, data_row_w, LV_SIZE_CONTENT);
    lv_obj_align(g_admin_data_strategy_view, LV_ALIGN_TOP_MID, 0, 70);
    lv_obj_set_style_bg_opa(g_admin_data_strategy_view, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_data_strategy_view, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_data_strategy_view, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(g_admin_data_strategy_view, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(g_admin_data_strategy_view, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_data_strategy_view, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(g_admin_data_strategy_view, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(g_admin_data_strategy_view, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_set_scroll_dir(g_admin_data_strategy_view, LV_DIR_NONE);
    lv_obj_set_scrollbar_mode(g_admin_data_strategy_view, LV_SCROLLBAR_MODE_OFF);

    for(int i = 0; i < 7; i++) {
        g_admin_data_row_upload[i] = lv_obj_create(g_admin_data_list_view);
        lv_obj_set_size(g_admin_data_row_upload[i], data_row_w, 60);
        lv_obj_set_style_bg_opa(g_admin_data_row_upload[i], LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(g_admin_data_row_upload[i], 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(g_admin_data_row_upload[i], 0, LV_PART_MAIN);
        lv_obj_set_style_layout(g_admin_data_row_upload[i], LV_LAYOUT_FLEX, LV_PART_MAIN);
        lv_obj_set_flex_flow(g_admin_data_row_upload[i], LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(g_admin_data_row_upload[i], LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(g_admin_data_row_upload[i], LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

        g_admin_lbl_data_upload[i] = lv_label_create(g_admin_data_row_upload[i]);
        lv_obj_set_style_text_color(g_admin_lbl_data_upload[i], lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_data_upload[i], s_font_sc_30);
        lv_label_set_long_mode(g_admin_lbl_data_upload[i], LV_LABEL_LONG_DOT);
        lv_obj_set_width(g_admin_lbl_data_upload[i], LV_SIZE_CONTENT);

        g_admin_cb_data_upload[i] = lv_switch_create(g_admin_data_row_upload[i]);
        admin_data_style_switch(g_admin_cb_data_upload[i]);
        lv_obj_add_event_cb(g_admin_cb_data_upload[i], cb_admin_data_upload_switch_changed, LV_EVENT_VALUE_CHANGED, NULL);

        /* 7 行之间用灰色横线隔开（宽度随行宽变化） */
        if(i < 6) {
            lv_obj_t * sep = lv_obj_create(g_admin_data_list_view);
            lv_obj_set_size(sep, data_row_w, 1);
            lv_obj_set_style_bg_color(sep, lv_color_hex(COL_DIM), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(sep, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_border_width(sep, 0, LV_PART_MAIN);
            lv_obj_clear_flag(sep, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        }
    }

    for(int i = 0; i < 5; i++) {
        g_admin_data_row_strategy[i] = lv_obj_create(g_admin_data_strategy_view);
        lv_obj_set_size(g_admin_data_row_strategy[i], data_row_w, 60);
        lv_obj_set_style_bg_opa(g_admin_data_row_strategy[i], LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(g_admin_data_row_strategy[i], 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(g_admin_data_row_strategy[i], 0, LV_PART_MAIN);
        lv_obj_set_style_layout(g_admin_data_row_strategy[i], LV_LAYOUT_FLEX, LV_PART_MAIN);
        lv_obj_set_flex_flow(g_admin_data_row_strategy[i], LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(g_admin_data_row_strategy[i], LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(g_admin_data_row_strategy[i], LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

        g_admin_lbl_data_strategy[i] = lv_label_create(g_admin_data_row_strategy[i]);
        lv_obj_set_style_text_color(g_admin_lbl_data_strategy[i], lv_color_hex(COL_TEXT), LV_PART_MAIN);
        ui_set_obj_font(g_admin_lbl_data_strategy[i], s_font_sc_30);
        lv_label_set_long_mode(g_admin_lbl_data_strategy[i], LV_LABEL_LONG_DOT);
        lv_obj_set_width(g_admin_lbl_data_strategy[i], LV_SIZE_CONTENT);

        g_admin_cb_data_strategy[i] = lv_switch_create(g_admin_data_row_strategy[i]);
        admin_data_style_switch(g_admin_cb_data_strategy[i]);
        lv_obj_add_event_cb(g_admin_cb_data_strategy[i], cb_admin_data_strategy_changed, LV_EVENT_VALUE_CHANGED, NULL);

        /* 5 行之间用灰色横线隔开 */
        if(i < 4) {
            lv_obj_t * sep = lv_obj_create(g_admin_data_strategy_view);
            lv_obj_set_size(sep, data_row_w, 1);
            lv_obj_set_style_bg_color(sep, lv_color_hex(COL_DIM), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(sep, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_border_width(sep, 0, LV_PART_MAIN);
            lv_obj_clear_flag(sep, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        }
    }

    lv_obj_add_flag(g_admin_data_strategy_view, LV_OBJ_FLAG_HIDDEN);

    admin_data_sync_upload_items_ui();
    admin_data_sync_strategy_ui();

    /* 网络设置子面板（全宽，标题居中 + 左右两个小按钮） */
    g_admin_panel_network = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_network, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_network, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_network, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_network, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_network, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_network, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_network, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_network_title = lv_label_create(g_admin_panel_network);
    ui_lang_bind_label(g_admin_lbl_network_title, STR_ADMIN_M2_NETWORK);
    lv_obj_set_style_text_color(g_admin_lbl_network_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_network_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_network_title, LV_ALIGN_TOP_MID, 0, 8);

    // 网络设置左右两个按钮
    {
        const lv_coord_t net_btn_w = 280;
        const lv_coord_t net_btn_h = 200;
        const lv_coord_t net_gap = 60;
        const lv_coord_t net_row_w = net_btn_w * 2 + net_gap;// 两个按钮的总宽度
        const lv_coord_t net_x0 = (lv_coord_t)((UI_FIXED_W - net_row_w) / 2);// 整体居中
        const lv_coord_t net_y = 150;// 距顶 Y 位置

        g_admin_btn_network_wifi = make_admin_menu_btn(g_admin_panel_network, ui_translation(STR_WIFI_SETTINGS), NULL);
        lv_obj_set_size(g_admin_btn_network_wifi, net_btn_w, net_btn_h);
        lv_obj_set_pos(g_admin_btn_network_wifi, net_x0, net_y);
        admin_menu_btn_bind_i18n(g_admin_btn_network_wifi, STR_WIFI_SETTINGS);
        lv_obj_add_event_cb(g_admin_btn_network_wifi, cb_admin_open_wifi_settings, LV_EVENT_CLICKED, NULL);

        g_admin_btn_network_4g = make_admin_menu_btn(g_admin_panel_network, ui_translation(STR_4G_SETTINGS), NULL);
        lv_obj_set_size(g_admin_btn_network_4g, net_btn_w, net_btn_h);
        lv_obj_set_pos(g_admin_btn_network_4g, net_x0 + net_btn_w + net_gap, net_y);
        admin_menu_btn_bind_i18n(g_admin_btn_network_4g, STR_4G_SETTINGS);
        lv_obj_add_event_cb(g_admin_btn_network_4g, cb_admin_open_4g_settings, LV_EVENT_CLICKED, NULL);
    }

    /* WIFI 设置子面板（与 4G 设置页同尺寸） */
    g_admin_panel_wifi = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_wifi, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_wifi, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_wifi, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_wifi, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_wifi, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_wifi, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_wifi, LV_OBJ_FLAG_HIDDEN);

    /* 标题 "WIFI设置" + title_box */
    lv_obj_t * img_wifi_title_box = lv_image_create(g_admin_panel_wifi);
    g_admin_img_wifi_title_box = img_wifi_title_box;
    lv_image_set_src(img_wifi_title_box, &title_box);
    lv_obj_align(img_wifi_title_box, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_lbl_wifi_title = lv_label_create(g_admin_panel_wifi);
    ui_lang_bind_label(g_admin_lbl_wifi_title, STR_WIFI_SETTINGS);
    lv_obj_set_style_text_color(g_admin_lbl_wifi_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_wifi_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_wifi_title, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_sw_wifi = lv_switch_create(g_admin_panel_wifi);
    admin_wifi_apply_switch_layout();
    lv_obj_add_event_cb(g_admin_sw_wifi, cb_admin_wifi_switch_changed, LV_EVENT_VALUE_CHANGED, NULL);

    /* set_box 背景 */
    lv_obj_t * wifi_set_box_wrap = lv_obj_create(g_admin_panel_wifi);
    g_admin_wifi_set_box_wrap = wifi_set_box_wrap;
    lv_obj_set_size(wifi_set_box_wrap, 1117, 409);
    lv_obj_align(wifi_set_box_wrap, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(wifi_set_box_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(wifi_set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(wifi_set_box_wrap, 0, LV_PART_MAIN);

    lv_obj_t * img_set_box_wifi = lv_image_create(wifi_set_box_wrap);
    lv_image_set_src(img_set_box_wifi, &set_box);
    lv_obj_center(img_set_box_wifi);

    g_admin_lbl_wifi_prompt = lv_label_create(wifi_set_box_wrap);
    ui_lang_bind_label(g_admin_lbl_wifi_prompt, STR_WIFI_PROMPT);
    lv_obj_set_style_text_color(g_admin_lbl_wifi_prompt, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_wifi_prompt, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_wifi_prompt, 900);
    lv_label_set_long_mode(g_admin_lbl_wifi_prompt, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(g_admin_lbl_wifi_prompt, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_center(g_admin_lbl_wifi_prompt);

    g_admin_lbl_wifi_status = lv_label_create(wifi_set_box_wrap);
    lv_label_set_text(g_admin_lbl_wifi_status, ui_translation(STR_WIFI_PROVISIONING));
    lv_obj_set_style_text_color(g_admin_lbl_wifi_status, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_wifi_status, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_wifi_status, 900);
    lv_label_set_long_mode(g_admin_lbl_wifi_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(g_admin_lbl_wifi_status, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(g_admin_lbl_wifi_status, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(g_admin_lbl_wifi_status, LV_OBJ_FLAG_HIDDEN);

    g_admin_wifi_done_center = lv_obj_create(g_admin_panel_wifi);
    lv_obj_set_size(g_admin_wifi_done_center, LV_PCT(100), LV_PCT(100));
    lv_obj_center(g_admin_wifi_done_center);
    lv_obj_set_style_bg_opa(g_admin_wifi_done_center, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_admin_wifi_done_center, lv_color_hex(COL_BG), LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_wifi_done_center, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_wifi_done_center, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_wifi_done_center, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_wifi_done_center, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(g_admin_wifi_done_center, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_admin_wifi_done_center, cb_admin_wifi_result_click, LV_EVENT_CLICKED, NULL);

    g_admin_img_wifi_done = lv_image_create(g_admin_wifi_done_center);
    lv_image_set_src(g_admin_img_wifi_done, &success);
    lv_obj_align(g_admin_img_wifi_done, LV_ALIGN_CENTER, 0, -80);
    lv_obj_add_flag(g_admin_img_wifi_done, LV_OBJ_FLAG_EVENT_BUBBLE);

    g_admin_lbl_wifi_done = lv_label_create(g_admin_wifi_done_center);
    ui_lang_bind_label(g_admin_lbl_wifi_done, STR_WIFI_SUCCESS);
    lv_obj_set_style_text_color(g_admin_lbl_wifi_done, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_wifi_done, s_font_sc_50);
    lv_obj_align(g_admin_lbl_wifi_done, LV_ALIGN_CENTER, 0, 60);
    lv_obj_add_flag(g_admin_lbl_wifi_done, LV_OBJ_FLAG_EVENT_BUBBLE);

    /* WIFI 开关移到最前，避免被遮挡 */
    if(g_admin_sw_wifi != NULL) {
        lv_obj_move_foreground(g_admin_sw_wifi);
    }

    /* 4G 设置子面板（与 ID 设置页同尺寸） */
    g_admin_panel_4g = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_4g, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_4g, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_4g, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_4g, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_4g, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_4g, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_4g, LV_OBJ_FLAG_HIDDEN);

    /* 标题 "4G设置" + title_box（与 ID 设置页位置相同） */
    lv_obj_t * img_4g_title_box = lv_image_create(g_admin_panel_4g);
    g_admin_img_4g_title_box = img_4g_title_box;
    lv_image_set_src(img_4g_title_box, &title_box);
    lv_obj_align(img_4g_title_box, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_lbl_4g_title = lv_label_create(g_admin_panel_4g);
    ui_lang_bind_label(g_admin_lbl_4g_title, STR_4G_SETTINGS);
    lv_obj_set_style_text_color(g_admin_lbl_4g_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_4g_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_4g_title, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_sw_4g = lv_switch_create(g_admin_panel_4g);
    admin_4g_apply_switch_layout();
    lv_obj_add_event_cb(g_admin_sw_4g, cb_admin_4g_switch_changed, LV_EVENT_VALUE_CHANGED, NULL);

    /* set_box 背景（与 ID 设置页位置相同） */
    lv_obj_t * set_box_wrap_4g = lv_obj_create(g_admin_panel_4g);
    g_admin_4g_set_box_wrap = set_box_wrap_4g;
    lv_obj_set_size(set_box_wrap_4g, 1117, 409);
    lv_obj_align(set_box_wrap_4g, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(set_box_wrap_4g, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(set_box_wrap_4g, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(set_box_wrap_4g, 0, LV_PART_MAIN);

    lv_obj_t * img_set_box_4g = lv_image_create(set_box_wrap_4g);
    lv_image_set_src(img_set_box_4g, &set_box);
    lv_obj_center(img_set_box_4g);

    g_admin_lbl_4g_prompt = lv_label_create(set_box_wrap_4g);
    ui_lang_bind_label(g_admin_lbl_4g_prompt, STR_4G_PROMPT);
    lv_obj_set_style_text_color(g_admin_lbl_4g_prompt, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_4g_prompt, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_4g_prompt, 900);
    lv_label_set_long_mode(g_admin_lbl_4g_prompt, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(g_admin_lbl_4g_prompt, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_center(g_admin_lbl_4g_prompt);

    g_admin_lbl_4g_status = lv_label_create(set_box_wrap_4g);
    lv_label_set_text(g_admin_lbl_4g_status, ui_translation(STR_4G_PROVISIONING));
    lv_obj_set_style_text_color(g_admin_lbl_4g_status, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_4g_status, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_4g_status, 900);
    lv_label_set_long_mode(g_admin_lbl_4g_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(g_admin_lbl_4g_status, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(g_admin_lbl_4g_status, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(g_admin_lbl_4g_status, LV_OBJ_FLAG_HIDDEN);

    g_admin_4g_done_center = lv_obj_create(g_admin_panel_4g);
    lv_obj_set_size(g_admin_4g_done_center, LV_PCT(100), LV_PCT(100));
    lv_obj_center(g_admin_4g_done_center);
    lv_obj_set_style_bg_opa(g_admin_4g_done_center, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_admin_4g_done_center, lv_color_hex(COL_BG), LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_4g_done_center, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_4g_done_center, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_4g_done_center, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_4g_done_center, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(g_admin_4g_done_center, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_admin_4g_done_center, cb_admin_4g_result_click, LV_EVENT_CLICKED, NULL);

    g_admin_img_4g_done = lv_image_create(g_admin_4g_done_center);
    lv_image_set_src(g_admin_img_4g_done, &success);
    lv_obj_align(g_admin_img_4g_done, LV_ALIGN_CENTER, 0, -80);
    lv_obj_add_flag(g_admin_img_4g_done, LV_OBJ_FLAG_EVENT_BUBBLE);

    g_admin_lbl_4g_done = lv_label_create(g_admin_4g_done_center);
    ui_lang_bind_label(g_admin_lbl_4g_done, STR_4G_SUCCESS);
    lv_obj_set_style_text_color(g_admin_lbl_4g_done, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_4g_done, s_font_sc_50);
    lv_obj_align(g_admin_lbl_4g_done, LV_ALIGN_CENTER, 0, 60);
    lv_obj_add_flag(g_admin_lbl_4g_done, LV_OBJ_FLAG_EVENT_BUBBLE);

    /* 4G 开关移到最前，避免被 set_box/成功页遮挡而无法点击 */
    if(g_admin_sw_4g != NULL) {
        lv_obj_move_foreground(g_admin_sw_4g);
    }

    /* 密码修改页1：title_box（同待机时间）+ 原密码/新密码两行；无 set_box */
    g_admin_panel_pwd_chg_old = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_pwd_chg_old, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_pwd_chg_old, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_pwd_chg_old, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_pwd_chg_old, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_pwd_chg_old, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_pwd_chg_old, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_pwd_chg_old, LV_OBJ_FLAG_HIDDEN);

    g_admin_img_pwd_chg_old_title_box = lv_image_create(g_admin_panel_pwd_chg_old);
    lv_image_set_src(g_admin_img_pwd_chg_old_title_box, &title_box);
    lv_obj_align(g_admin_img_pwd_chg_old_title_box, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_lbl_pwd_chg_old_title = lv_label_create(g_admin_panel_pwd_chg_old);
    ui_lang_bind_label(g_admin_lbl_pwd_chg_old_title, STR_ADMIN_M2_PASSWORD);
    lv_obj_set_style_text_color(g_admin_lbl_pwd_chg_old_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_pwd_chg_old_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_pwd_chg_old_title, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_pwd_chg_old_content = lv_obj_create(g_admin_panel_pwd_chg_old);
    lv_obj_set_size(g_admin_pwd_chg_old_content, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(g_admin_pwd_chg_old_content, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_pwd_chg_old_content, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_pwd_chg_old_content, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_pwd_chg_old_content, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_pwd_chg_old_content, LV_OBJ_FLAG_SCROLLABLE);

    /* 输入框尺寸参考图1：宽约 480、高约 72 的深灰圆角条 */
    const lv_coord_t pwd_chg_input_w = 480;
    const lv_coord_t pwd_chg_input_h = 72;
    const lv_coord_t pwd_chg_label_w = 240;
    const lv_coord_t pwd_chg_row_w = pwd_chg_label_w + 24 + pwd_chg_input_w;
    const lv_coord_t pwd_chg_row1_y = 140;
    const lv_coord_t pwd_chg_row2_y = 240;

    lv_obj_t * pwd_row_old = lv_obj_create(g_admin_pwd_chg_old_content);
    lv_obj_set_size(pwd_row_old, pwd_chg_row_w, pwd_chg_input_h);
    lv_obj_align(pwd_row_old, LV_ALIGN_TOP_MID, 0, pwd_chg_row1_y);
    lv_obj_set_style_bg_opa(pwd_row_old, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(pwd_row_old, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(pwd_row_old, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(pwd_row_old, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(pwd_row_old, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(pwd_row_old, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(pwd_row_old, 24, LV_PART_MAIN);
    lv_obj_remove_flag(pwd_row_old, LV_OBJ_FLAG_SCROLLABLE);

    g_admin_lbl_pwd_chg_old = lv_label_create(pwd_row_old);
    ui_lang_bind_label(g_admin_lbl_pwd_chg_old, STR_PWD_ENTER_OLD);
    lv_obj_set_style_text_color(g_admin_lbl_pwd_chg_old, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_pwd_chg_old, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_pwd_chg_old, pwd_chg_label_w);
    lv_obj_set_style_text_align(g_admin_lbl_pwd_chg_old, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);

    lv_obj_t * pwd_old_wrap = lv_obj_create(pwd_row_old);
    lv_obj_set_size(pwd_old_wrap, pwd_chg_input_w, pwd_chg_input_h);
    lv_obj_set_style_bg_color(pwd_old_wrap, lv_color_hex(0x2C2C2C), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(pwd_old_wrap, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(pwd_old_wrap, 12, LV_PART_MAIN);
    lv_obj_set_style_border_width(pwd_old_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(pwd_old_wrap, 0, LV_PART_MAIN);
    lv_obj_remove_flag(pwd_old_wrap, LV_OBJ_FLAG_SCROLLABLE);

    g_admin_ta_pwd_chg_old = lv_textarea_create(pwd_old_wrap);
    lv_obj_set_size(g_admin_ta_pwd_chg_old, pwd_chg_input_w - 40, 48);
    lv_obj_center(g_admin_ta_pwd_chg_old);
    lv_textarea_set_one_line(g_admin_ta_pwd_chg_old, true);
    lv_textarea_set_password_mode(g_admin_ta_pwd_chg_old, true);
    lv_textarea_set_max_length(g_admin_ta_pwd_chg_old, 6);
    lv_textarea_set_accepted_chars(g_admin_ta_pwd_chg_old, "0123456789");
    lv_obj_set_style_text_font(g_admin_ta_pwd_chg_old, s_font_sc_35, LV_PART_MAIN);
    lv_obj_set_style_text_color(g_admin_ta_pwd_chg_old, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_admin_ta_pwd_chg_old, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_ta_pwd_chg_old, 0, LV_PART_MAIN);
    lv_obj_set_style_text_align(g_admin_ta_pwd_chg_old, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_old, cb_admin_ta_ready, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_old, cb_admin_ta_kb_focus, LV_EVENT_ALL, NULL);

    lv_obj_t * pwd_row_new1 = lv_obj_create(g_admin_pwd_chg_old_content);
    lv_obj_set_size(pwd_row_new1, pwd_chg_row_w, pwd_chg_input_h);
    lv_obj_align(pwd_row_new1, LV_ALIGN_TOP_MID, 0, pwd_chg_row2_y);
    lv_obj_set_style_bg_opa(pwd_row_new1, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(pwd_row_new1, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(pwd_row_new1, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(pwd_row_new1, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(pwd_row_new1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(pwd_row_new1, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(pwd_row_new1, 24, LV_PART_MAIN);
    lv_obj_remove_flag(pwd_row_new1, LV_OBJ_FLAG_SCROLLABLE);

    g_admin_lbl_pwd_chg_new1 = lv_label_create(pwd_row_new1);
    ui_lang_bind_label(g_admin_lbl_pwd_chg_new1, STR_PWD_ENTER_NEW);
    lv_obj_set_style_text_color(g_admin_lbl_pwd_chg_new1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_pwd_chg_new1, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_pwd_chg_new1, pwd_chg_label_w);
    lv_obj_set_style_text_align(g_admin_lbl_pwd_chg_new1, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);

    lv_obj_t * pwd_new1_wrap = lv_obj_create(pwd_row_new1);
    lv_obj_set_size(pwd_new1_wrap, pwd_chg_input_w, pwd_chg_input_h);
    lv_obj_set_style_bg_color(pwd_new1_wrap, lv_color_hex(0x2C2C2C), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(pwd_new1_wrap, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(pwd_new1_wrap, 12, LV_PART_MAIN);
    lv_obj_set_style_border_width(pwd_new1_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(pwd_new1_wrap, 0, LV_PART_MAIN);
    lv_obj_remove_flag(pwd_new1_wrap, LV_OBJ_FLAG_SCROLLABLE);

    g_admin_ta_pwd_chg_new1 = lv_textarea_create(pwd_new1_wrap);
    lv_obj_set_size(g_admin_ta_pwd_chg_new1, pwd_chg_input_w - 40, 48);
    lv_obj_center(g_admin_ta_pwd_chg_new1);
    lv_textarea_set_one_line(g_admin_ta_pwd_chg_new1, true);
    lv_textarea_set_password_mode(g_admin_ta_pwd_chg_new1, true);
    lv_textarea_set_max_length(g_admin_ta_pwd_chg_new1, 6);
    lv_textarea_set_accepted_chars(g_admin_ta_pwd_chg_new1, "0123456789");
    lv_obj_set_style_text_font(g_admin_ta_pwd_chg_new1, s_font_sc_35, LV_PART_MAIN);
    lv_obj_set_style_text_color(g_admin_ta_pwd_chg_new1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_admin_ta_pwd_chg_new1, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_ta_pwd_chg_new1, 0, LV_PART_MAIN);
    lv_obj_set_style_text_align(g_admin_ta_pwd_chg_new1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_new1, cb_admin_ta_ready, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_new1, cb_admin_ta_kb_focus, LV_EVENT_ALL, NULL);

    /* 密码修改页2：title_box + 再次输入新密码 */
    g_admin_panel_pwd_chg_new = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_pwd_chg_new, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_pwd_chg_new, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_pwd_chg_new, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_pwd_chg_new, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_pwd_chg_new, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_pwd_chg_new, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_pwd_chg_new, LV_OBJ_FLAG_HIDDEN);

    g_admin_img_pwd_chg_new_title_box = lv_image_create(g_admin_panel_pwd_chg_new);
    lv_image_set_src(g_admin_img_pwd_chg_new_title_box, &title_box);
    lv_obj_align(g_admin_img_pwd_chg_new_title_box, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_lbl_pwd_chg_new_title = lv_label_create(g_admin_panel_pwd_chg_new);
    ui_lang_bind_label(g_admin_lbl_pwd_chg_new_title, STR_ADMIN_M2_PASSWORD);
    lv_obj_set_style_text_color(g_admin_lbl_pwd_chg_new_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_pwd_chg_new_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_pwd_chg_new_title, LV_ALIGN_TOP_MID, 0, 25);

    g_admin_pwd_chg_new_content = lv_obj_create(g_admin_panel_pwd_chg_new);
    lv_obj_set_size(g_admin_pwd_chg_new_content, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(g_admin_pwd_chg_new_content, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_pwd_chg_new_content, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_pwd_chg_new_content, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_pwd_chg_new_content, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_remove_flag(g_admin_pwd_chg_new_content, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * pwd_row_new2 = lv_obj_create(g_admin_pwd_chg_new_content);
    lv_obj_set_size(pwd_row_new2, pwd_chg_row_w + 60, pwd_chg_input_h);
    lv_obj_align(pwd_row_new2, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_bg_opa(pwd_row_new2, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(pwd_row_new2, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(pwd_row_new2, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(pwd_row_new2, LV_LAYOUT_FLEX, LV_PART_MAIN);
    lv_obj_set_flex_flow(pwd_row_new2, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(pwd_row_new2, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(pwd_row_new2, 24, LV_PART_MAIN);
    lv_obj_remove_flag(pwd_row_new2, LV_OBJ_FLAG_SCROLLABLE);

    g_admin_lbl_pwd_chg_new2 = lv_label_create(pwd_row_new2);
    ui_lang_bind_label(g_admin_lbl_pwd_chg_new2, STR_PWD_ENTER_NEW_AGAIN);
    lv_obj_set_style_text_color(g_admin_lbl_pwd_chg_new2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_pwd_chg_new2, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_pwd_chg_new2, pwd_chg_label_w + 60);
    lv_obj_set_style_text_align(g_admin_lbl_pwd_chg_new2, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);

    lv_obj_t * pwd_new2_wrap = lv_obj_create(pwd_row_new2);
    lv_obj_set_size(pwd_new2_wrap, pwd_chg_input_w, pwd_chg_input_h);
    lv_obj_set_style_bg_color(pwd_new2_wrap, lv_color_hex(0x2C2C2C), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(pwd_new2_wrap, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(pwd_new2_wrap, 12, LV_PART_MAIN);
    lv_obj_set_style_border_width(pwd_new2_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(pwd_new2_wrap, 0, LV_PART_MAIN);
    lv_obj_remove_flag(pwd_new2_wrap, LV_OBJ_FLAG_SCROLLABLE);

    g_admin_ta_pwd_chg_new2 = lv_textarea_create(pwd_new2_wrap);
    lv_obj_set_size(g_admin_ta_pwd_chg_new2, pwd_chg_input_w - 40, 48);
    lv_obj_center(g_admin_ta_pwd_chg_new2);
    lv_textarea_set_one_line(g_admin_ta_pwd_chg_new2, true);
    lv_textarea_set_password_mode(g_admin_ta_pwd_chg_new2, true);
    lv_textarea_set_max_length(g_admin_ta_pwd_chg_new2, 6);
    lv_textarea_set_accepted_chars(g_admin_ta_pwd_chg_new2, "0123456789");
    lv_obj_set_style_text_font(g_admin_ta_pwd_chg_new2, s_font_sc_35, LV_PART_MAIN);
    lv_obj_set_style_text_color(g_admin_ta_pwd_chg_new2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_admin_ta_pwd_chg_new2, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_ta_pwd_chg_new2, 0, LV_PART_MAIN);
    lv_obj_set_style_text_align(g_admin_ta_pwd_chg_new2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_new2, cb_admin_ta_ready, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_new2, cb_admin_ta_kb_focus, LV_EVENT_ALL, NULL);

    /* 结果页：与 WIFI 配网成功/失败同区域（body_h/body_y，露出顶部状态栏）与元素位置 */
    g_admin_pwd_chg_result = lv_obj_create(root);
    lv_obj_set_size(g_admin_pwd_chg_result, LV_PCT(100), body_h);
    lv_obj_align(g_admin_pwd_chg_result, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_pwd_chg_result, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_pwd_chg_result, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_pwd_chg_result, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_pwd_chg_result, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_pwd_chg_result, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(g_admin_pwd_chg_result, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_admin_pwd_chg_result, cb_admin_pwd_chg_result_click, LV_EVENT_CLICKED, NULL);

    g_admin_img_pwd_chg_result = lv_image_create(g_admin_pwd_chg_result);
    lv_image_set_src(g_admin_img_pwd_chg_result, &failure);
    lv_obj_align(g_admin_img_pwd_chg_result, LV_ALIGN_CENTER, 0, -80);

    g_admin_lbl_pwd_chg_result = lv_label_create(g_admin_pwd_chg_result);
    lv_label_set_text(g_admin_lbl_pwd_chg_result, "");
    lv_obj_set_style_text_color(g_admin_lbl_pwd_chg_result, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_pwd_chg_result, s_font_sc_50);
    lv_obj_align(g_admin_lbl_pwd_chg_result, LV_ALIGN_CENTER, 0, 60);

    /* 机器 ID 子面板 */
    g_admin_panel_machine_id = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_machine_id, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_machine_id, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_machine_id, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_machine_id, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_machine_id, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_machine_id, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_machine_id, LV_OBJ_FLAG_HIDDEN);

    /* 标题 "ID设置" + title_box 橙色点阵框 */
    lv_obj_t * img_title_box = lv_image_create(g_admin_panel_machine_id);
    lv_image_set_src(img_title_box, &title_box);
    lv_obj_align(img_title_box, LV_ALIGN_TOP_MID, 0, 25);

    lv_obj_t * lbl_mid_title = lv_label_create(g_admin_panel_machine_id);
    ui_lang_bind_label(lbl_mid_title, STR_ADMIN_M1_MACHINE_ID);
    lv_obj_set_style_text_color(lbl_mid_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_mid_title, s_font_sc_30);
    lv_obj_align(lbl_mid_title, LV_ALIGN_TOP_MID, 0, 25);

    /* set_box 背景 */
    lv_obj_t * set_box_wrap = lv_obj_create(g_admin_panel_machine_id);
    lv_obj_set_size(set_box_wrap, 1117, 409);
    lv_obj_align(set_box_wrap, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(set_box_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(set_box_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(set_box_wrap, 0, LV_PART_MAIN);

    lv_obj_t * img_set_box = lv_image_create(set_box_wrap);
    lv_image_set_src(img_set_box, &set_box);
    lv_obj_center(img_set_box);

    /* 左侧输入框（input_box 背景 + textarea） */
    lv_obj_t * input_wrap = lv_obj_create(set_box_wrap);
    lv_obj_set_size(input_wrap, 433, 149);
    lv_obj_align(input_wrap, LV_ALIGN_CENTER, -290, 0);
    lv_obj_set_style_bg_opa(input_wrap, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(input_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(input_wrap, 0, LV_PART_MAIN);

    lv_obj_t * img_input_bg = lv_image_create(input_wrap);
    lv_image_set_src(img_input_bg, &input_box);
    lv_obj_center(img_input_bg);

    g_admin_ta_machine_id = lv_textarea_create(input_wrap);
    lv_obj_set_size(g_admin_ta_machine_id, 350, 60);
    lv_obj_center(g_admin_ta_machine_id);
    lv_textarea_set_one_line(g_admin_ta_machine_id, true);
    lv_textarea_set_max_length(g_admin_ta_machine_id, 6);
    lv_textarea_set_accepted_chars(g_admin_ta_machine_id, "0123456789");
    lv_obj_set_style_text_font(g_admin_ta_machine_id, s_font_sc_40, LV_PART_MAIN);
    lv_obj_set_style_text_color(g_admin_ta_machine_id, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_admin_ta_machine_id, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_ta_machine_id, 0, LV_PART_MAIN);
    lv_obj_set_style_text_align(g_admin_ta_machine_id, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(g_admin_ta_machine_id, 8, LV_PART_MAIN);  //数字间距，越大越宽
    lv_obj_add_event_cb(g_admin_ta_machine_id, cb_admin_ta_ready, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(g_admin_ta_machine_id, cb_admin_ta_kb_focus, LV_EVENT_ALL, NULL);

    /* 右侧按钮 */
    const lv_coord_t mid_btn_w = 110;
    const lv_coord_t mid_btn_h = 50;
    g_admin_btn_machine_confirm = make_orange_fill_btn(set_box_wrap, ui_translation(STR_BTN_OK), mid_btn_w, mid_btn_h);
    lv_obj_align(g_admin_btn_machine_confirm, LV_ALIGN_CENTER, 400, -35);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_machine_confirm, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_machine_confirm, STR_BTN_OK);
    lv_obj_add_event_cb(g_admin_btn_machine_confirm, cb_admin_machine_confirm, LV_EVENT_CLICKED, NULL);

    g_admin_btn_machine_cancel = make_orange_outline_btn(set_box_wrap, ui_translation(STR_BTN_CANCEL), mid_btn_w, mid_btn_h);
    lv_obj_align(g_admin_btn_machine_cancel, LV_ALIGN_CENTER, 400, 35);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_machine_cancel, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_machine_cancel, STR_BTN_CANCEL);
    lv_obj_add_event_cb(g_admin_btn_machine_cancel, cb_admin_machine_cancel, LV_EVENT_CLICKED, NULL);

    /* 错误提示（set_box 下方） */
    g_admin_lbl_msg_machine_id = lv_label_create(g_admin_panel_machine_id);
    lv_obj_set_width(g_admin_lbl_msg_machine_id, LV_PCT(80));
    lv_obj_set_style_text_color(g_admin_lbl_msg_machine_id, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_align(g_admin_lbl_msg_machine_id, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_msg_machine_id, s_font_sc_30);
    lv_obj_align(g_admin_lbl_msg_machine_id, LV_ALIGN_BOTTOM_MID, 0, 20);
    lv_obj_add_flag(g_admin_lbl_msg_machine_id, LV_OBJ_FLAG_HIDDEN);

    /* 成功提示覆盖层：success 图片 + 文字；点击任意处返回 menu1 */
    g_admin_mid_success_overlay = lv_obj_create(g_admin_panel_machine_id);
    lv_obj_set_size(g_admin_mid_success_overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_center(g_admin_mid_success_overlay);
    lv_obj_set_style_bg_opa(g_admin_mid_success_overlay, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_admin_mid_success_overlay, lv_color_hex(COL_BG), LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_mid_success_overlay, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_mid_success_overlay, 0, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_mid_success_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(g_admin_mid_success_overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_admin_mid_success_overlay, cb_admin_mid_success_click, LV_EVENT_CLICKED, NULL);

    lv_obj_t * img_success = lv_image_create(g_admin_mid_success_overlay);
    lv_image_set_src(img_success, &success);
    lv_obj_align(img_success, LV_ALIGN_CENTER, 0, -80);
    lv_obj_add_flag(img_success, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t * lbl_success = lv_label_create(g_admin_mid_success_overlay);
    ui_lang_bind_label(lbl_success, STR_MACHINE_ID_SUCCESS);
    lv_obj_set_style_text_color(lbl_success, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_success, s_font_sc_50);
    lv_obj_align(lbl_success, LV_ALIGN_CENTER, 0, 60);
    lv_obj_add_flag(lbl_success, LV_OBJ_FLAG_EVENT_BUBBLE);

        program_admin_build_panel(root, body_y, body_h);

    /* 自定义数字键盘 */
    g_admin_kb = lv_obj_create(root);
    lv_obj_set_size(g_admin_kb, 1600, 150);
    lv_obj_align(g_admin_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(g_admin_kb, lv_color_hex(0x0c0c0c), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_admin_kb, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_kb, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_kb, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_kb, LV_LAYOUT_NONE, LV_PART_MAIN);
    {
        const lv_coord_t kbtn_w = 64, kbtn_h = 64, kbtn_w_icon = 108, kgap = 52;
        const lv_coord_t total_w = 10 * kbtn_w + 2 * kbtn_w_icon + 11 * kgap;
        const lv_coord_t start_x = (1600 - total_w) / 2;
        const lv_coord_t y_center = (150 - kbtn_h) / 2;
        lv_coord_t x = start_x;
        static const char * const digits[] = {"1","2","3","4","5","6","7","8","9","0"};
        for(int i = 0; i < 10; i++) {
            lv_obj_t * btn = lv_button_create(g_admin_kb);
            lv_obj_set_size(btn, kbtn_w, kbtn_h);
            lv_obj_set_pos(btn, x, y_center);
            lv_obj_set_style_radius(btn, 8, LV_PART_MAIN);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x545454), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN);
            lv_obj_add_event_cb(btn, cb_admin_kb_btn, LV_EVENT_CLICKED, (void *)digits[i]);
            lv_obj_t * lbl = lv_label_create(btn);
            lv_label_set_text(lbl, digits[i]);
            lv_obj_set_style_text_color(lbl, lv_color_hex(COL_TEXT), LV_PART_MAIN);
            if(s_font_admin_kb_ptr == NULL) admin_kb_font_init();
            if(s_font_admin_kb_ptr != NULL) lv_obj_set_style_text_font(lbl, s_font_admin_kb_ptr, LV_PART_MAIN);
            lv_obj_center(lbl);
            x += kbtn_w + kgap;
        }
        {
            lv_obj_t * btn = lv_button_create(g_admin_kb);
            lv_obj_set_size(btn, kbtn_w_icon, kbtn_h);
            lv_obj_set_pos(btn, x, y_center);
            lv_obj_set_style_radius(btn, 8, LV_PART_MAIN);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x545454), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN);
            lv_obj_add_event_cb(btn, cb_admin_kb_btn, LV_EVENT_CLICKED, "DEL");
            lv_obj_t * img = lv_image_create(btn);
            lv_image_set_src(img, &delete);
            lv_obj_center(img);
            x += kbtn_w_icon + kgap;
        }
        {
            lv_obj_t * btn = lv_button_create(g_admin_kb);
            lv_obj_set_size(btn, kbtn_w_icon, kbtn_h);
            lv_obj_set_pos(btn, x, y_center);
            lv_obj_set_style_radius(btn, 8, LV_PART_MAIN);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x545454), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN);
            lv_obj_add_event_cb(btn, cb_admin_kb_btn, LV_EVENT_CLICKED, "ENT");
            lv_obj_t * img = lv_image_create(btn);
            lv_image_set_src(img, &enter);
            lv_obj_center(img);
        }
    }
    lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);

    admin_panel_show(PASSWORD);
}









/* ========== 追加时间页：计算、会话与 UI ========== */

/* 追加时间页可选次数上限：程序设置 add_count；无追加能力时返回 0 */
static uint8_t add_time_max_count(int32_t idx)
{
	idx = wheel_mod_total(idx);
	const ui_program_admin_t * c = &g_prog_cfg[idx];
	if(!((c->cap & PROG_CAP_ADD_COUNT) && (c->cap & PROG_CAP_ADD_TIME))) {
		return 0u;
	}
	return c->add_count;
}

/* 总金额 = 程序金额 + 追加时间金额 × 选定次数 */
static int32_t add_time_calc_price(const ui_program_admin_t * c, uint8_t sel)
{
	int32_t price = c->price;
	if(price < 0) price = 0;
	if((c->cap & PROG_CAP_ADD_PRICE) && (c->cap & PROG_CAP_ADD_COUNT) && sel > 0u) {
		int32_t add_p = c->add_price;
		if(add_p < 0) add_p = 0;
		price += add_p * (int32_t)sel;
	}
	return price;
}

/* 总运行时间 = 初始烘干 + 选定次数×追加时间 + 冷却 */
static uint32_t add_time_calc_total_sec(const ui_program_admin_t * c, uint8_t sel)
{
	uint32_t total = 0u;

	if(c->cap & PROG_CAP_INIT_DRY) total += c->init_dry_min;
	if(c->cap & PROG_CAP_COOL) total += c->cool_min;
	if((c->cap & PROG_CAP_ADD_COUNT) && (c->cap & PROG_CAP_ADD_TIME) && sel > 0u) {
		total += (uint32_t)sel * (uint32_t)c->add_time_min;
	}
#if !PROG_ADMIN_DEMO_SEC
	total *= 60u;
#endif
	return total;
}

/* 按选定追加次数生成运行页分段（烘干段 + 冷却段） */
static void add_time_calc_run_stages(const ui_program_admin_t * c, uint8_t sel, ui_program_run_stages_t * out)
{
	if(out == NULL || c == NULL) return;

	uint32_t dry_sec = 0u;
	if(c->cap & PROG_CAP_INIT_DRY) dry_sec += c->init_dry_min;
	if((c->cap & PROG_CAP_ADD_COUNT) && (c->cap & PROG_CAP_ADD_TIME)) {
		dry_sec += (uint32_t)sel * (uint32_t)c->add_time_min;
	}
	uint32_t cool_sec = (c->cap & PROG_CAP_COOL) ? (uint32_t)c->cool_min : 0u;
#if !PROG_ADMIN_DEMO_SEC
	dry_sec *= 60u;
	cool_sec *= 60u;
#endif
	out->wash_sec = dry_sec;
	out->rinse_sec = 0u;
	out->spin_sec = cool_sec;
	if(dry_sec > 0u && cool_sec > 0u) {
		out->stages_bar_id = STR_RUN_STAGES;
	} else if(cool_sec > 0u) {
		out->stages_bar_id = STR_RUN_STAGES_SPIN_ONLY;
	} else {
		out->stages_bar_id = STR_RUN_STAGES_RINSE_SPIN;
	}
}

/* 确定后锁定会话：供支付页金额、param_change[2]、运行倒计时使用 */
static void add_time_session_commit(uint8_t sel)
{
	const ui_program_admin_t * c = &g_prog_cfg[wheel_mod_total(g_wheel_sel)];

	g_session_add_count = sel;
	g_session_total_price = add_time_calc_price(c, sel);
	g_session_total_sec = add_time_calc_total_sec(c, sel);
	add_time_calc_run_stages(c, sel, &g_session_run_stages);
	g_session_active = true;
}

/* 清除会话（追加页返回/取消、运行结束） */
static void add_time_session_clear(void)
{
	g_session_active = false;
	g_session_add_count = 0u;
	g_session_total_price = 0;
	g_session_total_sec = 0u;
	lv_memzero(&g_session_run_stages, sizeof(g_session_run_stages));
}

/* 追加次数步进，钳位 0..上限；仅由上下按钮触发 */
static void add_time_sel_step(int8_t delta)
{
	const uint8_t max_c = add_time_max_count(g_wheel_sel);
	int16_t next = (int16_t)g_add_time_sel + (int16_t)delta;

	if(next < 0) next = 0;
	if(next > (int16_t)max_c) next = (int16_t)max_c;
	g_add_time_sel = (uint8_t)next;
	add_time_page_sync_labels();
}

/* 刷新追加时间页：程序名、金额、时间、次数及上下按钮可用态 */
static void add_time_page_sync_labels(void)
{
	int32_t idx = wheel_mod_total(g_wheel_sel);
	const ui_program_admin_t * c = &g_prog_cfg[idx];
	char buf[16];
	const uint8_t max_c = add_time_max_count(idx);
	const bool can_step = (max_c > 0u);

	if(g_add_time_lbl_prog_name != NULL) {
		lv_label_set_text(g_add_time_lbl_prog_name, ui_program_name_get(idx));
	}
	if(g_add_time_lbl_total_price != NULL) {
		program_format_price_home(add_time_calc_price(c, g_add_time_sel), buf, sizeof(buf));
		lv_label_set_text(g_add_time_lbl_total_price, buf);
	}
	if(g_add_time_lbl_total_time != NULL) {
		program_format_time_label(add_time_calc_total_sec(c, g_add_time_sel), buf, sizeof(buf));
		lv_label_set_text(g_add_time_lbl_total_time, buf);
	}
	if(g_add_time_lbl_count_val != NULL) {
		lv_snprintf(buf, sizeof(buf), "%u", (unsigned)g_add_time_sel);
		lv_label_set_text(g_add_time_lbl_count_val, buf);
	}
	if(g_add_time_btn_up != NULL) {
		if(can_step) {
			lv_obj_remove_state(g_add_time_btn_up, LV_STATE_DISABLED);
		} else {
			lv_obj_add_state(g_add_time_btn_up, LV_STATE_DISABLED);
		}
	}
	if(g_add_time_btn_down != NULL) {
		if(can_step) {
			lv_obj_remove_state(g_add_time_btn_down, LV_STATE_DISABLED);
		} else {
			lv_obj_add_state(g_add_time_btn_down, LV_STATE_DISABLED);
		}
	}
}

/* 重建追加时间页编码器焦点顺序（所有按钮均带标准焦点框） */
static void add_time_encoder_group_build(void)
{
	if(g_group_add_time == NULL) return;

	lv_group_remove_all_objs(g_group_add_time);
	if(g_add_time_btn_back != NULL) ui_encoder_group_add(g_group_add_time, g_add_time_btn_back);
	if(g_add_time_btn_runpause != NULL) ui_encoder_group_add(g_group_add_time, g_add_time_btn_runpause);
	if(g_add_time_btn_power != NULL) ui_encoder_group_add(g_group_add_time, g_add_time_btn_power);
	if(g_add_time_btn_up != NULL) ui_encoder_group_add(g_group_add_time, g_add_time_btn_up);
	if(g_add_time_btn_down != NULL) ui_encoder_group_add(g_group_add_time, g_add_time_btn_down);
	lv_group_set_wrap(g_group_add_time, false);
}

static void cb_add_time_back(lv_event_t * e)
{
	if(g_ui_child_lock) return;
	if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
	add_time_session_clear();
	ui_screen_load(g_scr_home);
}

static void cb_add_time_btn_up(lv_event_t * e)
{
	if(g_ui_child_lock) return;
	if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
	add_time_sel_step(+1);
}

static void cb_add_time_btn_down(lv_event_t * e)
{
	if(g_ui_child_lock) return;
	if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
	add_time_sel_step(-1);
}

static void cb_add_time_runpause(lv_event_t * e)
{
	if(g_ui_child_lock) return;
	lv_event_code_t code = lv_event_get_code(e);
	if(code != LV_EVENT_CLICKED && code != LV_EVENT_LONG_PRESSED) return;
	add_time_session_commit(g_add_time_sel);
	SETFLAG(FSM_FLAG_NEED_PAYMENT);
	ui_scr_load_async();
}

static void cb_add_time_power_stub(lv_event_t * e)
{
	(void)e;
}

/* 追加时间页：白底值框（与程序设置外框一致） */
static lv_obj_t * add_time_make_value_box(lv_obj_t * parent, lv_coord_t w, lv_coord_t h)
{
	lv_obj_t * box = lv_obj_create(parent);
	lv_obj_set_size(box, w, h);
	lv_obj_set_style_bg_color(box, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
	lv_obj_set_style_bg_opa(box, LV_OPA_COVER, LV_PART_MAIN);
	lv_obj_set_style_radius(box, 6, LV_PART_MAIN);
	lv_obj_set_style_border_width(box, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(box, 0, LV_PART_MAIN);
	lv_obj_remove_flag(box, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_flex_flow(box, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(box, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
	return box;
}

/* 构建追加时间页：顶栏（返回/启停/电源）+ 1600×400 中间栏 */
static void build_add_time(void)
{
	lv_obj_t * root = lv_obj_create(g_scr_add_time);
	lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
	lv_obj_set_pos(root, 0, 0);
	lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(root, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(root, 0, LV_PART_MAIN);
	lv_obj_set_style_radius(root, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(root, LV_LAYOUT_NONE, LV_PART_MAIN);

	lv_obj_t * top = create_top_bar(root, &g_lbl_clock_add_time, NULL, NULL);

	g_add_time_btn_back = make_top_back_btn(top, cb_add_time_back);

	g_add_time_btn_runpause = add_top_text_btn(top, "启停", 100);
	lv_obj_add_event_cb(g_add_time_btn_runpause, cb_add_time_runpause, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(g_add_time_btn_runpause, cb_add_time_runpause, LV_EVENT_LONG_PRESSED, NULL);

	g_add_time_btn_power = add_top_text_btn(top, "电源", 180);
	lv_obj_add_event_cb(g_add_time_btn_power, cb_add_time_power_stub, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(g_add_time_btn_power, cb_add_time_power_stub, LV_EVENT_LONG_PRESSED, NULL);

    /* 中间栏：程序名、金额、时间、次数 */
	g_add_time_mid = lv_obj_create(root);
	lv_obj_set_size(g_add_time_mid, 1600, 400);
	lv_obj_align(g_add_time_mid, LV_ALIGN_TOP_MID, 0, 60);
	lv_obj_set_style_bg_opa(g_add_time_mid, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(g_add_time_mid, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(g_add_time_mid, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(g_add_time_mid, LV_LAYOUT_NONE, LV_PART_MAIN);

    //追加时间页面标题
	g_add_time_lbl_title = lv_label_create(g_add_time_mid);
	ui_lang_bind_label(g_add_time_lbl_title, STR_ADD_TIME_TITLE);
	lv_obj_set_style_text_color(g_add_time_lbl_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(g_add_time_lbl_title, s_font_sc_30);
	lv_obj_set_width(g_add_time_lbl_title, 1500);
	lv_obj_set_style_text_align(g_add_time_lbl_title, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	lv_label_set_long_mode(g_add_time_lbl_title, LV_LABEL_LONG_WRAP);
	lv_obj_align(g_add_time_lbl_title, LV_ALIGN_TOP_MID, 0, 16);

    //对应程序名称
	g_add_time_lbl_prog_name = lv_label_create(g_add_time_mid);
	lv_obj_set_style_text_color(g_add_time_lbl_prog_name, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(g_add_time_lbl_prog_name, s_font_sc_50);
	lv_obj_align(g_add_time_lbl_prog_name, LV_ALIGN_TOP_MID, 0, 90);

	static const ui_str_id_t field_ids[3] = {
		STR_ADD_TIME_TOTAL_PRICE, STR_ADD_TIME_TOTAL_TIME, STR_PROG_FIELD_ADD_COUNT
	};
	const lv_coord_t field_w = 158;
	const lv_coord_t field_h = 60;
	const lv_coord_t field_gap = 40;
	const lv_coord_t row_w = field_w * 3 + field_gap * 2;
	const lv_coord_t row_x0 = (lv_coord_t)((1600 - row_w) / 2);
	const lv_coord_t field_y_lbl = 130+60;
	const lv_coord_t field_y_val = 168+60;

	for(int i = 0; i < 3; i++) {
		const lv_coord_t fx = row_x0 + i * (field_w + field_gap);
		lv_obj_t * lbl = lv_label_create(g_add_time_mid);
		ui_lang_bind_label(lbl, field_ids[i]);
		ui_set_obj_font(lbl, s_font_sc_30);
		lv_obj_set_style_text_color(lbl, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		lv_obj_set_width(lbl, field_w);
		lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
		lv_obj_set_pos(lbl, fx, field_y_lbl);

		lv_obj_t * box = add_time_make_value_box(g_add_time_mid, field_w, field_h);
		lv_obj_set_pos(box, fx, field_y_val);

		if(i == 0) {
			g_add_time_lbl_total_price = lv_label_create(box);
			ui_set_obj_font(g_add_time_lbl_total_price, s_font_sc_30);
			lv_obj_set_style_text_color(g_add_time_lbl_total_price, lv_color_hex(0x333333), LV_PART_MAIN);
			lv_obj_center(g_add_time_lbl_total_price);
		} else if(i == 1) {
			g_add_time_lbl_total_time = lv_label_create(box);
			ui_set_obj_font(g_add_time_lbl_total_time, s_font_sc_30);
			lv_obj_set_style_text_color(g_add_time_lbl_total_time, lv_color_hex(0x333333), LV_PART_MAIN);
			lv_obj_center(g_add_time_lbl_total_time);
		} else {
			g_add_time_lbl_count_val = lv_label_create(box);
			ui_set_obj_font(g_add_time_lbl_count_val, s_font_sc_30);
			lv_obj_set_style_text_color(g_add_time_lbl_count_val, lv_color_hex(0x333333), LV_PART_MAIN);
			lv_obj_center(g_add_time_lbl_count_val);
		}
	}

	const lv_coord_t step_btn_w = 60;
	const lv_coord_t step_btn_h = 44;
	const lv_coord_t count_col_x = row_x0 + 2 * (field_w + field_gap);
	const lv_coord_t step_y = field_y_val + field_h + 8;

	g_add_time_btn_up = make_orange_fill_btn(g_add_time_mid, ui_translation(STR_ADD_TIME_BTN_UP), step_btn_w, step_btn_h);
	lv_obj_set_pos(g_add_time_btn_up, count_col_x, step_y);
	ui_set_obj_font(lv_obj_get_child(g_add_time_btn_up, 0), s_font_sc_30);
	orange_btn_bind_i18n(g_add_time_btn_up, STR_ADD_TIME_BTN_UP);
	lv_obj_add_event_cb(g_add_time_btn_up, cb_add_time_btn_up, LV_EVENT_CLICKED, NULL);

	g_add_time_btn_down = make_orange_fill_btn(g_add_time_mid, ui_translation(STR_ADD_TIME_BTN_DOWN), step_btn_w, step_btn_h);
	lv_obj_set_pos(g_add_time_btn_down, count_col_x + field_w - step_btn_w, step_y);
	ui_set_obj_font(lv_obj_get_child(g_add_time_btn_down, 0), s_font_sc_30);
	orange_btn_bind_i18n(g_add_time_btn_down, STR_ADD_TIME_BTN_DOWN);
	lv_obj_add_event_cb(g_add_time_btn_down, cb_add_time_btn_down, LV_EVENT_CLICKED, NULL);

	g_add_time_sel = 0;
	add_time_page_sync_labels();
}













/* 构建支付页：二维码、金额、扫码提示（提示支持中/英） */
static void build_pay(void)
{
		lv_obj_t * root = lv_obj_create(g_scr_pay);        /* 支付页根容器 */
		lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
		lv_obj_set_pos(root, 0, 0);
		lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(root, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(root, 0, LV_PART_MAIN);
		lv_obj_set_style_radius(root, 0, LV_PART_MAIN);
		lv_obj_set_style_layout(root, LV_LAYOUT_NONE, LV_PART_MAIN);

		lv_obj_t * top = create_top_bar(root, &g_lbl_clock_pay, g_scr_home, &g_pay_btn_back);
		lv_obj_t * btn_runpause = add_top_text_btn(top, "启停", 100);
#if UI_DEBUG_PAY_SKIP_TO_DONE
		lv_obj_add_event_cb(btn_runpause, cb_pay_debug_skip_to_done, LV_EVENT_CLICKED, NULL);
#else
		lv_obj_add_event_cb(btn_runpause, cb_runpause, LV_EVENT_CLICKED, g_scr_home);
#endif
		lv_obj_add_event_cb(btn_runpause, cb_runpause_long, LV_EVENT_LONG_PRESSED, g_scr_home);
		lv_obj_t * btn_power = add_top_text_btn(top, "电源", 180);
		lv_obj_add_event_cb(btn_power, cb_power_long, LV_EVENT_LONG_PRESSED, NULL);

		lv_obj_t * center = lv_obj_create(root);           /* 纵向：二维码区 → 金额 */
		lv_obj_set_size(center, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
		lv_obj_align(center, LV_ALIGN_CENTER, 0, 0);
		lv_obj_set_style_bg_opa(center, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(center, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(center, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_row(center, 20, LV_PART_MAIN);
		lv_obj_set_flex_flow(center, LV_FLEX_FLOW_COLUMN);
		lv_obj_set_flex_align(center, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

		/* 单码区域：仅一种支付方式或兜底仅支付宝 */
		g_pay_single_col = lv_obj_create(center);
		lv_obj_set_size(g_pay_single_col, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
		lv_obj_set_style_bg_opa(g_pay_single_col, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(g_pay_single_col, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(g_pay_single_col, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_row(g_pay_single_col, 16, LV_PART_MAIN);
		lv_obj_set_flex_flow(g_pay_single_col, LV_FLEX_FLOW_COLUMN);
		lv_obj_set_flex_align(g_pay_single_col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

		g_pay_qr_img = lv_image_create(g_pay_single_col);
		lv_obj_set_size(g_pay_qr_img, 250, 250);
		lv_image_set_inner_align(g_pay_qr_img, LV_IMAGE_ALIGN_STRETCH);

		g_lbl_pay_price = lv_label_create(g_pay_single_col);
		lv_obj_set_style_text_color(g_lbl_pay_price, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(g_lbl_pay_price, s_font_sc_35);

		g_lbl_pay_hint = lv_label_create(g_pay_single_col);
		lv_obj_set_style_text_color(g_lbl_pay_hint, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		lv_label_set_long_mode(g_lbl_pay_hint, LV_LABEL_LONG_WRAP);
		lv_obj_set_width(g_lbl_pay_hint, 560);
		lv_obj_set_style_text_align(g_lbl_pay_hint, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
		ui_set_obj_font(g_lbl_pay_hint, s_font_sc_30);

		/* 双码区域：左支付宝、右微信；间距见 PAY_DUAL_COL_GAP */
		g_pay_dual_row = lv_obj_create(center);
		lv_obj_set_size(g_pay_dual_row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
		lv_obj_set_style_bg_opa(g_pay_dual_row, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(g_pay_dual_row, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(g_pay_dual_row, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_column(g_pay_dual_row, PAY_DUAL_COL_GAP, LV_PART_MAIN);
		lv_obj_set_flex_flow(g_pay_dual_row, LV_FLEX_FLOW_ROW);
		lv_obj_set_flex_align(g_pay_dual_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);

		lv_obj_t * alipay_col = lv_obj_create(g_pay_dual_row);
		lv_obj_set_size(alipay_col, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
		lv_obj_set_style_bg_opa(alipay_col, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(alipay_col, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(alipay_col, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_row(alipay_col, 12, LV_PART_MAIN);
		lv_obj_set_flex_flow(alipay_col, LV_FLEX_FLOW_COLUMN);
		lv_obj_set_flex_align(alipay_col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

		g_pay_qr_alipay = lv_image_create(alipay_col);
		lv_obj_set_size(g_pay_qr_alipay, 200, 200);
		lv_image_set_inner_align(g_pay_qr_alipay, LV_IMAGE_ALIGN_STRETCH);

		g_lbl_pay_price_alipay = lv_label_create(alipay_col);
		lv_obj_set_style_text_color(g_lbl_pay_price_alipay, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(g_lbl_pay_price_alipay, s_font_sc_35);

		g_lbl_pay_hint_alipay_l1 = lv_label_create(alipay_col);
		lv_obj_set_style_text_color(g_lbl_pay_hint_alipay_l1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		lv_obj_set_width(g_lbl_pay_hint_alipay_l1, PAY_DUAL_HINT_W);
		lv_obj_set_style_text_align(g_lbl_pay_hint_alipay_l1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
		ui_set_obj_font(g_lbl_pay_hint_alipay_l1, s_font_sc_30);
		ui_lang_bind_label(g_lbl_pay_hint_alipay_l1, STR_PAY_HINT_ALIPAY_1);

		g_lbl_pay_hint_alipay_l2 = lv_label_create(alipay_col);
		lv_obj_set_style_text_color(g_lbl_pay_hint_alipay_l2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		lv_obj_set_width(g_lbl_pay_hint_alipay_l2, PAY_DUAL_HINT_W);
		lv_obj_set_style_text_align(g_lbl_pay_hint_alipay_l2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
		ui_set_obj_font(g_lbl_pay_hint_alipay_l2, s_font_sc_30);
		ui_lang_bind_label(g_lbl_pay_hint_alipay_l2, STR_PAY_HINT_ALIPAY_2);

		lv_obj_t * wechat_col = lv_obj_create(g_pay_dual_row);
		lv_obj_set_size(wechat_col, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
		lv_obj_set_style_bg_opa(wechat_col, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(wechat_col, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(wechat_col, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_row(wechat_col, 12, LV_PART_MAIN);
		lv_obj_set_flex_flow(wechat_col, LV_FLEX_FLOW_COLUMN);
		lv_obj_set_flex_align(wechat_col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

		g_pay_qr_wechat = lv_image_create(wechat_col);
		lv_obj_set_size(g_pay_qr_wechat, 200, 200);
		lv_image_set_inner_align(g_pay_qr_wechat, LV_IMAGE_ALIGN_STRETCH);

		g_lbl_pay_price_wechat = lv_label_create(wechat_col);
		lv_obj_set_style_text_color(g_lbl_pay_price_wechat, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(g_lbl_pay_price_wechat, s_font_sc_35);

		g_lbl_pay_hint_wechat_l1 = lv_label_create(wechat_col);
		lv_obj_set_style_text_color(g_lbl_pay_hint_wechat_l1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		lv_obj_set_width(g_lbl_pay_hint_wechat_l1, PAY_DUAL_HINT_W);
		lv_obj_set_style_text_align(g_lbl_pay_hint_wechat_l1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
		ui_set_obj_font(g_lbl_pay_hint_wechat_l1, s_font_sc_30);
		ui_lang_bind_label(g_lbl_pay_hint_wechat_l1, STR_PAY_HINT_WECHAT_1);

		g_lbl_pay_hint_wechat_l2 = lv_label_create(wechat_col);
		lv_obj_set_style_text_color(g_lbl_pay_hint_wechat_l2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		lv_obj_set_width(g_lbl_pay_hint_wechat_l2, PAY_DUAL_HINT_W);
		lv_obj_set_style_text_align(g_lbl_pay_hint_wechat_l2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
		ui_set_obj_font(g_lbl_pay_hint_wechat_l2, s_font_sc_30);
		ui_lang_bind_label(g_lbl_pay_hint_wechat_l2, STR_PAY_HINT_WECHAT_2);

		pay_sync_pay_ui();
		pay_sync_price_label();
}







/* 构建支付完成页：图标 +「支付完成」/ Payment Done，2 秒后自动进运行页 */
static void build_pay_done(void)
{
		lv_obj_t * root = lv_obj_create(g_scr_pay_done);   /* 支付完成页根容器 */
		lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
		lv_obj_set_pos(root, 0, 0);
		lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(root, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(root, 0, LV_PART_MAIN);
		lv_obj_set_style_radius(root, 0, LV_PART_MAIN);
		lv_obj_set_style_layout(root, LV_LAYOUT_NONE, LV_PART_MAIN);

		lv_obj_t * top = create_top_bar(root, &g_lbl_clock_pay_done, g_scr_home, NULL);
		lv_obj_t * btn_runpause = add_top_text_btn(top, "启停", 100);
		lv_obj_add_event_cb(btn_runpause, cb_runpause, LV_EVENT_CLICKED, NULL);
		lv_obj_add_event_cb(btn_runpause, cb_runpause_long, LV_EVENT_LONG_PRESSED, NULL);
		lv_obj_t * btn_power = add_top_text_btn(top, "电源", 180);
		lv_obj_add_event_cb(btn_power, cb_power_long, LV_EVENT_LONG_PRESSED, NULL);

		lv_obj_t * center = lv_obj_create(root);
		lv_obj_set_size(center, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
		lv_obj_align(center, LV_ALIGN_CENTER, 0, 0);
		lv_obj_set_style_bg_opa(center, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(center, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(center, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_row(center, 16, LV_PART_MAIN);
		lv_obj_set_flex_flow(center, LV_FLEX_FLOW_COLUMN);
		lv_obj_set_flex_align(center, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

		LV_IMAGE_DECLARE(success);
		lv_obj_t * img_done = lv_image_create(center);      /* 与结束页共用 success 图标 */
		lv_image_set_src(img_done, &success);

		/* 主文字：中「支付完成」/ 英「Payment Done」（STR_PAY_DONE） */
		g_lbl_pay_done = lv_label_create(center);
		lv_obj_set_style_text_color(g_lbl_pay_done, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(g_lbl_pay_done, s_font_sc_50);
		ui_lang_bind_label(g_lbl_pay_done, STR_PAY_DONE);
}








/* 构建待机页：顶栏（返回/启停/电源+状态栏）+ 居中大字系统时间 */
static void build_off(void)
{
		lv_obj_t * row = lv_obj_create(g_scr_off);
		lv_obj_set_size(row, LV_PCT(100), LV_PCT(100));
		lv_obj_set_pos(row, 0, 0);
		lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(row, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(row, 0, LV_PART_MAIN);
		lv_obj_set_style_radius(row, 0, LV_PART_MAIN);
		lv_obj_set_style_layout(row, LV_LAYOUT_NONE, LV_PART_MAIN);
		lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
		lv_obj_add_event_cb(row, cb_off_wake, LV_EVENT_CLICKED, NULL);

		/* 顶栏：左上返回/启停/电源（点击均唤醒），右上 4G/WiFi/时间 */
		lv_obj_t * top = create_top_bar(row, &g_lbl_clock_off, NULL, NULL);

		lv_obj_t * btn_back = make_top_back_btn(top, cb_off_wake);

		lv_obj_t * btn_runpause = add_top_text_btn(top, "启停", 100);
		lv_obj_add_event_cb(btn_runpause, cb_off_wake, LV_EVENT_CLICKED, NULL);
		g_off_btn_power = add_top_text_btn(top, "电源", 180);
		lv_obj_add_event_cb(g_off_btn_power, cb_off_wake, LV_EVENT_CLICKED, NULL);
		lv_obj_add_event_cb(g_off_btn_power, cb_power_long, LV_EVENT_LONG_PRESSED, NULL);

		/* 屏幕居中：系统时间 HH:MM，字号与运行页倒计时相同（125） */
		g_lbl_clock_off_center = lv_label_create(row);
		lv_label_set_text(g_lbl_clock_off_center, "16:30:00");
		lv_obj_set_style_text_color(g_lbl_clock_off_center, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		lv_obj_set_style_text_align(g_lbl_clock_off_center, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
		ui_set_obj_font(g_lbl_clock_off_center, s_font_sc_125);
		lv_obj_align(g_lbl_clock_off_center, LV_ALIGN_CENTER, 0, 0);
		lv_obj_add_flag(g_lbl_clock_off_center, LV_OBJ_FLAG_CLICKABLE);
		lv_obj_add_event_cb(g_lbl_clock_off_center, cb_off_wake, LV_EVENT_CLICKED, NULL);
}



static lv_obj_t* state_to_scr(uint8_t state)
{
	lv_obj_t* scr = NULL;
	switch(state) {
		case FSM_OFF: {
			scr = g_scr_off;
			break;
		}
		case FSM_STANDBY: {
			/* 5.2.1：循环模式设定/中断/故障页由 ui_screen_load 管理，勿被 FSM 异步切回主页 */
			if(g_cycle_active && g_cycle_ui_state != CYCLE_UI_RUNNING) {
				scr = NULL;
				break;
			}
			if(!GETFLAG(FSM_FLAG_NEED_PAYMENT)) {
				scr = g_scr_home;
			} else if(GETFLAG(FSM_FLAG_NEED_PAYMENT) && !GETFLAG(FSM_FLAG_PAYMENT_SUCCESS)){
				scr = g_scr_pay;
				CLRFLAG(FSM_FLAG_NEED_PAYMENT);
			} else if(GETFLAG(FSM_FLAG_PAYMENT_SUCCESS)) {
				scr = g_scr_pay_done;
			}
			break;
		}
		case FSM_RUNNING: {
			scr = NULL;
			break;
		}
		case FSM_PAUSED: {
			scr = NULL;
			break;
		}
		default: {
			scr = NULL;
			break;
		}
	}
	return scr;
}



static void scr_load_async()
{
	lv_obj_t* scr = NULL;

	if(!need_scr_load) {
		return;
	}
	need_scr_load = false;

	scr = state_to_scr(fsm.state);

	if(scr != NULL) {
		if(scr == g_scr_pay) {
			g_pay_focus_back_on_enter = true;
		}
		ui_screen_load(scr);
	}
}



BaseType_t ui_scr_load_async()
{
	need_scr_load = true;
	return pdTRUE;
}







/* UI 入口：字体、屏幕、编码器 group、各 build_* 页面构建并加载主页 */
void ui_init(void)
{
		if(g_ui_group == NULL) {                           /* 默认编码器组仅创建一次 */
			g_ui_group = lv_group_create();
			lv_group_set_wrap(g_ui_group, false);          /* 焦点不首尾循环 */
		}
		lv_group_set_default(g_ui_group);                  /* 新控件默认加入该组 */
		{
			lv_indev_t * indev = NULL;
			while((indev = lv_indev_get_next(indev)) != NULL) { /* 遍历输入设备 */
				lv_indev_type_t type = lv_indev_get_type(indev);
				if(type == LV_INDEV_TYPE_ENCODER || type == LV_INDEV_TYPE_KEYPAD) {
					lv_indev_set_group(indev, g_ui_group); /* 编码器/键盘绑定默认组 */
				}
			}
		}

		ui_layout_init();                                  /* 轮播尺寸、间距等 */
		ui_apply_chinese_font();                           /* 主题字体 + s_font_sc_* */
		program_admin_init_factory();                      /* 表3.1 程序初值 → 主页/运行页参数 */
		create_screens();                                  /* 创建全部 lv_screen（含报警页） */
		ui_apply_indev_long_press_ms(CHILD_LOCK_LONG_PRESS_MS);

		g_group_home = lv_group_create();
		lv_group_set_wrap(g_group_home, false);

		build_off();                                       /* 待机页（无业务 label） */
		build_home();                                      /* 主页：含全部底部文字 label */
		build_running();                                   /* 运行页：程序名/状态/阶段 label */
		build_end();                                       /* 结束页：标题/提示 label */
		build_selfcheck();                                 /* 有水自检页 */
		build_cycle();                                     /* 循环程序（寿命试验）页 */
		build_alarm_overlay();                             /* lv_layer_top 报警弹层 */
		build_admin();                                     /* 管理员页 */
		build_add_time();                                  /* 追加时间页 */
		build_pay();                                       /* 支付页：金额/扫码提示 label */
		build_pay_done();                                  /* 支付完成页：标题 label */

		ui_idle_init();

		g_scr_before_off = NULL;                           /* 开机首屏待机，唤醒后进主页 */
		ui_screen_load(g_scr_off);
		lv_obj_update_layout(g_scr_off);
}





void task_lvgl(void *pvParameters)
{
	uint32_t inactive_time = 0;

	while (1) {
		uint32_t delay_ms = lv_timer_handler();
		scr_load_async();
		ui_fsm_poll_running_pause_sync();  /* 外部改 FSM 后同步运行页倒计时 */
		ui_alarm_poll();                   /* 报警弹层轮播与显隐 */

		if(delay_ms == LV_NO_TIMER_READY) {
			delay_ms = LV_DEF_REFR_PERIOD;
		}

		inactive_time = lv_display_get_inactive_time(NULL);
		if(inactive_time >= 300000 && fsm.state == FSM_STANDBY) {
			fsm_state_change(FSM_OFF);
		}

		vTaskDelay(pdMS_TO_TICKS(delay_ms));
	}
}






/* PC 仿真：main.c 主循环每帧调用（对应 task_lvgl 中的 FSM 切屏逻辑） */
void ui_tick(void)
{
	scr_load_async();
	ui_fsm_poll_running_pause_sync();
	ui_alarm_poll();                                     //报警弹层轮播与显隐

	if(lv_display_get_inactive_time(NULL) >= 300000 && fsm.state == FSM_STANDBY) {
		fsm_state_change(FSM_OFF);
	}
}
