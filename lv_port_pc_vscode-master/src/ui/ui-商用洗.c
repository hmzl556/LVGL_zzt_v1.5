
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

/* 1：PC 调试：支付页点启停 → 支付完成 → 2s 进运行页。
 * 0：正式发布：支付页点启停 → 无任何效果。*/
#define UI_DEBUG_PAY_SKIP_TO_DONE  1

/* 1：PC 仿真运行页水位频率扫频 + 温度初值。
 * 0：实机：取消仿真，由 ui_running_freq_x100_set / ui_running_live_temp_set 推送。*/
#define UI_RUNNING_LIVE_PC_SIM  1





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
 * 自投功能（自动投放）— 硬件联动接口（仅在本文件 ui.c 说明与实现）
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
 * enabled == true  ：自动投放开启（管理员页选「开启」或恢复出厂默认）
 * enabled == false ：自动投放关闭（管理员页选「关闭」）
 *
 * 【职责划分】
 *   - UI：维护显示与用户选择；变更时调用 ui_auto_dispense_set。
 *   - 硬件：实现 cb，在 cb 内写 Modbus 保持寄存器、控制投放泵/阀、写 Flash 等。
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
 * 【恢复出厂】admin 恢复默认 → ui_auto_dispense_set(true) → cb(true)。
 * 【只读】if(ui_auto_dispense_get()) { ... }
 * 【通信重连】ui_auto_dispense_sync_to_hw() 再推送当前状态，不改动 UI。
 * 【PC 仿真】未 register 时 get/set 仍有效，仅无硬件回调。
 * ============================================================================ */

typedef void (*ui_auto_dispense_changed_cb_t)(bool enabled);

static bool g_ui_auto_dispense_enabled = true;
static ui_auto_dispense_changed_cb_t s_auto_dispense_hw_cb;

static void admin_auto_dispense_sync_btn_ui(void);  //自投功能页：刷新开启/关闭按钮选中样式
static void admin_payment_sync_checkbox_ui(void);  //支付设置页：刷新支付方式复选框
static void admin_payment_sync_timeout_roller_ui(void);  //支付设置页：刷新支付超时 roller
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

/* 注册自投功能状态变化回调；保存后立即以当前 enabled 调用一次 cb */
void ui_auto_dispense_hw_register(ui_auto_dispense_changed_cb_t cb)
{
    s_auto_dispense_hw_cb = cb;
    if(cb != NULL) {
        cb(g_ui_auto_dispense_enabled);
    }
}

/* 读取自投功能是否开启（true=开启，false=关闭） */
bool ui_auto_dispense_get(void)
{
    return g_ui_auto_dispense_enabled;
}

/* 设置自投功能开关；与当前相同返回 false 且不回调；否则刷新管理员页按钮并通知硬件 */
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

/* 将当前自投开关状态再次推送给硬件（不修改 UI，用于通信重连等） */
void ui_auto_dispense_sync_to_hw(void)
{
    ui_auto_dispense_apply_hw(g_ui_auto_dispense_enabled);
}

/* ============================================================================
 * 臭氧功能 — 硬件联动接口（仅在本文件 ui.c 说明与实现）
 * ============================================================================
 *
 * 【对外符号】硬件模块在自家 .c 顶部自行添加 extern 声明即可链接，无需改 ui.h：
 *
 *   typedef void (*ui_ozone_changed_cb_t)(bool enabled);
 *
 *   extern void ui_ozone_hw_register(ui_ozone_changed_cb_t cb);
 *   extern bool ui_ozone_get(void);
 *   extern bool ui_ozone_set(bool enabled);
 *   extern void ui_ozone_sync_to_hw(void);
 *
 * enabled == true  ：臭氧功能开启（管理员页选「开启」或恢复出厂默认）
 * enabled == false ：臭氧功能关闭（管理员页选「关闭」）
 *
 * 【职责划分】
 *   - UI：维护显示与用户选择；变更时调用 ui_ozone_set。
 *   - 硬件：实现 cb，在 cb 内写 Modbus 保持寄存器、控制臭氧发生器/继电器、写 Flash 等。
 *     勿在 cb 内调用 LVGL API 或长时间阻塞；跨任务请用队列投递。
 *
 * 【推荐启动顺序（真机示例）】
 *
 *   static void on_ozone_changed(bool enabled)
 *   {
 *       holding_register[MB_HOLD_xxx] = enabled ? 1u : 0u;
 *       ozone_relay_set(enabled);
 *   }
 *
 *   void board_init(void)
 *   {
 *       ui_ozone_hw_register(on_ozone_changed);
 *       bool saved = nvm_read_ozone();
 *       ui_ozone_set(saved);
 *       ui_init();
 *   }
 *
 * 【用户点「开启/关闭」】UI 内部 → ui_ozone_set → 刷新按钮 → cb(enabled)。
 * 【恢复出厂】admin 恢复默认 → ui_ozone_set(true) → cb(true)。
 * 【只读】if(ui_ozone_get()) { ... }
 * 【通信重连】ui_ozone_sync_to_hw() 再推送当前状态，不改动 UI。
 * 【PC 仿真】未 register 时 get/set 仍有效，仅无硬件回调。
 * ============================================================================ */

typedef void (*ui_ozone_changed_cb_t)(bool enabled);

static bool g_ui_ozone_enabled = true;
static lv_obj_t * g_running_o3_img;           /* 前置声明：运行页 O3 图标 */
static ui_ozone_changed_cb_t s_ozone_hw_cb;

static void admin_ozone_sync_btn_ui(void);  //臭氧功能页：刷新开启/关闭按钮选中样式

/* 通知已注册的臭氧硬件回调（未注册则无操作） */
static void ui_ozone_apply_hw(bool enabled)
{
    if(s_ozone_hw_cb != NULL) {
        s_ozone_hw_cb(enabled);
    }
}

/* 注册臭氧功能状态变化回调；保存后立即以当前 enabled 调用一次 cb */
void ui_ozone_hw_register(ui_ozone_changed_cb_t cb)
{
    s_ozone_hw_cb = cb;
    if(cb != NULL) {
        cb(g_ui_ozone_enabled);
    }
}

/* 读取臭氧功能是否开启（true=开启，false=关闭） */
bool ui_ozone_get(void)
{
    return g_ui_ozone_enabled;
}

/* 设置臭氧功能开关；与当前相同返回 false 且不回调；否则刷新管理员页按钮并通知硬件 */
bool ui_ozone_set(bool enabled)
{
    if(enabled == g_ui_ozone_enabled) {
        return false;
    }
    g_ui_ozone_enabled = enabled;
    admin_ozone_sync_btn_ui();
    ui_ozone_apply_hw(enabled);
    /* 同步运行页 O3 图标可见性 */
    if(g_running_o3_img != NULL) {
        if(enabled) lv_obj_remove_flag(g_running_o3_img, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_running_o3_img, LV_OBJ_FLAG_HIDDEN);
    }
    return true;
}

/* 将当前臭氧开关状态再次推送给硬件（不修改 UI，用于通信重连等） */
void ui_ozone_sync_to_hw(void)
{
    ui_ozone_apply_hw(g_ui_ozone_enabled);
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

/* 支付超时 roller 选项见 STR_PAYMENT_TIMEOUT_ROLLER */

static const uint16_t g_payment_timeout_sec_tbl[] = {60, 120, 180, 240, 300};
#define PAYMENT_TIMEOUT_ROLLER_CNT          ((uint32_t)(sizeof(g_payment_timeout_sec_tbl) / sizeof(g_payment_timeout_sec_tbl[0])))
#define PAYMENT_TIMEOUT_ROLLER_DEFAULT_IDX  2u  /* 180 秒 */

static bool g_ui_payment_alipay_enabled = true;
static bool g_ui_payment_wechat_enabled = true;  /* 默认支付宝与微信均启用 */
static uint16_t g_ui_payment_timeout_sec = 180;

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
    admin_payment_sync_checkbox_ui();
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
    admin_payment_sync_checkbox_ui();
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
    uint32_t idx = 0;
    bool found = false;
    for(uint32_t i = 0; i < PAYMENT_TIMEOUT_ROLLER_CNT; i++) {
        if(g_payment_timeout_sec_tbl[i] == sec) {
            idx = i;
            found = true;
            break;
        }
    }
    if(!found) {
        sec = g_payment_timeout_sec_tbl[PAYMENT_TIMEOUT_ROLLER_DEFAULT_IDX];
    }
    if(sec == g_ui_payment_timeout_sec) {
        return false;
    }
    g_ui_payment_timeout_sec = sec;
    admin_payment_sync_timeout_roller_ui();
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
static ui_data_upload_strategy_t g_ui_data_upload_strategy = UI_DATA_STRATEGY_4G_ONLY;

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

/* 读取当前上传策略（五选一） */
ui_data_upload_strategy_t ui_data_upload_strategy_get(void)
{
    return g_ui_data_upload_strategy;
}

/* 设置上传策略；非法值按「仅 4G 上传」处理；与当前相同返回 false */
bool ui_data_upload_strategy_set(ui_data_upload_strategy_t strategy)
{
    if(strategy > UI_DATA_STRATEGY_FORBIDDEN) {
        strategy = UI_DATA_STRATEGY_4G_ONLY;
    }
    if(strategy == g_ui_data_upload_strategy) {
        return false;
    }
    g_ui_data_upload_strategy = strategy;
    admin_data_sync_strategy_ui();
    return true;
}

/* ============================================================================
 * 4G 开关 — 状态维护（管理员 4G 设置页）
 * ============================================================================ */

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
 * enabled == true  ：用户选择「运行过程中屏幕常亮」（管理员页开关 ON）
 * enabled == false ：用户选择关闭常亮（管理员页开关 OFF、上电默认、恢复出厂默认）
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
 * 【恢复出厂】admin 恢复默认 → ui_screen_run_always_on_set(false) → cb(false)。
 * 【只读】if(ui_screen_run_always_on_get()) { ... }
 * 【通信重连】ui_screen_run_always_on_sync_to_hw() 再推送当前状态，不改动 UI。
 * ============================================================================ */

typedef void (*ui_screen_run_always_on_changed_cb_t)(bool enabled);

static bool g_ui_screen_run_always_on = false;  /* 上电默认关闭 */
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
 * percent：0~100，与管理员页亮度滑动条一致；步进 10（0、10、…、100）；上电默认 100，恢复出厂默认 100。
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

#define UI_SCREEN_BRIGHTNESS_STEP  10  /* 亮度滑条步进值 */

/* 亮度百分比对齐到步进（0、10、…、100） */
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
 * percent：0~100，步进 10；上电默认 100，恢复出厂默认 100。
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

#define UI_SOUND_VOLUME_STEP  10  /* 音量滑条步进值 */

/* 音量百分比对齐到步进（0、10、…、100） */
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
 * percent：0~100，步进 10；上电默认 100，恢复出厂默认 100。
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

#define UI_TOUCH_SOUND_VOLUME_STEP  10  /* 触控声音滑条步进值 */

/* 触控声音百分比对齐到步进（0、10、…、100） */
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

#define TOTAL_PROGRAMS 5    //总程序数

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
    STR_ALARM_E15_TITLE, STR_ALARM_E15_LINE1, STR_ALARM_E15_LINE2,
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
    STR_ADMIN_M1_VENDOR_MAINT,
    STR_ADMIN_M1_SOUND,
    STR_ADMIN_M1_LANGUAGE,
    STR_VENDOR_SERIAL_HINT,
    STR_VENDOR_SELF_CHECK,
    STR_VENDOR_SELF_LEARN,
    STR_SELF_CHECK_TITLE,
    STR_SELF_CHECK_DONE,
    STR_CYCLE_TITLE,
    STR_CYCLE_RUN_COUNT_FMT,
    STR_ADMIN_M1_STANDBY,
    STR_ADMIN_M1_FACTORY_RESET,
    STR_ADMIN_M1_CONTACT,
    STR_ADMIN_M2_AUTO_DISPENSE,
    STR_ADMIN_M2_OZONE,
    STR_ADMIN_M2_FRESH_AIR,
    STR_ADMIN_M2_UPGRADE,
    STR_ADMIN_M2_NETWORK,
    STR_ADMIN_M2_DATA,
    STR_ADMIN_M2_PAYMENT,
    STR_ADMIN_M2_PASSWORD,
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
    STR_PWD_WRONG_REENTER,
    STR_PWD_MISMATCH,
    STR_PWD_CHANGE_OK,
    STR_MACHINE_ID_TITLE,
    STR_MACHINE_ID_CUR_NONE,
    STR_MACHINE_ID_CUR_FMT,
    STR_PROG_FIELD_PRICE,
    STR_PROG_FIELD_WASH,
    STR_PROG_FIELD_RINSE_DUR,
    STR_PROG_FIELD_SPIN_DUR,
    STR_PROG_FIELD_RINSE_CNT,
    STR_PROG_FIELD_SPIN_RPM,
    STR_PROG_FIELD_TEMP,
    STR_PROG_FIELD_WATER,
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
    STR_FACTORY_CONFIRM_Q,
    STR_FACTORY_CONFIRM_HINT,
    STR_FACTORY_RESTORING,
    STR_FACTORY_DONE,
    STR_CONTACT_HOTLINE,
    STR_CONTACT_SLOGAN,
    STR_AUTO_DISP_LINE1,
    STR_AUTO_DISP_LINE2,
    STR_OZONE_LINE1,
    STR_OZONE_LINE2,
    STR_FRESH_AIR_LINE1,
    STR_FRESH_AIR_LINE2,
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
    STR_4G_SETTINGS,
    STR_4G_PROMPT,
    STR_4G_PROVISIONING,
    STR_4G_SUCCESS,
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
        [STR_PROG_DAWU]      = "大物",           /* 程序0：轮播/运行页程序名 */
        [STR_PROG_DANTUO]    = "单脱水",
        [STR_PROG_BIAOZHUN]  = "标准洗",
        [STR_PROG_TONGZIJIE] = "筒自洁",
        [STR_PROG_KUAIXI]    = "快洗",
        [STR_LANG_INDICATOR] = "China",          /* 主页第4列：当前为中文界面时的指示文字 */
        [STR_PAY_HINT]       = "请扫描屏幕上二维码\n支付完成机器自动运行",
        [STR_PAY_HINT_ALIPAY] = "请扫描屏幕上支付宝支付二维码\n支付完成机器自动运行",
        [STR_PAY_HINT_WECHAT] = "请扫描屏幕上微信支付二维码\n支付完成机器自动运行",
        [STR_PAY_HINT_ALIPAY_1] = "请扫描屏幕上支付宝支付二维码",
        [STR_PAY_HINT_ALIPAY_2] = "支付完成机器自动运行",
        [STR_PAY_HINT_WECHAT_1] = "请扫描屏幕上微信支付二维码",
        [STR_PAY_HINT_WECHAT_2] = "支付完成机器自动运行",
        [STR_PAY_DONE]       = "支付完成",
        [STR_RUN_WASHING]    = "洗涤中...",
        [STR_RUN_RINSING]    = "漂洗中...",
        [STR_RUN_SPINNING]   = "脱水中...",
        [STR_RUN_STAGES]     = "洗涤        漂洗        脱水",
        [STR_RUN_STAGES_SPIN_ONLY]  = "脱水",
        [STR_RUN_STAGES_RINSE_SPIN] = "漂洗        脱水",
        [STR_END_TITLE]      = "洗涤完成",
        [STR_END_HINT]       = "请及时取衣",
        [STR_ALARM_SOFTENER_LOW] = "检测到柔顺剂不足，请及时添加",
        [STR_ALARM_DETERGENT_LOW] = "检测到洗涤剂不足，请及时添加",
        [STR_ALARM_E1_TITLE]  = "门锁异常",
        [STR_ALARM_E1_LINE1]  = "请检查门是否关好，关紧门锁",
        [STR_ALARM_E1_LINE2]  = "或检查门锁与主控板连线是否松动",
        [STR_ALARM_E2_TITLE]  = "排水异常",
        [STR_ALARM_E2_LINE1]  = "检查排水泵是否正常工作",
        [STR_ALARM_E2_LINE2]  = "清洗并拉直排水软管，清洁排水管",
        [STR_ALARM_E3_TITLE]  = "进水异常",
        [STR_ALARM_E3_LINE1]  = "检查水龙头，完全打开水龙头，将进水管摆正",
        [STR_ALARM_E3_LINE2]  = "检查进水管过滤网，检查进水管是否打开",
        [STR_ALARM_E4_TITLE]  = "警戒水位",
        [STR_ALARM_E4_LINE1]  = "关闭进水阀，排水泵开始排水",
        [STR_ALARM_E4_LINE2]  = "10s 后程序继续运行",
        [STR_ALARM_E5_TITLE]  = "电机异常",
        [STR_ALARM_E5_LINE1]  = "请检查电机与主控板连线是否正常",
        [STR_ALARM_E5_LINE2]  = "",
        [STR_ALARM_E6_TITLE]  = "显示屏通讯故障",
        [STR_ALARM_E6_LINE1]  = "请检查主控板与显示屏的连线",
        [STR_ALARM_E6_LINE2]  = "",
        [STR_ALARM_E7_TITLE]  = "加热器异常",
        [STR_ALARM_E7_LINE1]  = "请检查加热器是否已坏",
        [STR_ALARM_E7_LINE2]  = "或与主控板接线是否连接正常",
        [STR_ALARM_E8_TITLE]  = "水温传感器异常",
        [STR_ALARM_E8_LINE1]  = "请关机，检查水温传感器是否已坏",
        [STR_ALARM_E8_LINE2]  = "或与主控板接线是否连接正常",
        [STR_ALARM_E9_TITLE]  = "水位传感器异常",
        [STR_ALARM_E9_LINE1]  = "请关机，检查水位传感器是否已坏",
        [STR_ALARM_E9_LINE2]  = "或与主控板接线是否连接正常",
        [STR_ALARM_E10_TITLE] = "电机通讯故障",
        [STR_ALARM_E10_LINE1] = "请重新运行机器",
        [STR_ALARM_E10_LINE2] = "并检查电机驱动器与主控板的通讯连线是否正常",
        [STR_ALARM_E11_TITLE] = "驱动板通讯异常",
        [STR_ALARM_E11_LINE1] = "请重新运行机器并检查驱动板的连线",
        [STR_ALARM_E11_LINE2] = "",
        [STR_ALARM_E12_TITLE] = "电脑主控板通讯异常",
        [STR_ALARM_E12_LINE1] = "请重新运行机器并检查电源板的连线",
        [STR_ALARM_E12_LINE2] = "",
        [STR_ALARM_E13_TITLE] = "物联网模块配置异常",
        [STR_ALARM_E13_LINE1] = "请断电并重新进行网络配置",
        [STR_ALARM_E13_LINE2] = "",
        [STR_ALARM_E14_TITLE] = "支付通讯异常",
        [STR_ALARM_E14_LINE1] = "请检查网络，并重试支付",
        [STR_ALARM_E14_LINE2] = "",
        [STR_ALARM_E15_TITLE] = "支付超时",
        [STR_ALARM_E15_LINE1] = "请重新选程序并支付",
        [STR_ALARM_E15_LINE2] = "",
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
        [STR_ADMIN_M1_VENDOR_MAINT]  = "厂商维护",
        [STR_ADMIN_M1_SOUND]         = "声音控制",
        [STR_ADMIN_M1_LANGUAGE]      = "语言设置",
        [STR_VENDOR_SERIAL_HINT]      = "请输入特殊出厂序列号",
        [STR_VENDOR_SELF_CHECK]     = "1、自检程序",
        [STR_VENDOR_SELF_LEARN]     = "2、循环程序",
        [STR_SELF_CHECK_TITLE]      = "自检程序",
        [STR_SELF_CHECK_DONE]       = "自检完成",
        [STR_CYCLE_TITLE]           = "循环程序",
        [STR_CYCLE_RUN_COUNT_FMT]   = "已完成：%u 次",
        [STR_ADMIN_M1_STANDBY]       = "待机时间",
        [STR_ADMIN_M1_FACTORY_RESET] = "恢复默认",
        [STR_ADMIN_M1_CONTACT]       = "联系我们",
        [STR_ADMIN_M2_AUTO_DISPENSE] = "自投功能",
        [STR_ADMIN_M2_OZONE]         = "臭氧功能",
        [STR_ADMIN_M2_FRESH_AIR]     = "新风护理",
        [STR_ADMIN_M2_UPGRADE]       = "系统升级",
        [STR_ADMIN_M2_NETWORK]       = "网络设置",
        [STR_ADMIN_M2_DATA]          = "数据设置",
        [STR_ADMIN_M2_PAYMENT]       = "支付设置",
        [STR_ADMIN_M2_PASSWORD]      = "密码修改",
        [STR_BTN_CONFIRM]            = "确认",
        [STR_BTN_CANCEL]             = "取消",
        [STR_BTN_OK]                 = "确定",
        [STR_BTN_RESET]              = "重置",
        [STR_BTN_ON]                 = "开启",
        [STR_BTN_OFF]                = "关闭",
        [STR_BTN_QUERY]              = "查询",
        [STR_PWD_ENTER_ADMIN]        = "请输入管理员密码",
        [STR_PWD_WRONG_RETRY]        = "密码错误，请稍后再试！",
        [STR_PWD_ENTER_OLD]          = "请输入原密码",
        [STR_PWD_ENTER_NEW]          = "请输入新密码",
        [STR_PWD_WRONG_REENTER]      = "密码错误，请重新输入",
        [STR_PWD_MISMATCH]           = "两次输入不一致，请重新输入",
        [STR_PWD_CHANGE_OK]          = "密码修改成功",
        [STR_MACHINE_ID_TITLE]       = "机器 ID 设置",
        [STR_MACHINE_ID_CUR_NONE]    = "当前 ID：未配置",
        [STR_MACHINE_ID_CUR_FMT]     = "当前 ID：%06u",
        [STR_PROG_FIELD_PRICE]       = "程序金额",
        [STR_PROG_FIELD_WASH]        = "洗涤时长",
        [STR_PROG_FIELD_RINSE_DUR]   = "漂洗时长",
        [STR_PROG_FIELD_SPIN_DUR]    = "脱水时长",
        [STR_PROG_FIELD_RINSE_CNT]   = "漂洗次数",
        [STR_PROG_FIELD_SPIN_RPM]    = "脱水转速",
        [STR_PROG_FIELD_TEMP]        = "水温",
        [STR_PROG_FIELD_WATER]       = "水位",
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
        [STR_FACTORY_CONFIRM_Q]      = "是否需要恢复默认设置？",
        [STR_FACTORY_CONFIRM_HINT]   = "（所有设置都将恢复出厂设置）",
        [STR_FACTORY_RESTORING]      = "正在恢复出厂设置......",
        [STR_FACTORY_DONE]           = "所有设置已恢复出厂设置",
        [STR_CONTACT_HOTLINE]        = "24小时服务热线：400-999-999",
        [STR_CONTACT_SLOGAN]         = "小鸭专属热线将为您提供优质的服务体验！",
        [STR_AUTO_DISP_LINE1]        = "自动投放功能开启时，",
        [STR_AUTO_DISP_LINE2]        = "可智能调节洗涤剂与柔顺剂用量",
        [STR_OZONE_LINE1]            = "开启后，可实现高效杀菌、消毒，",
        [STR_OZONE_LINE2]            = "提升洗涤效果和卫生水平",
        [STR_FRESH_AIR_LINE1]        = "开启后，洗衣完成后内筒间歇性转动，",
        [STR_FRESH_AIR_LINE2]        = "有效促进湿气散发，保持衣物干爽",
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
        [STR_4G_SETTINGS]            = "4G设置",
        [STR_4G_PROMPT]              = "使用右上方的切换开关打开4G，连接到附近的网络",
        [STR_4G_PROVISIONING]        = "正在配网中......",
        [STR_4G_SUCCESS]             = "配网成功",
    },
    [UI_LANG_EN] = {
        [STR_PROG_DAWU]      = "Heavy",
        [STR_PROG_DANTUO]    = "Spin",
        [STR_PROG_BIAOZHUN]  = "Standard",
        [STR_PROG_TONGZIJIE] = "Drum Clean",
        [STR_PROG_KUAIXI]    = "Quick",
        [STR_LANG_INDICATOR] = "English",       /* 英文界面时第4列显示 English */
        [STR_PAY_HINT]       = "Scan the QR code on screen\nMachine starts after payment",
        [STR_PAY_HINT_ALIPAY] = "Scan the Alipay QR code on screen\nMachine starts after payment",
        [STR_PAY_HINT_WECHAT] = "Scan the WeChat Pay QR code on screen\nMachine starts after payment",
        [STR_PAY_HINT_ALIPAY_1] = "Scan the Alipay QR code on screen",
        [STR_PAY_HINT_ALIPAY_2] = "Machine starts after payment",
        [STR_PAY_HINT_WECHAT_1] = "Scan the WeChat Pay QR code on screen",
        [STR_PAY_HINT_WECHAT_2] = "Machine starts after payment",
        [STR_PAY_DONE]       = "Payment Done",
        [STR_RUN_WASHING]    = "Washing...",
        [STR_RUN_RINSING]    = "Rinsing...",
        [STR_RUN_SPINNING]   = "Spinning...",
        [STR_RUN_STAGES]     = "Wash        Rinse        Spin",
        [STR_RUN_STAGES_SPIN_ONLY]  = "Spin",
        [STR_RUN_STAGES_RINSE_SPIN] = "Rinse        Spin",
        [STR_END_TITLE]      = "Wash Complete",
        [STR_END_HINT]       = "Please take clothes promptly",
        [STR_ALARM_SOFTENER_LOW] = "Low softener detected, Please refill promptly.",
        [STR_ALARM_DETERGENT_LOW] = "Low detergent detected, Please refill promptly.",
        [STR_ALARM_E1_TITLE]  = "Door Lock Fault",
        [STR_ALARM_E1_LINE1]  = "Please check that the door is closed and locked tightly",
        [STR_ALARM_E1_LINE2]  = "Or check whether the door lock wiring to the main board is loose",
        [STR_ALARM_E2_TITLE]  = "Drainage Fault",
        [STR_ALARM_E2_LINE1]  = "Check whether the drain pump works properly",
        [STR_ALARM_E2_LINE2]  = "Clean and straighten the drain hose and drain pipe",
        [STR_ALARM_E3_TITLE]  = "Water Inlet Fault",
        [STR_ALARM_E3_LINE1]  = "Check the faucet, open it fully, and straighten the inlet hose",
        [STR_ALARM_E3_LINE2]  = "Check the inlet filter and whether the inlet valve is open",
        [STR_ALARM_E4_TITLE]  = "Warning Water Level",
        [STR_ALARM_E4_LINE1]  = "Close the inlet valve; the drain pump will start draining",
        [STR_ALARM_E4_LINE2]  = "The program will continue after 10 seconds",
        [STR_ALARM_E5_TITLE]  = "Motor Fault",
        [STR_ALARM_E5_LINE1]  = "Please check whether the motor wiring to the main board is normal",
        [STR_ALARM_E5_LINE2]  = "",
        [STR_ALARM_E6_TITLE]  = "Display Communication Fault",
        [STR_ALARM_E6_LINE1]  = "Please check the wiring between the main board and the display",
        [STR_ALARM_E6_LINE2]  = "",
        [STR_ALARM_E7_TITLE]  = "Heater Fault",
        [STR_ALARM_E7_LINE1]  = "Please check whether the heater is damaged",
        [STR_ALARM_E7_LINE2]  = "Or check whether the heater wiring to the main board is connected properly",
        [STR_ALARM_E8_TITLE]  = "Water Temperature Sensor Fault",
        [STR_ALARM_E8_LINE1]  = "Please power off and check whether the water temperature sensor is damaged",
        [STR_ALARM_E8_LINE2]  = "Or check whether the sensor wiring to the main board is connected properly",
        [STR_ALARM_E9_TITLE]  = "Water Level Sensor Fault",
        [STR_ALARM_E9_LINE1]  = "Please power off and check whether the water level sensor is damaged",
        [STR_ALARM_E9_LINE2]  = "Or check whether the sensor wiring to the main board is connected properly",
        [STR_ALARM_E10_TITLE] = "Motor Communication Fault",
        [STR_ALARM_E10_LINE1] = "Please restart the machine",
        [STR_ALARM_E10_LINE2] = "And check the communication wiring between the motor driver and the main board",
        [STR_ALARM_E11_TITLE] = "Driver Board Communication Fault",
        [STR_ALARM_E11_LINE1] = "Please restart the machine and check the driver board wiring",
        [STR_ALARM_E11_LINE2] = "",
        [STR_ALARM_E12_TITLE] = "Main Board Communication Fault",
        [STR_ALARM_E12_LINE1] = "Please restart the machine and check the power board wiring",
        [STR_ALARM_E12_LINE2] = "",
        [STR_ALARM_E13_TITLE] = "IoT Module Configuration Fault",
        [STR_ALARM_E13_LINE1] = "Please power off and reconfigure the network",
        [STR_ALARM_E13_LINE2] = "",
        [STR_ALARM_E14_TITLE] = "Payment Communication Fault",
        [STR_ALARM_E14_LINE1] = "Please check the network and retry payment",
        [STR_ALARM_E14_LINE2] = "",
        [STR_ALARM_E15_TITLE] = "Payment Timeout",
        [STR_ALARM_E15_LINE1] = "Please select a program again and pay",
        [STR_ALARM_E15_LINE2] = "",
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
        [STR_ADMIN_M1_VENDOR_MAINT]  = "Vendor Maint.",
        [STR_ADMIN_M1_SOUND]         = "Sound",
        [STR_ADMIN_M1_LANGUAGE]      = "Language",
        [STR_VENDOR_SERIAL_HINT]      = "Enter factory serial no.",
        [STR_VENDOR_SELF_CHECK]     = "1. Self-check Prog.",
        [STR_VENDOR_SELF_LEARN]     = "2. Cycle Prog.",
        [STR_SELF_CHECK_TITLE]      = "Self-check Prog.",
        [STR_SELF_CHECK_DONE]       = "Self Check Done",
        [STR_CYCLE_TITLE]           = "Cycle Prog.",
        [STR_CYCLE_RUN_COUNT_FMT]   = "Completed: %u",
        [STR_ADMIN_M1_STANDBY]       = "Standby",
        [STR_ADMIN_M1_FACTORY_RESET] = "Factory Reset",
        [STR_ADMIN_M1_CONTACT]       = "Contact",
        [STR_ADMIN_M2_AUTO_DISPENSE] = "Auto Dispense",
        [STR_ADMIN_M2_OZONE]         = "Ozone",
        [STR_ADMIN_M2_FRESH_AIR]     = "Fresh Air",
        [STR_ADMIN_M2_UPGRADE]       = "Upgrade",
        [STR_ADMIN_M2_NETWORK]       = "Network",
        [STR_ADMIN_M2_DATA]          = "Data",
        [STR_ADMIN_M2_PAYMENT]       = "Payment",
        [STR_ADMIN_M2_PASSWORD]      = "Password",
        [STR_BTN_CONFIRM]            = "Confirm",
        [STR_BTN_CANCEL]             = "Cancel",
        [STR_BTN_OK]                 = "OK",
        [STR_BTN_RESET]              = "Reset",
        [STR_BTN_ON]                 = "On",
        [STR_BTN_OFF]                = "Off",
        [STR_BTN_QUERY]              = "Query",
        [STR_PWD_ENTER_ADMIN]        = "Enter admin password",
        [STR_PWD_WRONG_RETRY]        = "Wrong password, try again later!",
        [STR_PWD_ENTER_OLD]          = "Enter current password",
        [STR_PWD_ENTER_NEW]          = "Enter new password",
        [STR_PWD_WRONG_REENTER]      = "Wrong password, please re-enter",
        [STR_PWD_MISMATCH]           = "Passwords do not match, please re-enter",
        [STR_PWD_CHANGE_OK]          = "Password changed successfully",
        [STR_MACHINE_ID_TITLE]       = "Machine ID Settings",
        [STR_MACHINE_ID_CUR_NONE]    = "Current ID: Not configured",
        [STR_MACHINE_ID_CUR_FMT]     = "Current ID: %06u",
        [STR_PROG_FIELD_PRICE]       = "Price",
        [STR_PROG_FIELD_WASH]        = "Wash Time",
        [STR_PROG_FIELD_RINSE_DUR]   = "Rinse Time",
        [STR_PROG_FIELD_SPIN_DUR]    = "Spin Time",
        [STR_PROG_FIELD_RINSE_CNT]   = "Rinse Cycles",
        [STR_PROG_FIELD_SPIN_RPM]    = "Spin Speed",
        [STR_PROG_FIELD_TEMP]        = "Temperature",
        [STR_PROG_FIELD_WATER]       = "Water Level",
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
        [STR_FACTORY_CONFIRM_Q]      = "Restore factory settings?",
        [STR_FACTORY_CONFIRM_HINT]   = "(All settings will be reset to factory defaults)",
        [STR_FACTORY_RESTORING]      = "Restoring factory settings......",
        [STR_FACTORY_DONE]           = "All settings restored to factory defaults",
        [STR_CONTACT_HOTLINE]        = "24-hour hotline: 400-999-999",
        [STR_CONTACT_SLOGAN]         = "Duckling hotline provides premium service!",
        [STR_AUTO_DISP_LINE1]        = "When auto dispense is enabled,",
        [STR_AUTO_DISP_LINE2]        = "detergent and softener are adjusted intelligently",
        [STR_OZONE_LINE1]            = "When enabled, provides efficient sterilization",
        [STR_OZONE_LINE2]            = "and improves wash quality and hygiene",
        [STR_FRESH_AIR_LINE1]        = "When enabled, drum rotates intermittently after wash",
        [STR_FRESH_AIR_LINE2]        = "to release moisture and keep clothes dry",
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
        [STR_4G_SETTINGS]            = "4G Settings",
        [STR_4G_PROMPT]              = "Use the switch at top-right to enable 4G and connect to nearby networks",
        [STR_4G_PROVISIONING]        = "Connecting......",
        [STR_4G_SUCCESS]             = "Connected successfully",
    },
};

static const ui_str_id_t g_mode_name_ids[TOTAL_PROGRAMS] = {
    STR_PROG_DAWU, STR_PROG_DANTUO, STR_PROG_BIAOZHUN, STR_PROG_TONGZIJIE, STR_PROG_KUAIXI
};

static void carousel_update_card_images(void);  //按当前选中程序刷新 5 张可见卡片图与名称标签
static void running_screen_sync_mode_name(void);  //运行页程序名标签与 g_wheel_sel 保持一致
static void running_status_sync_labels(void);  //按程序与剩余时间刷新运行页底部左右文案
static void alarm_content_update(void);  //按液位检测结果刷新报警页中部图与文字
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
static void cb_lang_toggle(lv_event_t * e);	//主页语言图标按钮点击：切换语言并全界面刷新文案
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

/* 主页语言图标按钮点击：切换语言并全界面刷新文案 */
static void cb_lang_toggle(lv_event_t * e)
{
    (void)e;                                           /* 未使用事件参数 */
    ui_lang_toggle();                                  /* 翻转中/英 */
    ui_lang_apply_all();                               /* 刷新所有可翻译文字 */
}

/* 按程序索引取显示名（大物/Heavy 等），idx 先取模到 0..4 */
static const char * ui_program_name_get(int32_t idx)
{
    idx = wheel_mod_total(idx);                        /* 程序索引环上取模 */
    return ui_translation(g_mode_name_ids[(unsigned)idx]); /* 程序 ID → 当前语言字符串 */
}

#define UI_FIXED_W          1600u     //屏幕宽度像素
#define UI_FIXED_H          600u      //屏幕高度像素

#define MODE_SCALE_MIN      180       //最小缩放 (180/256倍)
#define MODE_SCALE_MAX      256       //最大缩放 (256/256倍)

#define MODE_IDX_DEFAULT    2         //商用洗默认选中「标准洗」（居中槽位）
#define CAROUSEL_VISIBLE_SLOTS 5      //轮播区同时可见卡片数（其余程序在逻辑环上隐藏）
#define CAROUSEL_CENTER_SLOT   2      //居中选中槽位下标 0..4

#define COL_BG              0x000000  //背景颜色-黑
#define COL_TEXT            0xFFFFFF  //文本颜色-白
#define COL_ORANGE          0xFF8C42  //按钮颜色-橙
#define COL_DIM             0x888888  //非选中状态颜色-灰
#define COL_CHILD_LOCK_RED  0xD03030  //童锁激活：圆形按钮填充色
#define COL_CHILD_LOCK_WHITE 0xF5F5F5 //童锁未激活：白按钮填充色
#define CHILD_LOCK_LONG_PRESS_MS 3000u	//童锁长按 3 秒
#define HOME_CAROUSEL_DOUBLE_CLICK_MS 400u  /* 主页轮播编码器双击间隔上限；固定居中并回启停 */

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
static lv_group_t* g_group_off;
static lv_group_t* g_group_home;
static lv_group_t* g_group_running;
static lv_group_t* g_group_end;
static lv_group_t* g_group_pay;
static lv_group_t* g_group_pay_done;
static lv_group_t* g_group_alarm;
static lv_group_t* g_group_admin;
static lv_group_t* g_group_selfcheck;
static lv_group_t* g_group_cycle;

typedef enum {
    PASSWORD,			//密码页面
    MENU1,				//管理员设置页1（8 宫格）
    MENU2,				//管理员设置页2（8 宫格）
    MACHINE_ID,			//机器ID页面
    PROGRAM_SETTINGS,	//程序设置页面
    SCREEN_BRIGHTNESS,	//屏幕亮度设置页面
    VENDOR_SERIAL,		//厂商维护：出厂序列号
    VENDOR_MENU,		//厂商维护：功能选择
    DORMANCY_STANDBY,	//待机时间设置页面
    LANGUAGE_SETTINGS,	//语言设置页面
    FACTORY_RESET,		//恢复默认设置页面
    CONTACT_US,			//联系我们页面
    AUTO_DISPENSE,		//自投功能页面
    OZONE,				//臭氧功能页面
    FRESH_AIR_CARE,		//新风护理页面
    SYSTEM_UPGRADE,		//系统升级页面
    PAYMENT_SETTINGS,	//支付设置页面
    DATA_SETTINGS,		//数据设置页面
    NETWORK_SETTINGS,	//网络设置入口页
    WIFI_SETTINGS,		//WIFI 设置页面（占位）
    SETTINGS_4G,		//4G 设置页面（三步配网）
    SOUND_CONTROL,		//声音控制（保留面板，menu1 已改为厂商维护）
    PASSWORD_CHANGE_OLD,	//密码修改：校验原密码
    PASSWORD_CHANGE_NEW,	//密码修改：设置新密码
} admin_view_t;

#define MACHINE_MODEL           "XQG150-001"
#define VENDOR_SERIAL_CODE      "123456"
#define SELFCHECK_STEP_UI_MS    3000u
#define SELFCHECK_FLASH_MS      500u
#define SELFCHECK_BLINK_MS      500u
#define SELFCHECK_STATUS_ROW_Y  ((lv_coord_t)((UI_FIXED_H * 30) / 100))
#define SELFCHECK_STATUS_ROW_W  ((lv_coord_t)((UI_FIXED_W * 87) / 100))
#define SELFCHECK_LIVE_TEMP_DEFAULT_C  45
#define SELFCHECK_STEP_COUNT    5

typedef enum {
    SELFCHECK_UI_MODEL,
    SELFCHECK_UI_FLASH,
    SELFCHECK_UI_INIT,
    SELFCHECK_UI_S1,
    SELFCHECK_UI_S2,
    SELFCHECK_UI_S3,
    SELFCHECK_UI_S4,
    SELFCHECK_UI_S5,
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

#define CYCLE_FAULT_CODE_MAX  15u
#define CYCLE_FAULT_BLINK_MS  500u
#define CYCLE_PROG_FIELD_CNT  6u

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
    ADMIN_4G_PHASE_PROMPT,       /* 小页面一：说明 + 开关 */
    ADMIN_4G_PHASE_PROVISIONING, /* 小页面二：正在配网中 */
    ADMIN_4G_PHASE_DONE,         /* 小页面三：配网成功 */
} admin_4g_phase_t;

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
static lv_obj_t * g_admin_kb_ta;     /* 键盘当前绑定的 textarea */
static lv_obj_t * g_admin_ta_pwd;
static lv_obj_t * g_admin_lbl_serial_title;   /* "请输入特殊出厂序列号" label */
static lv_obj_t * g_admin_input_wrap;         /* input_box 背景+输入框容器 */
static lv_obj_t * g_admin_ta_machine_id;
static lv_obj_t * g_admin_lbl_msg_pwd;
static lv_obj_t * g_admin_lbl_msg_machine_id;
static lv_obj_t * g_admin_lbl_machine_id_cur;
static lv_obj_t * g_admin_btn_machine_confirm;
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
static lv_obj_t * g_selfcheck_lbl_init_rpm;
static lv_obj_t * g_selfcheck_lbl_init_temp;
static lv_obj_t * g_selfcheck_lbl_init_rinse;
static lv_obj_t * g_selfcheck_step_btns[SELFCHECK_STEP_COUNT];
static lv_obj_t * g_selfcheck_lbl_done_title;
static lv_obj_t * g_selfcheck_row_status;
static lv_obj_t * g_selfcheck_lbl_doorlock;
static lv_obj_t * g_selfcheck_lbl_temp_live;
static lv_obj_t * g_selfcheck_row_indicator;
static lv_obj_t * g_selfcheck_lbl_phase;
static selfcheck_ui_state_t g_selfcheck_ui_state = SELFCHECK_UI_MODEL;
static lv_timer_t * g_selfcheck_timer;
static lv_timer_t * g_selfcheck_blink_timer;
static lv_obj_t * g_selfcheck_blink_target;
static uint8_t g_selfcheck_flash_count;
static bool g_selfcheck_flash_on;
static uint8_t g_selfcheck_init_flash_count;
static bool g_selfcheck_init_flash_on;
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
static lv_timer_t * g_cycle_fault_blink_timer;
static cycle_fault_phase_t g_cycle_fault_phase;
static bool g_cycle_fault_on_running;
static bool g_cycle_active;
static cycle_ui_state_t g_cycle_ui_state = CYCLE_UI_SETUP;
static int32_t g_cycle_prog_sel;
static lv_obj_t * g_cycle_prog_btns[TOTAL_PROGRAMS];
static lv_obj_t * g_cycle_lbl_run_count;

static lv_obj_t * s_admin_group_prev_focus;     /* 编码器 focus_cb：检测 menu2 第1钮左转 */
/* 程序设置子页控件 */
static lv_obj_t * g_admin_panel_program;
static lv_obj_t * g_admin_panel_brightness;
static lv_obj_t * g_admin_panel_dormancy;
static lv_obj_t * g_admin_dormancy_roller;
static lv_obj_t * g_admin_lbl_dormancy_cur;
static lv_obj_t * g_admin_btn_dormancy_confirm;
/* 屏幕亮度子页控件 */
static lv_obj_t * g_admin_lbl_brightness_title;
static lv_obj_t * g_admin_lbl_brightness_line1;
static lv_obj_t * g_admin_lbl_brightness_line2;
static lv_obj_t * g_admin_sw_run_always_on;
static lv_obj_t * g_admin_brightness_bar;           /* 亮度渐变动条（仅显示，下层） */
static lv_obj_t * g_admin_slider_brightness;        /* 亮度交互滑条（透明，中层） */
static lv_obj_t * g_admin_brightness_slider_focus;  /* 编码器白色焦点框（上层，不拦截点击） */
/* 声音控制子页控件 */
static lv_obj_t * g_admin_panel_sound;
static lv_obj_t * g_admin_lbl_sound_title;
static lv_obj_t * g_admin_lbl_sound_line1;
static lv_obj_t * g_admin_lbl_sound_line2;
static lv_obj_t * g_admin_sw_touch_sound;
static lv_obj_t * g_admin_sw_voice_broadcast;
static lv_obj_t * g_admin_lbl_sound_vol_icon;
static lv_obj_t * g_admin_sound_volume_bar;
static lv_obj_t * g_admin_slider_sound_volume;
static lv_obj_t * g_admin_sound_volume_slider_focus;
static lv_obj_t * g_admin_lbl_touch_sound_icon;
static lv_obj_t * g_admin_touch_sound_volume_bar;
static lv_obj_t * g_admin_slider_touch_sound_volume;
static lv_obj_t * g_admin_touch_sound_volume_slider_focus;
/* 语言设置子页控件 */
static lv_obj_t * g_admin_panel_lang;
static lv_obj_t * g_admin_lbl_lang_title;
static lv_obj_t * g_admin_lbl_lang_line1;
static lv_obj_t * g_admin_btn_lang_zh;
static lv_obj_t * g_admin_btn_lang_en;
/* 恢复默认子页控件 */
static lv_obj_t * g_admin_panel_factory;
static lv_obj_t * g_admin_lbl_factory_title;
static lv_obj_t * g_admin_lbl_factory_line1;
static lv_obj_t * g_admin_lbl_factory_line2;
static lv_obj_t * g_admin_lbl_factory_status;
static lv_obj_t * g_admin_btn_factory_ok;
static lv_obj_t * g_admin_btn_factory_cancel;
/* 联系我们子页控件 */
static lv_obj_t * g_admin_panel_contact;
static lv_obj_t * g_admin_lbl_contact_title;
static lv_obj_t * g_admin_lbl_contact_line1;
static lv_obj_t * g_admin_lbl_contact_line2;
static lv_obj_t * g_admin_img_contact_qr;
/* 自投功能子页控件 */
static lv_obj_t * g_admin_panel_auto_dispense;
static lv_obj_t * g_admin_lbl_auto_dispense_title;
static lv_obj_t * g_admin_lbl_auto_dispense_line1;
static lv_obj_t * g_admin_lbl_auto_dispense_line2;
static lv_obj_t * g_admin_btn_auto_dispense_on;
static lv_obj_t * g_admin_btn_auto_dispense_off;
/* 臭氧功能子页控件 */
static lv_obj_t * g_admin_panel_ozone;
static lv_obj_t * g_admin_lbl_ozone_title;
static lv_obj_t * g_admin_lbl_ozone_line1;
static lv_obj_t * g_admin_lbl_ozone_line2;
static lv_obj_t * g_admin_btn_ozone_on;
static lv_obj_t * g_admin_btn_ozone_off;
/* 新风护理子页控件 */
static lv_obj_t * g_admin_panel_fresh_air_care;
static lv_obj_t * g_admin_lbl_fresh_air_care_title;
static lv_obj_t * g_admin_lbl_fresh_air_care_line1;
static lv_obj_t * g_admin_lbl_fresh_air_care_line2;
static lv_obj_t * g_admin_btn_fresh_air_care_on;
static lv_obj_t * g_admin_btn_fresh_air_care_off;
/* 系统升级子页控件 */
static lv_obj_t * g_admin_panel_system_upgrade;
static lv_obj_t * g_admin_lbl_system_upgrade_title;
static lv_obj_t * g_admin_lbl_system_upgrade_line1;
static lv_obj_t * g_admin_lbl_system_upgrade_status;
static lv_obj_t * g_admin_btn_system_upgrade_ok;
/* 支付设置子页控件 */
static lv_obj_t * g_admin_panel_payment;
static lv_obj_t * g_admin_lbl_payment_title;
static lv_obj_t * g_admin_lbl_payment_method_hdr;
static lv_obj_t * g_admin_lbl_payment_timeout_hdr;
static lv_obj_t * g_admin_lbl_payment_order_hdr;
static lv_obj_t * g_admin_cb_payment_alipay;
static lv_obj_t * g_admin_cb_payment_wechat;
static lv_obj_t * g_admin_payment_timeout_roller;
static lv_obj_t * g_admin_lbl_payment_order_hint;
static lv_obj_t * g_admin_btn_payment_query;
/* 数据设置子页控件 */
static lv_obj_t * g_admin_panel_data;
static lv_obj_t * g_admin_lbl_data_title;
static lv_obj_t * g_admin_lbl_data_upload_hdr;
static lv_obj_t * g_admin_lbl_data_strategy_hdr;
static lv_obj_t * g_admin_cb_data_upload[7];
static lv_obj_t * g_admin_cb_data_strategy[5];
static bool g_admin_data_strategy_ui_loading = false;
/* 网络设置子页控件 */
static lv_obj_t * g_admin_panel_network;
static lv_obj_t * g_admin_lbl_network_title;
static lv_obj_t * g_admin_btn_network_wifi;
static lv_obj_t * g_admin_btn_network_4g;
/* WIFI 设置子页控件 */
static lv_obj_t * g_admin_panel_wifi;
static lv_obj_t * g_admin_lbl_wifi_title;
/* 4G 设置子页控件 */
static lv_obj_t * g_admin_panel_4g;
static lv_obj_t * g_admin_lbl_4g_title;
static lv_obj_t * g_admin_sw_4g;
static lv_obj_t * g_admin_lbl_4g_prompt;
static lv_obj_t * g_admin_lbl_4g_status;
static lv_obj_t * g_admin_4g_done_center;
static lv_obj_t * g_admin_img_4g_done;
static lv_obj_t * g_admin_lbl_4g_done;
static admin_4g_phase_t g_admin_4g_phase = ADMIN_4G_PHASE_PROMPT;
static lv_timer_t * g_admin_4g_timer;
static bool g_admin_4g_ui_loading;
/* 密码修改子页控件 */
static lv_obj_t * g_admin_panel_pwd_chg_old;
static lv_obj_t * g_admin_ta_pwd_chg_old;
static lv_obj_t * g_admin_lbl_msg_pwd_chg_old;
static lv_obj_t * g_admin_panel_pwd_chg_new;
static lv_obj_t * g_admin_ta_pwd_chg_new1;
static lv_obj_t * g_admin_ta_pwd_chg_new2;
static lv_obj_t * g_admin_lbl_msg_pwd_chg_new;
static uint8_t g_admin_pwd_chg_new_step;  /* 0=第一框 1=第二框 */
static admin_factory_phase_t g_admin_factory_phase = ADMIN_FACTORY_PHASE_PROMPT;
static lv_timer_t * g_admin_factory_timer;
static admin_system_upgrade_phase_t g_admin_system_upgrade_phase = ADMIN_SYSTEM_UPGRADE_PHASE_PROMPT;
static lv_timer_t * g_admin_system_upgrade_timer;
static lv_obj_t * g_admin_prog_btns[TOTAL_PROGRAMS];
static lv_obj_t * g_admin_prog_ta[4];           /* 金额/洗涤/漂洗/脱水时长 */
static lv_obj_t * g_admin_prog_roller[3];       /* 漂洗次数/转速/水温 */
static lv_obj_t * g_admin_prog_field_box[8];      /* 8 列统一白底外框 */
static lv_obj_t * g_admin_prog_water_lbl;         /* 水位只读文字 */
static lv_obj_t * g_admin_prog_dash[8];         /* 不可用参数显示 -- */
static lv_obj_t * g_admin_btn_prog_reset;
static lv_obj_t * g_admin_btn_prog_confirm;
static int32_t g_admin_prog_sel;
static lv_obj_t * g_admin_prog_field_lbl[8];       /* 程序设置 8 参数字段名 */
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
static void admin_password_try(void);  //校验管理员密码并进入菜单
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
static int32_t admin_brightness_snap_slider(int32_t v);  //亮度滑条：对齐到 10 的倍数
static void cb_admin_brightness_switch_changed(lv_event_t * e);  //常亮开关 VALUE_CHANGED
static void admin_brightness_slider_sync_bar(int32_t v);  //同步下层 lv_bar 动条显示
static void admin_brightness_slider_sync_focus_frame(void);  //同步上层焦点框与滑条焦点态
static void cb_admin_brightness_slider_focus_frame(lv_event_t * e);  //滑条获焦/失焦时刷新焦点框
static void cb_admin_brightness_slider_changed(lv_event_t * e);  //亮度滑条 VALUE_CHANGED
static void cb_admin_brightness_slider_encoder(lv_event_t * e);  //亮度滑条编码器短按切换编辑/导航
static void cb_admin_brightness_slider_key(lv_event_t * e);  //亮度滑条编码器编辑态：左右旋转按步进 10 调值
static void admin_brightness_slider_exit_edit(lv_obj_t * slider);  //亮度滑条退出编码器编辑态
static void cb_admin_open_vendor_maint(lv_event_t * e);  //菜单「厂商维护」入口
static void admin_vendor_serial_back_to_menu1(void);
static void admin_vendor_menu_back_to_menu1(void);
static void admin_vendor_serial_try(void);
static void cb_admin_open_selfcheck(lv_event_t * e);
static void cb_admin_open_cycle(lv_event_t * e);
static void build_selfcheck(void);
static void build_cycle(void);
static void cycle_ui_reset(void);
static void cycle_finish_current_run(void);
static void cycle_enter_fault_display(void);
static void cycle_abort_run(void);
static void cycle_wash_sync_prog_pick_ui(void);
static void cycle_encoder_group_build(void);
static void cycle_run_count_label_sync(void);
static void cycle_fault_overlay_create(lv_obj_t * parent, lv_obj_t ** panel_out, lv_obj_t ** lbl_out);
static void cycle_fault_blink_stop(void);
static void cycle_fault_blink_start(void);
static void cycle_wash_build_program_panel(lv_obj_t * panel, lv_coord_t panel_h);
static void cb_cycle_prog_pick(lv_event_t * e);
static void cb_cycle_back(lv_event_t * e);
static void cb_cycle_runpause(lv_event_t * e);
static void cycle_start_run(void);
static void cycle_on_run_finished(void);
static void cycle_exit_to_vendor_menu(void);
static void selfcheck_ui_reset(void);
static void selfcheck_ui_goto_step(selfcheck_ui_state_t step);
static void cb_selfcheck_back(lv_event_t * e);
static void cb_selfcheck_runpause(lv_event_t * e);
static void selfcheck_timer_stop_all(void);
static void ui_admin_resume_unlocked(void);
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
static void admin_lang_back_to_menu1(void);  //离开语言设置页：回 menu1
static void cb_admin_open_language_settings(lv_event_t * e);  //菜单「语言设置」入口
static void cb_admin_lang_zh(lv_event_t * e);  //语言页「中文」
static void cb_admin_lang_en(lv_event_t * e);  //语言页「英文」
static void admin_factory_set_phase(admin_factory_phase_t phase);  //恢复默认子页：切换确认/恢复中/完成 UI
static void admin_factory_reset_ui_enter(void);  //进入恢复默认页：复位为确认前态
static void admin_factory_back_to_menu1(void);  //离开恢复默认页：回 menu1
static void admin_factory_run_restore(void);  //执行出厂恢复：程序/待机/机器 ID 等
static void admin_factory_timer_stop(void);  //停止恢复默认页 2s 完成态定时器
static void cb_admin_factory_cancel(lv_event_t * e);  //恢复默认「取消」：回菜单
static void cb_admin_factory_ok(lv_event_t * e);  //恢复默认「确定」：写回出厂数据
static void cb_admin_factory_timer(lv_timer_t * t);  //2s 定时器：恢复中 → 完成态
static void cb_admin_open_factory_reset(lv_event_t * e);  //菜单「恢复默认」入口
static void admin_contact_back_to_menu1(void);  //离开联系我们页：回 menu1
static void cb_admin_open_contact_us(lv_event_t * e);  //菜单「联系我们」入口
static void admin_auto_dispense_back_to_menu2(void);  //离开自投功能页：回 menu2
static void cb_admin_open_auto_dispense(lv_event_t * e);  //菜单「自投功能」入口
static void cb_admin_auto_dispense_on(lv_event_t * e);  //自投功能「开启」：ui_auto_dispense_set(true)
static void cb_admin_auto_dispense_off(lv_event_t * e);  //自投功能「关闭」：ui_auto_dispense_set(false)
static void admin_ozone_sync_btn_ui(void);  //臭氧功能页：刷新开启/关闭按钮选中样式
static void admin_ozone_back_to_menu2(void);  //离开臭氧功能页：回 menu2
static void cb_admin_open_ozone(lv_event_t * e);  //菜单「臭氧功能」入口
static void cb_admin_ozone_on(lv_event_t * e);  //臭氧功能「开启」：ui_ozone_set(true)
static void cb_admin_ozone_off(lv_event_t * e);  //臭氧功能「关闭」：ui_ozone_set(false)
static void admin_fresh_air_care_sync_btn_ui(void);  //新风护理页：刷新开启/关闭按钮选中样式
static void admin_fresh_air_care_back_to_menu2(void);  //离开新风护理页：回 menu2
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
static void admin_4g_back_to_menu2(void);  //离开 4G 设置页：回 menu2
static void admin_4g_timer_stop(void);  //停止 4G 配网 2s 完成态定时器
static void admin_4g_start_provisioning(void);  //4G 开关 OFF→ON：进入配网中并启动 2s 定时器
static void cb_admin_4g_switch_changed(lv_event_t * e);  //4G 开关切换：更新状态或触发配网流程
static void cb_admin_4g_timer(lv_timer_t * t);  //2s 定时器：配网中 → 成功态
static void cb_admin_open_network_settings(lv_event_t * e);  //menu2「网络设置」入口
static void cb_admin_open_wifi_settings(lv_event_t * e);  //网络设置「WIFI设置」入口
static void cb_admin_open_4g_settings(lv_event_t * e);  //网络设置「4G设置」入口
static void admin_network_back_to_menu2(void);  //离开网络设置页：回 menu2
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
static void cb_admin_payment_alipay_changed(lv_event_t * e);  //支付设置：支付宝复选框变更
static void cb_admin_payment_wechat_changed(lv_event_t * e);  //支付设置：微信复选框变更
static void cb_admin_payment_timeout_roller_encoder(lv_event_t * e);  //支付超时 roller 编码器短按切换编辑/导航
static void admin_payment_back_to_menu2(void);  //离开支付设置页：回 menu2
static void cb_admin_open_payment_settings(lv_event_t * e);  //menu2「支付设置」入口
static void admin_dormancy_sync_roller(void);
static void admin_dormancy_back_to_menu1(void);
static void cb_admin_dormancy_roller_changed(lv_event_t * e);
static void cb_admin_dormancy_roller_encoder(lv_event_t * e);
static void admin_data_back_to_menu2(void);
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
static lv_obj_t * make_admin_menu_btn(lv_obj_t * parent, const char * txt);  //创建管理员菜单渐变按钮
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
static lv_obj_t * g_btn_home_lang;
static lv_obj_t * g_lbl_home_lang;
static lv_obj_t * g_lbl_pay_price;
static lv_obj_t * g_lbl_pay_hint;
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
#define PAY_DUAL_COL_GAP   140
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
static lv_obj_t * g_pay_qr_wechat;
static lv_obj_t * g_kuaixi_run_lbl_status;
static lv_obj_t * g_kuaixi_run_lbl_stages;
static lv_obj_t * g_running_mode_label;     /* 运行页中间程序名，与 g_wheel_sel / 轮播一致 */
static lv_obj_t * g_running_time_label;     /* 运行页倒计时标签 */
static lv_obj_t * g_running_lbl_freq;       /* 运行页实时频率 label */
static lv_obj_t * g_running_lbl_water_mm;   /* 运行页实时水位 label */
static lv_obj_t * g_running_lbl_water_L;    /* 运行页实时水量 label */
static lv_obj_t * g_running_lbl_water_stage;/* 运行页实时分档 label */
static lv_obj_t * g_running_lbl_temp_live;  /* 运行页实时温度 label */
static lv_timer_t * g_running_countdown_timer;
#if UI_RUNNING_LIVE_PC_SIM
static lv_timer_t * g_running_freq_sim_timer; /* PC：频率 4350↔3500 每秒±10 */
#endif
static lv_timer_t * g_running_blink_timer;
static lv_timer_t * g_pay_done_timer;
static uint32_t g_running_remain_sec;
static bool g_running_countdown_paused;
static bool g_running_blink_visible;
static int32_t g_wheel_sel = MODE_IDX_DEFAULT; /* 0..TOTAL_PROGRAMS-1 */
int32_t* get_g_wheel_sel(void) { return &g_wheel_sel; }
static float g_wheel_turn = 0.f; /* fractional slot offset while dragging */
static lv_coord_t g_wheel_press_x;
static bool g_wheel_dragging;
static bool g_wheel_moved;
/* 运行页童锁 */
static lv_obj_t * g_running_mid;                 /* 中间栏：童锁按钮对齐参考 */
static lv_obj_t * g_running_btn_back;
static lv_obj_t * g_running_btn_runpause;
static lv_obj_t * g_running_btn_power;
static lv_obj_t * g_running_child_lock_btn;
static lv_obj_t * g_running_child_lock_img;
static lv_obj_t * g_running_lock_blocker;      /* 全屏遮罩：童锁时拦截触摸 */
static lv_obj_t * g_running_o3_img;           /* 运行页 O3 图标 */
static bool g_ui_child_lock;                     /* 童锁激活时拦截主页滑动及除童锁外的界面跳转 */
static lv_obj_t * g_home_btn_runpause;           /* 主页顶栏启停；轮播长按固定后编码器焦点回到此键 */
static lv_obj_t * g_home_btn_power;              /* 主页顶栏电源 */
static lv_obj_t * g_home_btn_admin;              /* 主页底栏管理员入口 */
static lv_obj_t * g_home_carousel_enc;           /* 轮播区编码器焦点代理（旋转选程序，双击回启停） */
static bool g_home_carousel_dbl_exit_suppress_click; /* 轮播双击回启停后，吞掉同一次松手的 CLICKED */
static uint32_t g_home_carousel_last_click_ms;        /* 双击间隔计时：第二次 CLICKED 在 400ms 内则退出编辑 */
static lv_timer_t * g_home_carousel_dbl_suppress_timer;

LV_IMAGE_DECLARE(img_01_dawu);
LV_IMAGE_DECLARE(img_02_dantuoshui);
LV_IMAGE_DECLARE(img_03_biaozhunxi);
LV_IMAGE_DECLARE(img_04_tongzijie);
LV_IMAGE_DECLARE(img_05_kuaixi);

LV_IMAGE_DECLARE(QR_Alipay_01_dawu);
LV_IMAGE_DECLARE(QR_Alipay_02_dantuoshui);
LV_IMAGE_DECLARE(QR_Alipay_03_biaozhunxi);
LV_IMAGE_DECLARE(QR_Alipay_04_tongzijie);
LV_IMAGE_DECLARE(QR_Alipay_05_kuaixi);
LV_IMAGE_DECLARE(QR_WeChat_01_dawu);
LV_IMAGE_DECLARE(QR_WeChat_02_dantuoshui);
LV_IMAGE_DECLARE(QR_WeChat_03_biaozhunxi);
LV_IMAGE_DECLARE(QR_WeChat_04_tongzijie);
LV_IMAGE_DECLARE(QR_WeChat_05_kuaixi);
LV_IMAGE_DECLARE(QR_code_xiaoya);

LV_IMAGE_DECLARE(bar_01);
LV_IMAGE_DECLARE(lack_of_softener);
LV_IMAGE_DECLARE(lack_of_detergent);
LV_IMAGE_DECLARE(wifi_logo);
LV_IMAGE_DECLARE(child_lock_logo);
LV_IMAGE_DECLARE(O3_logo);
LV_IMAGE_DECLARE(title_bar);
LV_IMAGE_DECLARE(input_box);
LV_IMAGE_DECLARE(delete);
LV_IMAGE_DECLARE(enter);

static const lv_image_dsc_t * const g_program_imgs[TOTAL_PROGRAMS] = {
    &img_01_dawu, &img_02_dantuoshui, &img_03_biaozhunxi, &img_04_tongzijie, &img_05_kuaixi
};

static const lv_image_dsc_t * const g_alipay_qr_imgs[TOTAL_PROGRAMS] = {
    &QR_Alipay_01_dawu, &QR_Alipay_02_dantuoshui, &QR_Alipay_03_biaozhunxi,
    &QR_Alipay_04_tongzijie, &QR_Alipay_05_kuaixi
};
static const lv_image_dsc_t * const g_wechat_qr_imgs[TOTAL_PROGRAMS] = {
    &QR_WeChat_01_dawu, &QR_WeChat_02_dantuoshui, &QR_WeChat_03_biaozhunxi,
    &QR_WeChat_04_tongzijie, &QR_WeChat_05_kuaixi
};

/* 程序参数：与 g_mode_name_ids[] / g_program_imgs[] 下标 0=大物 … 4=快洗 */
typedef struct {
    uint32_t wash_sec;       /* 主页底部：程序总运行时间（秒）= 洗涤+漂洗×次数+脱水 */
    const char * temp;
    int32_t price;           /* 默认金额；-1 表示无 */
} ui_program_profile_t;

typedef struct {
    uint32_t wash_sec;       /* 洗涤阶段时长（秒）；0 表示无洗涤 */
    uint32_t rinse_sec;      /* 漂洗总时长（秒）= 单次漂洗 × 漂洗次数 */
    uint32_t spin_sec;       /* 脱水阶段时长（秒） */
    ui_str_id_t stages_bar_id;
} ui_program_run_stages_t;

/* 管理员「程序设置」扩展参数（表2.4 程序时限表；配置层时长单位为分钟） */
#define PROG_ADMIN_DEMO_SEC   0   /* 1=PC 演示用秒；0=配置存分钟，运行层 ×60 换算为秒 */
#define PROG_CAP_PRICE      0x0001u
#define PROG_CAP_WASH       0x0002u
#define PROG_CAP_RINSE_DUR  0x0004u
#define PROG_CAP_SPIN_DUR   0x0008u
#define PROG_CAP_RINSE_CNT  0x0010u
#define PROG_CAP_SPIN_RPM   0x0020u
#define PROG_CAP_TEMP       0x0040u
#define PROG_CAP_WATER      0x0080u

typedef struct {
    int32_t price;          /* 程序金额（元）；-1 表示 -- */
    uint16_t wash_min;      /* 单次洗涤时长（分钟）；无洗涤时忽略 */
    uint16_t rinse_min;     /* 单次漂洗时长（分钟） */
    uint16_t spin_min;      /* 脱水时长（分钟） */
    int8_t rinse_cnt;       /* 漂洗次数 0~4；-1 表示 -- */
    int16_t spin_rpm;       /* 脱水转速；-1 表示 -- */
    int8_t temp_idx;        /* 0=COLD 1=30℃ … 4=90℃；-1 表示 -- */
    bool water_smart;       /* true 显示「智能设定」 */
    uint16_t cap;           /* 各参数是否可用（见 PROG_CAP_*） */
} ui_program_admin_t;

typedef struct {
    int32_t prog_idx;
    ui_program_admin_t cfg;
    uint32_t run_count;
    uint32_t fault_at_count;
    uint8_t fault_code;
} cycle_session_t;

static ui_program_admin_t g_cycle_cfg[TOTAL_PROGRAMS];
static cycle_session_t g_cycle_session;

static ui_program_profile_t g_program_profiles[TOTAL_PROGRAMS];
static ui_program_run_stages_t g_program_run_stages[TOTAL_PROGRAMS];
static ui_program_admin_t g_prog_cfg[TOTAL_PROGRAMS];
static ui_program_admin_t g_prog_cfg_factory[TOTAL_PROGRAMS];

static void program_admin_init_factory(void);  //写入表3.1 程序初值并同步到主页/运行页
static void program_admin_apply_one(int32_t idx);  //将单程序 cfg 同步到主页/运行页参数表
static void program_admin_apply_all(void);  //将全部程序 cfg 同步到主页/运行页
static void program_admin_ui_save_fields(void);  //从程序设置 UI 控件写回 g_prog_cfg
static void program_admin_ui_load_fields(void);  //将 g_prog_cfg 加载到程序设置 UI 控件
static void program_admin_ui_apply_caps(void);  //按 cap 显示/隐藏程序设置各参数字段
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
    lv_obj_t * back_target, lv_group_t * encoder_group, lv_obj_t ** back_btn_out);  //创建顶部栏：可选返回键 + 状态栏
static lv_obj_t * add_encoder_top_btn(lv_obj_t * top, const char * txt, lv_coord_t x, lv_group_t * group);  //在顶部栏添加编码器可聚焦按钮
static void carousel_size_cb(lv_event_t * e);  //轮播区尺寸变化时重新布局
static void carousel_update_card_images(void);  //按当前选中程序刷新 5 张可见卡片图与名称标签
static lv_coord_t carousel_slot_center_x(int slot);  //计算槽位相对轮播中心的水平偏移
static void carousel_wrap_relayout(void);  //轮播区整体布局：卡片位置/缩放/透明度与指示点
static int32_t wheel_mod_total(int32_t v);  //程序索引在 0..TOTAL_PROGRAMS-1 内循环取模
static void home_sync_encoder_focus_after_carousel_drag(void);  //触摸滑动改程序后对齐编码器焦点
static void home_encoder_group_build(void);  //主页编码器：启停→电源→语言→管理员→轮播（不首尾循环）
static void home_carousel_encoder_lock_and_exit(void);  //轮播双击固定居中并焦点回启停
static void cb_home_carousel_encoder(lv_event_t * e);  //轮播编码器：编辑模式旋转选程序，双击回启停
static void cb_home_runpause(lv_event_t * e);  //主页启停：吞掉轮播双击回启停后的误触 CLICKED
static void ui_indev_encoder_reset_long_press(void);  //重置编码器长按状态，避免松手误判短按
static void cb_home_prog_dot_focus(lv_event_t * e);  //轮播指示点：触摸点击同步选中程序
static void wheel_pointer_cb(lv_event_t * e);  //轮播区按下/拖动/释放：滑动切换程序
static void mode_card_click(lv_event_t * e);  //轮播居中卡片点击：进入支付页
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
static void cb_off_wake(lv_event_t * e);  //待机页任意按钮：恢复待机前界面（首屏待机则进主页）
static void ui_apply_indev_long_press_ms(uint16_t ms);  //统一设置指针/编码器长按判定时间
static void ui_indev_set_encoder_long_press_ms(uint16_t ms);  //仅编码器长按阈值（轮播 800ms / 其它 3s）
static void running_child_lock_on_enter(void);  //童锁激活时的进入钩子（预留扩展）
static void running_child_lock_on_exit(void);  //童锁解除时的退出钩子（预留扩展）
static void running_child_lock_align_btn(void);  //将童锁按钮对齐到运行页中间栏右侧
static void running_child_lock_apply_locked(bool locked, bool silent_unlock);  //应用童锁锁定/解锁 UI 与 group
static void running_child_lock_remote_clear_if_leaving_running(lv_obj_t * target_scr);  //离开运行页时静默解除童锁
static void cb_running_child_lock_long(lv_event_t * e);  //童锁按钮长按：切换童锁开/关
static void cb_running_child_lock_released(lv_event_t * e);  //童锁松开：焦点回到童锁按钮
static void build_home(void);  //构建主页：顶部栏、程序轮播、底部参数栏
static const ui_program_profile_t * program_profile_get(int32_t idx);  //获取指定程序索引的参数表项
static void program_format_time_label(uint32_t sec, char * buf, size_t buf_sz);  //格式化为首页时间标签文本
static void program_format_price_home(int32_t price, char * buf, size_t buf_sz);  //格式化为首页价格标签
static void program_format_price_pay(int32_t price, char * buf, size_t buf_sz);  //格式化为支付页价格标签
static void home_sync_program_labels(void);  //按当前选中程序刷新主页底部时间/温度/价格标签
static void pay_sync_price_label(void);
static void pay_sync_pay_ui(void);
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
static void running_live_params_sync(void);  //刷新运行页实时水位/温度 label
static void build_running(void);  //构建运行页：背景、顶部栏、程序名/倒计时、童锁
static void build_end(void);  //构建洗涤结束页：返回、图标与提示文字
static void build_admin(void);  //构建管理员页：密码/设置网格/机器ID子面板
static void build_alarm_overlay(void);  //构建 lv_layer_top 报警弹层：E1–E15 + 缺液
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
static const lv_font_t * s_font_sc_125;

static void ui_fsm_poll_running_pause_sync(void);
static void ui_fsm_runpause_apply(bool long_press);
static void ui_fsm_runpause_apply_running_page(void);
static void cb_power_long(lv_event_t * event);
static void cb_runpause(lv_event_t * event);
static void cb_runpause_long(lv_event_t * event);
static void cb_home_power_alarm_sim(lv_event_t * e);  //PC：主页电源键仿真报警

static void cb_power_long(lv_event_t* event)
{
	(void)event;

	if(fsm.state != FSM_OFF) {
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
	ui_fsm_runpause_apply(false);
}

static void cb_runpause_long(lv_event_t* event)
{
	(void)event;
	ui_fsm_runpause_apply(true);
}

#if UI_DEBUG_PAY_SKIP_TO_DONE
/* 调试：支付页启停模拟支付成功，走支付完成 → 运行页 */
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
	if(g_home_carousel_dbl_exit_suppress_click) return;

	if(g_cycle_active && lv_scr_act() == g_scr_running) {
		cycle_abort_run();
		return;
	}

	if(fsm.state == FSM_STANDBY && lv_screen_active() == g_scr_home) {
		SETFLAG(FSM_FLAG_NEED_PAYMENT);
		ui_scr_load_async();
	} else if(fsm.state == FSM_RUNNING) {
		fsm_state_change(FSM_PAUSED);
	} else if(fsm.state == FSM_PAUSED) {
		fsm_state_change(FSM_RUNNING);
	} else if(fsm.state == FSM_OFF) {
		fsm_state_change(FSM_STANDBY);
	}

	ui_fsm_poll_running_pause_sync();
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
	if(s_font_sc_35 == NULL) return;
	s_font_admin_kb = *s_font_sc_35;
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
	lv_obj_set_style_radius(b, 8, LV_PART_MAIN);                              //圆角半径 8
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
	lv_obj_set_style_radius(b, 8, LV_PART_MAIN);                              //圆角半径 8
	lv_obj_set_style_bg_opa(b, LV_OPA_TRANSP, LV_PART_MAIN);                  //背景透明
	lv_obj_set_style_border_width(b, 2, LV_PART_MAIN);                        //边框宽度为 2
	lv_obj_set_style_border_color(b, lv_color_hex(COL_ORANGE), LV_PART_MAIN); //边框颜色 COL_ORANGE
	lv_obj_t * l = lv_label_create(b);
	lv_label_set_text(l, txt);
	lv_obj_set_style_text_color(l, lv_color_hex(COL_ORANGE), LV_PART_MAIN);   //文本颜色 COL_ORANGE
	lv_obj_center(l);
	return b;
}//创建橙色描边按钮

/* 定时器回调：每秒更新各页顶部时钟 label（HH:MM，非 i18n） */

static void cb_clock(lv_timer_t * t)
{
	(void)t;                                             /* 未使用定时器指针 */
	static uint32_t s_clock_seconds = 10u * 3600u + 8u * 60u; /* 仿真时钟起点 10:08:00 */
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
	if(g_lbl_clock_admin != NULL) {                      /* 管理员页状态栏时钟 */
		lv_label_set_text(g_lbl_clock_admin, buf);
	}
	if(g_lbl_clock_selfcheck != NULL) {                  /* 自检页状态栏时钟 */
		lv_label_set_text(g_lbl_clock_selfcheck, buf);
	}
	if(g_lbl_clock_cycle != NULL) {                      /* 循环程序页状态栏时钟 */
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
	lv_obj_align(t4g, LV_ALIGN_BOTTOM_LEFT, LV_PCT(25), -10);
	lv_obj_set_style_text_color(t4g, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(t4g, s_font_sc_27);

	lv_obj_t * tw = lv_image_create(status);
	lv_image_set_src(tw, &wifi_logo);
	lv_obj_align(tw, LV_ALIGN_BOTTOM_LEFT, LV_PCT(47), -10);

	lv_obj_t * clock = lv_label_create(status);
	lv_label_set_text(clock, "10:08");
	lv_obj_align(clock, LV_ALIGN_BOTTOM_RIGHT, 0, -10);
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
		cb_clock(NULL);  //鍒锋柊鏃堕挓
	}
}

/* 顶部栏：可选返回 + 4G / WiFi / 时间（启停、电源由各界面单独添加） */
static lv_obj_t * create_top_bar(lv_obj_t * parent, lv_obj_t ** clock_lbl_out,
	lv_obj_t * back_target, lv_group_t * encoder_group, lv_obj_t ** back_btn_out)  //创建顶部栏
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
		lv_obj_t * imgbtn_back = lv_imgbtn_create(top);
		lv_imgbtn_set_src(imgbtn_back, LV_IMGBTN_STATE_RELEASED, NULL, &back, NULL);
		lv_obj_align(imgbtn_back, LV_ALIGN_LEFT_MID, 20, 0);
		lv_obj_add_event_cb(imgbtn_back, cb_load_screen, LV_EVENT_CLICKED, back_target);
		lv_obj_remove_flag(imgbtn_back, LV_OBJ_FLAG_SCROLLABLE);
		if(encoder_group != NULL) {
			ui_encoder_group_add(encoder_group, imgbtn_back);
		}
		if(back_btn_out != NULL) {
			*back_btn_out = imgbtn_back;
		}
	}

	create_top_status_bar(top, clock_lbl_out);  //创建顶部右侧状态栏：4G / WiFi / 时间
	return top;
}

//在顶部栏添加编码器可聚焦的启停/电源类文本按钮
static lv_obj_t * add_encoder_top_btn(lv_obj_t * top, const char * txt, lv_coord_t x, lv_group_t * group)  //在顶部栏添加编码器可聚焦的启停/电源类文本按钮
{
	lv_obj_t * btn = make_text_btn(top, txt, 80, 32);  //创建透明背景文本按钮
	lv_obj_align(btn, LV_ALIGN_LEFT_MID, x, 0);
	lv_obj_remove_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
	if(group != NULL) {
		ui_encoder_group_add(group, btn);
	}
	return btn;
}

//轮播区函数定义
//轮播区尺寸变化事件回调函数
static void carousel_size_cb(lv_event_t * e)  //轮播区尺寸变化时重新布局
{
	(void)e;
	carousel_wrap_relayout();  //重排轮播布局
}

/* 按当前选中程序刷新 5 张可见卡片图与名称标签（槽位 i 显示 g_wheel_sel + (i - center)） */

static void carousel_update_card_images(void)
{
	for(int i = 0; i < CAROUSEL_VISIBLE_SLOTS; i++) {    /* 遍历 5 个可见槽位 */
		int32_t prog_idx = wheel_mod_total(g_wheel_sel + (int32_t)(i - CAROUSEL_CENTER_SLOT)); /* 该槽对应程序下标 */
		if(g_mode_card_imgs[i] != NULL) {
			lv_image_set_src(g_mode_card_imgs[i], g_program_imgs[prog_idx]); /* 换程序图标 */
		}
		if(g_mode_card_labels[i] != NULL) {
			lv_label_set_text(g_mode_card_labels[i], ui_program_name_get(prog_idx)); /* 换程序名文字（中/英） */
		}
	}
}

/* 5 张卡片中心相对轮播区中心的 X 偏移：外侧两对更紧，靠近中心两对更疏 */
static lv_coord_t carousel_slot_center_x(int slot)  //计算槽位相对轮播中心的水平偏移
{
	switch(slot) {
		case 0: return -(s_mode_gap_inner + s_mode_gap_outer);
		case 1: return -s_mode_gap_inner;
		case 2: return 0;
		case 3: return s_mode_gap_inner;
		case 4: return s_mode_gap_inner + s_mode_gap_outer;
		default: return 0;
	}
}

//轮播区布局
static void carousel_wrap_relayout(void)  //重排轮播布局
{
	if(g_mode_carousel == NULL) return;
	lv_coord_t mw = lv_obj_get_width(g_mode_carousel);
	lv_coord_t mh = lv_obj_get_height(g_mode_carousel);
	if(mw < 16 || mh < 16) return;

	carousel_update_card_images();  //按当前选中程序刷新 5 张可见卡片图

	lv_coord_t cx = mw / 2;
	lv_coord_t cy = mh / 2;
	/* 五个圆同一水平中线，相对区域垂直中心略向上 */
	const lv_coord_t y = cy - s_mode_card_sz / 2 - s_mode_row_shift_up;

	for(int i = 0; i < CAROUSEL_VISIBLE_SLOTS; i++) {
		if(g_mode_cards[i] == NULL) continue;
		float x_rel = (float)(i - CAROUSEL_CENTER_SLOT) + g_wheel_turn;

		lv_coord_t x = cx + carousel_slot_center_x(i)  //计算槽位相对轮播中心的水平偏移
			+ (lv_coord_t)(g_wheel_turn * (float)s_mode_step_x) - s_mode_card_sz / 2;
		float af = x_rel < 0.f ? -x_rel : x_rel;
		if(af > 2.5f) af = 2.5f;

		lv_obj_set_pos(g_mode_cards[i], x, y);

		float t = 1.0f - ((af > 2.0f) ? 2.0f : af) / 2.0f;
		int32_t scale = MODE_SCALE_MIN + (int32_t)((float)(MODE_SCALE_MAX - MODE_SCALE_MIN) * t);
		lv_obj_set_style_transform_pivot_x(g_mode_cards[i], s_mode_card_sz / 2, LV_PART_MAIN);
		lv_obj_set_style_transform_pivot_y(g_mode_cards[i], s_mode_card_sz / 2, LV_PART_MAIN);
		lv_obj_set_style_transform_scale_x(g_mode_cards[i], scale, LV_PART_MAIN);
		lv_obj_set_style_transform_scale_y(g_mode_cards[i], scale, LV_PART_MAIN);

		// 调整透明度计算逻辑：中心(af=0)透明度约255，最两边(af=2)透明度约为 255 * 30% ≈ 76
		lv_opa_t opa;
		if(af <= 0.1f) opa = LV_OPA_COVER;
		else if(af >= 2.0f) opa = (lv_opa_t)(LV_OPA_30);
		else {
			opa = (lv_opa_t)(255 - (int)(af * (255 - 76) / 2.0f));
		}
		lv_obj_set_style_opa(g_mode_cards[i], opa, LV_PART_MAIN);
	}

	if(g_mode_cards[CAROUSEL_CENTER_SLOT] != NULL) {
		lv_obj_move_foreground(g_mode_cards[CAROUSEL_CENTER_SLOT]);
	}

	lv_coord_t dot_w[TOTAL_PROGRAMS];
	lv_coord_t total_w = 0;
	const lv_coord_t dot_gap = 8;
	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		dot_w[i] = (i == g_wheel_sel) ? 13 : 10;
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
		lv_obj_set_style_bg_color(g_mode_dots[i], lv_color_hex((i == g_wheel_sel) ? COL_TEXT : COL_DIM), LV_PART_MAIN);
		x_cursor += dot_w[i] + dot_gap;
	}
}

//程序索引在 0..TOTAL_PROGRAMS-1 范围内循环
static int32_t wheel_mod_total(int32_t v)  //程序索引取模
{
	v %= TOTAL_PROGRAMS;
	if(v < 0) v += TOTAL_PROGRAMS;
	return v;
}


// 触摸滑动改程序后，将编码器焦点对齐到当前程序指示点

static void home_sync_encoder_focus_after_carousel_drag(void)
{
	if(g_group_home == NULL) return;
	if(g_home_carousel_enc != NULL) {
		lv_group_focus_obj(g_home_carousel_enc);
		if(g_group_home != NULL) {
			lv_group_set_editing(g_group_home, true);
		}
	}
}

// 主页编码器 focus 顺序：启停→电源→语言→管理员→轮播；不首尾循环，轮播仅双击回启停

static void home_encoder_group_build(void)
{
	if(g_group_home == NULL) return;
	lv_group_remove_all_objs(g_group_home);
	if(g_home_btn_runpause != NULL) ui_encoder_group_add(g_group_home, g_home_btn_runpause);
	if(g_home_btn_power != NULL) ui_encoder_group_add(g_group_home, g_home_btn_power);
	if(g_btn_home_lang != NULL) ui_encoder_group_add(g_group_home, g_btn_home_lang);
	if(g_home_btn_admin != NULL) ui_encoder_group_add(g_group_home, g_home_btn_admin);
	if(g_home_carousel_enc != NULL) ui_encoder_group_add(g_group_home, g_home_carousel_enc);
	lv_group_set_wrap(g_group_home, false);
	if(g_home_btn_runpause != NULL) {
		lv_group_focus_obj(g_home_btn_runpause);
		lv_group_set_editing(g_group_home, false);
	}
}

static void home_carousel_dbl_suppress_timer_cb(lv_timer_t * t)
{
	(void)t;
	g_home_carousel_dbl_suppress_timer = NULL;
	g_home_carousel_dbl_exit_suppress_click = false;
}

static void ui_indev_encoder_reset_long_press(void)
{
	lv_indev_t * indev = NULL;
	while((indev = lv_indev_get_next(indev)) != NULL) {
		if(lv_indev_get_type(indev) == LV_INDEV_TYPE_ENCODER) {
			lv_indev_reset_long_press(indev);
			break;
		}
	}
}

static void home_carousel_encoder_lock_and_exit(void)
{
	if(g_ui_child_lock) return;
	g_wheel_turn = 0.f;
	carousel_wrap_relayout();
	home_sync_program_labels();

	/* 双击过程中已切到启停：松手时 LVGL 仍可能对启停发 CLICKED，需隔开 */
	g_home_carousel_dbl_exit_suppress_click = true;
	if(g_home_carousel_dbl_suppress_timer != NULL) {
		lv_timer_delete(g_home_carousel_dbl_suppress_timer);
	}
	g_home_carousel_dbl_suppress_timer = lv_timer_create(home_carousel_dbl_suppress_timer_cb, 400, NULL);
	lv_timer_set_repeat_count(g_home_carousel_dbl_suppress_timer, 1);

	ui_indev_encoder_reset_long_press();
	if(g_home_carousel_enc != NULL) {
		lv_obj_remove_state(g_home_carousel_enc, LV_STATE_PRESSED);
	}

	if(g_group_home != NULL) {
		lv_group_set_editing(g_group_home, false); /* 导航模式：旋转在启停/电源/语言/管理员/轮播间移动 */
	}
	if(g_home_btn_runpause != NULL) {
		lv_obj_remove_state(g_home_btn_runpause, LV_STATE_PRESSED);
		lv_group_focus_obj(g_home_btn_runpause);
	}
}

static void home_carousel_encoder_step(int32_t delta)
{
	g_wheel_sel = wheel_mod_total(g_wheel_sel + delta);
	g_wheel_turn = 0.f;
	carousel_wrap_relayout();
	home_sync_program_labels();
}

/* 主页启停：FSM 启动；吞掉轮播双击回启停后的误触 CLICKED */

static void cb_home_runpause(lv_event_t * e)
{
	lv_event_code_t code = lv_event_get_code(e);
	if(code == LV_EVENT_FOCUSED) {
		if(g_group_home != NULL) {
			lv_group_set_editing(g_group_home, false);
		}
		return;
	}
	if(code == LV_EVENT_CLICKED) {
		if(g_home_carousel_dbl_exit_suppress_click) {
			g_home_carousel_dbl_exit_suppress_click = false;
			if(g_home_carousel_dbl_suppress_timer != NULL) {
				lv_timer_delete(g_home_carousel_dbl_suppress_timer);
				g_home_carousel_dbl_suppress_timer = NULL;
			}
			lv_event_stop_processing(e);
			return;
		}
		cb_runpause(e);
		return;
	}
	if(code == LV_EVENT_RELEASED && g_home_btn_runpause != NULL) {
		lv_obj_remove_state(g_home_btn_runpause, LV_STATE_PRESSED);
	}
}

/* 轮播编码器：编辑模式旋转选程序；DOUBLE_CLICK(400ms) 固定居中并回启停 */

static void cb_home_carousel_encoder(lv_event_t * e)
{
	if(g_ui_child_lock) return;
	if(g_home_carousel_enc == NULL || lv_event_get_target(e) != g_home_carousel_enc) return;

	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_FOCUSED) {
		g_home_carousel_last_click_ms = 0;
		if(g_group_home != NULL) {
			lv_group_set_editing(g_group_home, true);
		}
		return;
	}
	if(code == LV_EVENT_DEFOCUSED) {
		g_home_carousel_last_click_ms = 0;
		if(g_group_home != NULL && lv_group_get_focused(g_group_home) != g_home_carousel_enc) {
			lv_group_set_editing(g_group_home, false);
		}
		return;
	}
	if(code == LV_EVENT_CLICKED) {
		uint32_t now = lv_tick_get();
		if(g_home_carousel_last_click_ms != 0 &&
		   lv_tick_elaps(g_home_carousel_last_click_ms) <= HOME_CAROUSEL_DOUBLE_CLICK_MS) {
			g_home_carousel_last_click_ms = 0;
			home_carousel_encoder_lock_and_exit();
		} else {
			g_home_carousel_last_click_ms = now;
		}
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

/* 轮播指示点：仅触摸点击时同步选中程序（编码器走 g_home_carousel_enc） */

static void cb_home_prog_dot_focus(lv_event_t * e)
{
	if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
	int32_t idx = (int32_t)(intptr_t)lv_event_get_user_data(e);
	if(idx < 0 || idx >= TOTAL_PROGRAMS) return;
	if(g_wheel_sel != idx) {
		g_wheel_sel = idx;
		g_wheel_turn = 0.f;
		carousel_wrap_relayout();
		home_sync_program_labels();
	}
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
		g_wheel_press_x = p.x;
		g_wheel_dragging = true;
		g_wheel_turn = 0.f;
		g_wheel_moved = false;
		return;
	}

	if(code == LV_EVENT_PRESSING && g_wheel_dragging) {
		lv_coord_t dx = p.x - g_wheel_press_x;
		g_wheel_turn = (float)dx / (float)s_mode_step_x;
		carousel_wrap_relayout();  //重排轮播布局
		return;
	}

	if(code == LV_EVENT_RELEASED && g_wheel_dragging) {
		lv_coord_t dx = p.x - g_wheel_press_x;
		g_wheel_dragging = false;

		if(LV_ABS(dx) < 14) {
			g_wheel_turn = 0.f;
			carousel_wrap_relayout();  //重排轮播布局
			return;
		}

		/* 左滑(dx<0)：索引前进；右滑：索引后退。|dx| 每满一格 s_mode_drag_snap_px 多切 1 个程序 */
		int k;
		if(dx < 0) k = (-dx) / s_mode_drag_snap_px;
		else k = dx / s_mode_drag_snap_px;
		if(k == 0) k = 1;

		int32_t delta = (dx < 0) ? k : -k;
		g_wheel_sel = wheel_mod_total((int32_t)g_wheel_sel + delta);  //程序索引取模

		g_wheel_turn = 0.f;
		g_wheel_moved = true;
		carousel_wrap_relayout();						//轮播区布局
		home_sync_program_labels();  //刷新主页程序标签
		home_sync_encoder_focus_after_carousel_drag();
	}
}

//轮播区卡片点击（仅居中槽位：与主页启停相同，经 FSM 进支付页）
static void mode_card_click(lv_event_t * e)  //轮播居中卡片点击
{
	if(g_ui_child_lock) return;
	if(g_wheel_moved) return;
	if(lv_scr_act() != g_scr_home) return;
	intptr_t slot = (intptr_t)lv_event_get_user_data(e);
	if(slot != CAROUSEL_CENTER_SLOT) return;
	cb_runpause(NULL);
}







static void cb_load_screen(lv_event_t * e)  //通用界面加载事件回调函数，根据事件参数加载对应界面
{
	if(g_ui_child_lock) return;
	lv_obj_t * scr = (lv_obj_t *)lv_event_get_user_data(e);
	if(scr == NULL) return;
	if(g_cycle_active && lv_scr_act() == g_scr_running && scr == g_scr_home) {
		cycle_abort_run();
		return;
	}
	ui_screen_load(scr);  //加载目标屏幕
}

static void cb_load_admin(lv_event_t * e)
{
	(void)e;
	if(g_ui_child_lock) return;
	ui_screen_load(g_scr_admin);
}

static void ui_layout_init(void)    //界面布局参数初始化
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

// 选中键盘首键「1」

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
		if(g_group_admin != NULL && lv_group_get_editing(g_group_admin) &&
		   lv_keyboard_get_selected_button(kb) == LV_BUTTONMATRIX_BUTTON_NONE) {
			admin_kb_encoder_select_first(kb);
		}
	}
	else if(code == LV_EVENT_KEY) {
		if(g_group_admin != NULL && lv_group_get_editing(g_group_admin)) {
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
	lv_group_set_editing(g_group_admin, true);
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
		lv_group_set_editing(g_group_admin, false);
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

/* 弹出键盘并绑定输入框 */
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

	if(g_group_admin != NULL && lv_group_get_editing(g_group_admin) &&
	   admin_kb_is_visible() && g_admin_kb != NULL &&
	   lv_group_get_focused(g_group_admin) == g_admin_kb) {
		return;
	}

	if(g_admin_view == PASSWORD_CHANGE_NEW && ta != NULL) {
		if(ta == g_admin_ta_pwd_chg_new1) {
			g_admin_pwd_chg_new_step = 0;
		}
		else if(ta == g_admin_ta_pwd_chg_new2) {
			g_admin_pwd_chg_new_step = 1;
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
			g_admin_pwd_chg_new_step = 0;
		}
		else if(ta == g_admin_ta_pwd_chg_new2) {
			g_admin_pwd_chg_new_step = 1;
		}
	}

	if(code == LV_EVENT_FOCUSED) {
		if(g_group_admin != NULL && lv_group_get_editing(g_group_admin) && admin_kb_is_visible()) {
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
	g_scr_admin = lv_obj_create(NULL);
	g_scr_selfcheck = lv_obj_create(NULL);
	g_scr_cycle = lv_obj_create(NULL);
	style_screen_base(g_scr_off);  //设置屏幕基础样式（黑底、无边框）
	style_screen_base(g_scr_home);             //主页
	style_screen_base(g_scr_running);          //运行
	style_screen_base(g_scr_end);                //运行结束
	style_screen_base(g_scr_pay);                  //支付
	style_screen_base(g_scr_pay_done);             //支付完成
	style_screen_base(g_scr_admin);                //管理员
	style_screen_base(g_scr_selfcheck);
	style_screen_base(g_scr_cycle);
	lv_obj_set_size(g_scr_off, UI_FIXED_W, UI_FIXED_H);
	lv_obj_set_size(g_scr_home, UI_FIXED_W, UI_FIXED_H);              //主页宽高
	lv_obj_set_size(g_scr_running, UI_FIXED_W, UI_FIXED_H);           //运行宽高
	lv_obj_set_size(g_scr_end, UI_FIXED_W, UI_FIXED_H);
	lv_obj_set_size(g_scr_pay, UI_FIXED_W, UI_FIXED_H);
	lv_obj_set_size(g_scr_pay_done, UI_FIXED_W, UI_FIXED_H);
	lv_obj_set_size(g_scr_admin, UI_FIXED_W, UI_FIXED_H);
	lv_obj_set_size(g_scr_selfcheck, UI_FIXED_W, UI_FIXED_H);
	lv_obj_set_size(g_scr_cycle, UI_FIXED_W, UI_FIXED_H);
}

//将编码器/键盘输入设备绑定到指定 focus group
static void ui_set_encoder_group(lv_group_t * group)  //切换编码器 group
{
	if(group == NULL) return;
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

	if(scr == g_scr_off) {
		ui_set_encoder_group(g_group_off);
	}
	else if(scr == g_scr_home) {
		ui_set_encoder_group(g_group_home);              /* 主页：含语言/轮播焦点 */
	}
	else if(scr == g_scr_running) {
		ui_set_encoder_group(g_group_running);
		running_screen_sync_mode_name();                 /* 刷新运行页程序名 label */
		running_live_params_sync();                      /* 刷新运行页实时参数 label */
		/* 同步 O3 图标可见性 */
		if(g_running_o3_img != NULL) {
			if(ui_ozone_get()) lv_obj_remove_flag(g_running_o3_img, LV_OBJ_FLAG_HIDDEN);
			else lv_obj_add_flag(g_running_o3_img, LV_OBJ_FLAG_HIDDEN);
		}
		if(g_cycle_active && g_cycle_ui_state == CYCLE_UI_FAULT && g_cycle_fault_on_running) {
			cycle_fault_blink_start();
		} else {
			running_countdown_start();                       /* 刷新运行页倒计时 label */
		}
	}
	else if(scr == g_scr_end) {
		ui_set_encoder_group(g_group_end);               /* 结束页 label 已在 build 时绑定 i18n */
	}
	else if(scr == g_scr_pay) {
		param_change[0] = g_prog_cfg[g_wheel_sel].wash_min;
		param_change[1] = g_prog_cfg[g_wheel_sel].rinse_min;
		param_change[2] = g_prog_cfg[g_wheel_sel].rinse_cnt <= 0 ? 0 : g_prog_cfg[g_wheel_sel].rinse_cnt;
		param_change[3] = g_prog_cfg[g_wheel_sel].temp_idx <= 0 ? 0 : g_prog_cfg[g_wheel_sel].temp_idx;
		param_change[4] = g_prog_cfg[g_wheel_sel].spin_rpm;
		param_change[5] = g_prog_cfg[g_wheel_sel].spin_min;
		param_change[6] = g_prog_cfg[g_wheel_sel].water_smart ? prog_table_comm[g_wheel_sel][6] : 0;
		switch(g_wheel_sel) {
			case 0: {param_change[7] = 3;break;}
			case 1: {param_change[7] = 4;break;}
			case 2: {param_change[7] = 2;break;}
			case 3: {param_change[7] = 5;break;}
			case 4: {param_change[7] = 1;break;}
			default: {param_change[7] = 3;break;}
		}
		ui_set_encoder_group(g_group_pay);
		pay_sync_price_label();                          /* 刷新 g_lbl_pay_price */
		pay_sync_pay_ui();
		if(g_pay_focus_back_on_enter && g_pay_btn_back != NULL) {
			lv_group_focus_obj(g_pay_btn_back);          /* 避免焦点落在启停上误进支付完成 */
			g_pay_focus_back_on_enter = false;
		}
	}
	else if(scr == g_scr_pay_done) {
		ui_set_encoder_group(g_group_pay_done);
		pay_done_timer_start();                          /* 2s 后进入运行页 */
	}
	else if(scr == g_scr_admin) {
		admin_session_reset();
		ui_set_encoder_group(g_group_admin);
	}
	else if(scr == g_scr_selfcheck) {
		ui_set_encoder_group(g_group_selfcheck);
		selfcheck_ui_reset();
	}
	else if(scr == g_scr_cycle) {
		ui_set_encoder_group(g_group_cycle);
		cycle_run_count_label_sync();
	}
	else {
		ui_set_encoder_group(g_ui_group);                /* 未知屏用默认组 */
	}
	ui_idle_on_screen_changed(scr);                    /* 待机页暂停空闲计时 */
}

/* 当前屏幕不进入空闲待机（待机页、运行页、洗涤完成页、报警弹层可见、管理员页常亮） */
static bool ui_alarm_overlay_is_visible(void);  //前向声明：报警弹层是否正在显示

static bool ui_idle_screen_keeps_awake(lv_obj_t * scr)
{
	if(scr == NULL) return false;
	if(ui_alarm_overlay_is_visible()) return true;     //报警弹层可见时保持常亮
	return scr == g_scr_off || scr == g_scr_running || scr == g_scr_end || scr == g_scr_admin;
}

//重置空闲计时（有输入时调用）

static void ui_idle_reset(void)
{
    if(g_idle_timer == NULL) return;
    if(ui_idle_screen_keeps_awake(lv_scr_act())) return;
    if(g_ui_dormancy_timeout_ms == UI_DORMANCY_DISABLED_MS) return;
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
        if(scr == g_scr_off && g_off_btn_power != NULL && g_group_off != NULL) {
            lv_group_focus_obj(g_off_btn_power);
        }
    }
    else {
        ui_idle_reset();
    }
}

//检测鼠标移动并重置空闲计时

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

//空闲超时：进入待机页

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

/* 仅编码器：设置长按阈值（由 ui_apply_indev_long_press_ms 统一配置） */

static void ui_indev_set_encoder_long_press_ms(uint16_t ms)
{
	lv_indev_t * indev = NULL;
	while((indev = lv_indev_get_next(indev)) != NULL) {
		if(lv_indev_get_type(indev) == LV_INDEV_TYPE_ENCODER) {
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
	/* O3 跟随 mid 同步重定位 */
	if(g_running_o3_img != NULL && g_running_mid != NULL) {
		lv_obj_align_to(g_running_o3_img, g_running_mid, LV_ALIGN_RIGHT_MID, 90, 40);
	}
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

//应用童锁锁定/解锁 UI 与编码器 group 状态
static void running_child_lock_apply_locked(bool locked, bool silent_unlock) //应用童锁锁定/解锁 UI 与编码器 group 状态
{
	if(g_running_child_lock_btn == NULL || g_group_running == NULL) return;

	bool* need_child_lock = &g_ui_child_lock;
	if(locked) {
		*need_child_lock = true;
		running_child_lock_on_enter();  //童锁激活时的进入钩子（预留扩展）
	}
	else {
		*need_child_lock = false;
		if(!silent_unlock) {
			running_child_lock_on_exit();  //童锁解除时的退出钩子（预留扩展）
		}
	}

	g_ui_child_lock = locked;

	if(locked) {
		/* 锁定态：深色圆形底 + 白色图标 */
		lv_obj_set_style_bg_opa(g_running_child_lock_btn, LV_OPA_COVER, LV_PART_MAIN);
		lv_obj_set_style_bg_color(g_running_child_lock_btn, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
		lv_obj_set_style_border_width(g_running_child_lock_btn, 0, LV_PART_MAIN);
		lv_obj_set_style_image_opa(g_running_child_lock_img, LV_OPA_COVER, LV_PART_MAIN);
		if(g_running_lock_blocker != NULL) {
			lv_obj_remove_flag(g_running_lock_blocker, LV_OBJ_FLAG_HIDDEN);
		}
		lv_group_remove_all_objs(g_group_running);
		lv_group_add_obj(g_group_running, g_running_child_lock_btn); /* 童锁：长按 3s 解锁 */
		if(g_running_btn_power != NULL) {
			lv_group_add_obj(g_group_running, g_running_btn_power);   /* 电源：童锁开启时仍可操作 */
		}
		lv_group_focus_obj(g_running_child_lock_btn);
		if(g_running_lock_blocker != NULL) {
			lv_obj_move_foreground(g_running_lock_blocker);
		}
		if(g_running_btn_power != NULL) {
			lv_obj_move_foreground(g_running_btn_power);             /* 电源置于遮罩之上可点击 */
		}
		lv_obj_move_foreground(g_running_child_lock_btn);
	}
	else {
		/* 解锁态：浅色圆形底 + 半透明图标 */
		lv_obj_set_style_bg_opa(g_running_child_lock_btn, LV_OPA_COVER, LV_PART_MAIN);
		lv_obj_set_style_bg_color(g_running_child_lock_btn, lv_color_hex(0x555555), LV_PART_MAIN);
		lv_obj_set_style_border_width(g_running_child_lock_btn, 0, LV_PART_MAIN);
		lv_obj_set_style_image_opa(g_running_child_lock_img, LV_OPA_40, LV_PART_MAIN);
		if(g_running_lock_blocker != NULL) {
			lv_obj_add_flag(g_running_lock_blocker, LV_OBJ_FLAG_HIDDEN);
		}
		if(g_running_btn_back != NULL) {
			lv_group_remove_all_objs(g_group_running);
			lv_group_add_obj(g_group_running, g_running_btn_back);
			if(g_running_btn_runpause != NULL) {
				lv_group_add_obj(g_group_running, g_running_btn_runpause);
			}
			if(g_running_btn_power != NULL) {
				lv_group_add_obj(g_group_running, g_running_btn_power);
			}
			lv_group_add_obj(g_group_running, g_running_child_lock_btn);
		}
		ui_set_encoder_group(g_group_running);
		lv_group_focus_obj(g_running_child_lock_btn);              /* 解锁后焦点留在童锁，不跳到返回/启停 */
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
	lv_group_focus_obj(g_running_child_lock_btn);                  /* 长按触发后焦点保持在童锁 */
}

/* 童锁按钮松开：避免 LONG_PRESSED 后焦点被 LVGL 移到其它控件 */

static void cb_running_child_lock_released(lv_event_t * e)
{
	if(lv_event_get_code(e) != LV_EVENT_RELEASED) return;
	if(lv_scr_act() != g_scr_running) return;
	if(g_running_child_lock_btn == NULL || g_group_running == NULL) return;
	lv_group_focus_obj(g_running_child_lock_btn);
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
		lv_obj_t * top = create_top_bar(root, &g_lbl_clock, NULL, NULL, NULL);
		g_home_btn_runpause = add_encoder_top_btn(top, "启停", 100, NULL); /* 固定中文：启停 */
		lv_obj_add_event_cb(g_home_btn_runpause, cb_home_runpause, LV_EVENT_ALL, g_scr_running);
		lv_obj_add_event_cb(g_home_btn_runpause, cb_runpause_long, LV_EVENT_LONG_PRESSED, g_scr_running);
		g_home_btn_power = add_encoder_top_btn(top, "电源", 180, NULL); /* 固定中文：电源 */
		lv_obj_add_event_cb(g_home_btn_power, cb_home_power_alarm_sim, LV_EVENT_CLICKED, NULL); /* PC：切换 E1 仿真 */
		lv_obj_add_event_cb(g_home_btn_power, cb_power_long, LV_EVENT_LONG_PRESSED, g_scr_running);

		/* --- 语言切换图标按钮（稍后挂到第 4 列图标行） --- */
		LV_IMAGE_DECLARE(language);                        /* 声明语言图标资源 */
		g_btn_home_lang = lv_imgbtn_create(root);          /* 创建可点击的语言 imgbtn */
		lv_imgbtn_set_src(g_btn_home_lang, LV_IMGBTN_STATE_RELEASED, NULL, &language, NULL); /* 常态显示 language 图 */
		lv_obj_remove_flag(g_btn_home_lang, LV_OBJ_FLAG_SCROLLABLE); /* 禁止滚动 */
		lv_obj_add_event_cb(g_btn_home_lang, cb_lang_toggle, LV_EVENT_CLICKED, NULL); /* 点击切换中/英 */

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
		lv_obj_add_flag(g_home_carousel_enc, LV_OBJ_FLAG_CLICKABLE);
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
				lv_obj_add_event_cb(card, mode_card_click, LV_EVENT_CLICKED, (void *)(uintptr_t)(unsigned)i);   //卡片点击事件

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
		for(int i = 0; i < TOTAL_PROGRAMS; i++) {
				lv_obj_t * d = lv_obj_create(dots);
				g_mode_dots[i] = d;
				lv_coord_t ds = (i == g_wheel_sel) ? 13 : 10;
				lv_obj_set_size(d, ds, ds);
				lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, LV_PART_MAIN);
				lv_obj_set_style_bg_color(d, lv_color_hex(i == g_wheel_sel ? COL_TEXT : COL_DIM), LV_PART_MAIN);
				lv_obj_set_style_bg_opa(d, LV_OPA_COVER, LV_PART_MAIN);
				lv_obj_set_style_border_width(d, 0, LV_PART_MAIN);
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

		/* 第 1 列文字：程序时间，如「35s」，由 home_sync_program_labels() 按选中程序填写 */
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

		lv_obj_set_parent(g_btn_home_lang, icon_row);          /* 语言 imgbtn 移到第 4 列图标位 */
		lv_obj_align(g_btn_home_lang, LV_ALIGN_TOP_MID, bar_col4_x, 0); /* 与上方 g_lbl_home_lang 对齐 */

		g_home_btn_admin = lv_imgbtn_create(icon_row);               /* 第 5 列：管理员入口 */
		lv_imgbtn_set_src(g_home_btn_admin, LV_IMGBTN_STATE_RELEASED, NULL, &admin, NULL);
		lv_obj_align(g_home_btn_admin, LV_ALIGN_TOP_MID, bar_col5_x, 0);
		lv_obj_remove_flag(g_home_btn_admin, LV_OBJ_FLAG_SCROLLABLE);
		lv_obj_add_event_cb(g_home_btn_admin, cb_load_admin, LV_EVENT_CLICKED, NULL);

		home_encoder_group_build();
}








//获取指定程序索引的参数表项（时间/温度/价格）
static const ui_program_profile_t * program_profile_get(int32_t idx)  //获取指定程序索引的参数表项（时间/温度/价格）
{
	idx = wheel_mod_total(idx);  //程序索引取模
	return &g_program_profiles[(unsigned)idx];
}

//格式化为首页/程序设置总时长标签（如 24min）

static void program_format_time_label(uint32_t sec, char * buf, size_t buf_sz)
{
	uint32_t min = (sec == 0u) ? 0u : (sec + 59u) / 60u;
	snprintf(buf, buf_sz, "%umin", (unsigned)min);
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
		program_format_time_label(profile->wash_sec, buf, sizeof(buf)); /* 如 24min */
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

/* 刷新支付页金额 label（g_lbl_pay_price），如「¥ 8.00」 */

static void pay_sync_price_label(void)
{
	if(g_lbl_pay_price == NULL) return;                  /* 支付页未构建 */
	const ui_program_profile_t * profile = program_profile_get(g_wheel_sel);
	char buf[16];
	program_format_price_pay(profile->price, buf, sizeof(buf));
	lv_label_set_text(g_lbl_pay_price, buf);             /* 写入金额文字 */
	if(g_lbl_pay_price_alipay != NULL) lv_label_set_text(g_lbl_pay_price_alipay, buf);
	if(g_lbl_pay_price_wechat != NULL) lv_label_set_text(g_lbl_pay_price_wechat, buf);
}

//按支付方式与当前程序刷新支付页二维码与提示布局
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

/* 根据已运行时间与分段配置，决定左侧当前阶段文案 ID */
static ui_str_id_t running_status_id_for_elapsed(const ui_program_run_stages_t * st, uint32_t elapsed)
{
	const uint32_t wash_end = st->wash_sec;
	const uint32_t rinse_end = wash_end + st->rinse_sec;

	if(st->wash_sec > 0u && elapsed < wash_end) {
		return STR_RUN_WASHING;
	}
	if(st->rinse_sec > 0u && elapsed < rinse_end) {
		return STR_RUN_RINSING;
	}
	return STR_RUN_SPINNING;
}

/* 运行页底部：左=当前阶段（洗涤中/漂洗中/脱水中），右=该程序阶段条示意 */

static void running_status_sync_labels(void)
{
	if(g_kuaixi_run_lbl_status == NULL && g_kuaixi_run_lbl_stages == NULL) return;

	int32_t idx = wheel_mod_total(g_wheel_sel);
	const ui_program_run_stages_t * st = &g_program_run_stages[(unsigned)idx];
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
	running_status_sync_labels();                      /* 剩余时间变化 → 切换洗涤/漂洗/脱水文案 */
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

/* PC 仿真运行页水位频率扫频 + 温度初值 */
#if UI_RUNNING_LIVE_PC_SIM
static void running_freq_sim_stop(void)
{
	if(g_running_freq_sim_timer != NULL) {
		lv_timer_delete(g_running_freq_sim_timer);
		g_running_freq_sim_timer = NULL;
	}
}
#endif

//重置倒计时状态：清除暂停、闪烁与定时器
static void running_countdown_reset_all(void)  //重置倒计时状态
{
	g_running_countdown_paused = false;
	running_blink_stop();       //停止暂停时倒计时闪烁定时器并恢复标签可见
	running_countdown_stop();   //停止运行页倒计时定时器
#if UI_RUNNING_LIVE_PC_SIM
	running_freq_sim_stop();    //停止 PC 仿真运行页水位频率扫频定时器
#endif
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
		cycle_abort_run();
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

/* ---------- 运行页实时水位（频率表）+ 温度：数据 ---------- */
#define RUNNING_WATER_PT_CNT     6u
#if UI_RUNNING_LIVE_PC_SIM
/* PC 频率仿真：4350→3500→4350…，每秒 ±10 */
#define RUNNING_FREQ_SIM_MAX_X100   4350u
#define RUNNING_FREQ_SIM_MIN_X100   3500u
#define RUNNING_FREQ_SIM_STEP       10u
#define RUNNING_FREQ_SIM_PERIOD_MS  1000u
#endif

typedef struct {
	uint16_t freq_x100;
	uint16_t mm;
	int16_t liters;
	const char * stage;
} running_water_pt_t;

static const running_water_pt_t g_running_water_tbl[RUNNING_WATER_PT_CNT] = {
	{ 4282,   0,  0, "复位水位" },
	{ 4085,  92, 14, "1水位" },
	{ 4020, 120, 20, "2水位" },
	{ 3970, 142, 26, "3水位" },
	{ 3920, 164, 32, "4水位" },
	/* 溢水水量：按 4水位→溢水 mm 比例外推 ≈32*326/164 */
	{ 3600, 326, 64, "溢水水位" },
};

static uint16_t g_running_freq_x100 = 4282; /* 运行页实时频率 4282为初始默认占位值*/
static int16_t g_running_live_temp_c = 25;  /* 运行页实时温度 25为初始默认占位值*/

#if UI_RUNNING_LIVE_PC_SIM
static int8_t g_running_freq_sim_dir = -1; /* -1 递减，+1 递增 */
#endif

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
		if(g_cycle_active && g_cycle_ui_state == CYCLE_UI_RUNNING) {
			cycle_finish_current_run();
			return;
		}
		ui_send_beep_seq(7);
		ui_screen_load(g_scr_end);
	}
}

/* PC 仿真运行页水位频率扫频定时器回调，实机不使用*/
#if UI_RUNNING_LIVE_PC_SIM
/* PC：频率在 4350↔3500 间每秒往返步进，驱动水位分档变化 */
static void cb_running_freq_sim(lv_timer_t * t)
{
	(void)t;
	if(g_running_countdown_paused) return;

	if(g_running_freq_sim_dir < 0) {
		if(g_running_freq_x100 <= RUNNING_FREQ_SIM_MIN_X100 + RUNNING_FREQ_SIM_STEP) {
			g_running_freq_x100 = RUNNING_FREQ_SIM_MIN_X100;
			g_running_freq_sim_dir = 1;
		} else {
			g_running_freq_x100 = (uint16_t)(g_running_freq_x100 - RUNNING_FREQ_SIM_STEP);
		}
	} else {
		if(g_running_freq_x100 + RUNNING_FREQ_SIM_STEP >= RUNNING_FREQ_SIM_MAX_X100) {
			g_running_freq_x100 = RUNNING_FREQ_SIM_MAX_X100;
			g_running_freq_sim_dir = -1;
		} else {
			g_running_freq_x100 = (uint16_t)(g_running_freq_x100 + RUNNING_FREQ_SIM_STEP);
		}
	}
	running_live_params_sync();
}

static void running_freq_sim_arm(void)
{
	if(g_running_freq_sim_timer != NULL) return;
	g_running_freq_sim_timer = lv_timer_create(cb_running_freq_sim, RUNNING_FREQ_SIM_PERIOD_MS, NULL);
}
#endif

//若剩余时间>0 则启动分钟倒计时定时器
static void running_countdown_arm(void)
{
	if(g_running_countdown_timer != NULL) return;
	if(g_running_remain_sec == 0u) return;
	g_running_countdown_timer = lv_timer_create(cb_running_countdown, RUNNING_COUNTDOWN_PERIOD_MS, NULL);
}

//进入运行页时按程序时长初始化并开始倒计时
static void running_countdown_start(void)
{
	running_countdown_reset_all();
	g_running_remain_sec = running_program_total_sec(g_wheel_sel);
	if(g_running_remain_sec == 0u) {
		g_running_remain_sec = 60u;
	}
#if UI_RUNNING_LIVE_PC_SIM
	g_running_freq_x100 = RUNNING_FREQ_SIM_MAX_X100;
	g_running_freq_sim_dir = -1;
	g_running_live_temp_c = 25;
#endif
	running_live_params_sync();
	running_countdown_update_label();
	running_countdown_arm();
#if UI_RUNNING_LIVE_PC_SIM
	running_freq_sim_arm();
#endif
}

/* ---------- 运行页实时水位（频率表）+ 温度：lookup / sync / API ---------- */
static const running_water_pt_t * running_water_nearest(uint16_t freq_x100)
{
	const running_water_pt_t * best = &g_running_water_tbl[0];
	uint32_t best_d = 0xFFFFFFFFu;
	for(uint8_t i = 0; i < RUNNING_WATER_PT_CNT; i++) {
		const running_water_pt_t * p = &g_running_water_tbl[i];
		uint32_t d = (freq_x100 > p->freq_x100)
			? (uint32_t)(freq_x100 - p->freq_x100)
			: (uint32_t)(p->freq_x100 - freq_x100);
		if(d < best_d || (d == best_d && p->freq_x100 < best->freq_x100)) {
			best_d = d;
			best = p;
		}
	}
	return best;
}

/* 水位/水量：档间线性插值（输出 ×10，显示一位小数）；端点外钳位 */
static void running_water_interp(uint16_t freq_x100, int * mm_x10_out, int * liters_x10_out)
{
	for(uint8_t i = 0; i < RUNNING_WATER_PT_CNT; i++) {
		if(g_running_water_tbl[i].freq_x100 == freq_x100) {
			*mm_x10_out = (int)g_running_water_tbl[i].mm * 10;
			*liters_x10_out = (int)g_running_water_tbl[i].liters * 10;
			return;
		}
	}
	const running_water_pt_t * hi = &g_running_water_tbl[0];
	const running_water_pt_t * lo = &g_running_water_tbl[RUNNING_WATER_PT_CNT - 1u];
	if(freq_x100 >= hi->freq_x100) {
		*mm_x10_out = (int)hi->mm * 10;
		*liters_x10_out = (int)hi->liters * 10;
		return;
	}
	if(freq_x100 <= lo->freq_x100) {
		*mm_x10_out = (int)lo->mm * 10;
		*liters_x10_out = (int)lo->liters * 10;
		return;
	}
	for(uint8_t i = 0; i + 1u < RUNNING_WATER_PT_CNT; i++) {
		const running_water_pt_t * a = &g_running_water_tbl[i];
		const running_water_pt_t * b = &g_running_water_tbl[i + 1u];
		if(freq_x100 < a->freq_x100 && freq_x100 > b->freq_x100) {
			int32_t df = (int32_t)a->freq_x100 - (int32_t)b->freq_x100;
			int32_t t = (int32_t)a->freq_x100 - (int32_t)freq_x100;
			*mm_x10_out = (int)((((int32_t)a->mm * 10) * (df - t) + ((int32_t)b->mm * 10) * t + df / 2) / df);
			*liters_x10_out = (int)((((int32_t)a->liters * 10) * (df - t) + ((int32_t)b->liters * 10) * t + df / 2) / df);
			return;
		}
	}
	*mm_x10_out = (int)hi->mm * 10;
	*liters_x10_out = (int)hi->liters * 10;
}

/* 运行页实时水位频率 + 温度：同步到 UI */
static void running_live_params_sync(void)
{
	const running_water_pt_t * stage_pt = running_water_nearest(g_running_freq_x100);
	int mm_x10 = 0;
	int liters_x10 = 0;
	char buf[32];
	running_water_interp(g_running_freq_x100, &mm_x10, &liters_x10);
	if(g_running_lbl_freq != NULL) {
		lv_snprintf(buf, sizeof(buf), "频率：%u.%02u KHz",
			(unsigned)(g_running_freq_x100 / 100u),
			(unsigned)(g_running_freq_x100 % 100u));
		lv_label_set_text(g_running_lbl_freq, buf);
	}
	if(g_running_lbl_water_mm != NULL) {
		lv_snprintf(buf, sizeof(buf), "水位：%d.%d mm", mm_x10 / 10, (mm_x10 < 0 ? -mm_x10 : mm_x10) % 10);
		lv_label_set_text(g_running_lbl_water_mm, buf);
	}
	if(g_running_lbl_water_L != NULL) {
		lv_snprintf(buf, sizeof(buf), "水量：%d.%d L", liters_x10 / 10, (liters_x10 < 0 ? -liters_x10 : liters_x10) % 10);
		lv_label_set_text(g_running_lbl_water_L, buf);
	}
	if(g_running_lbl_water_stage != NULL) {
		lv_snprintf(buf, sizeof(buf), "分档：%s", stage_pt->stage);
		lv_label_set_text(g_running_lbl_water_stage, buf);
	}
	if(g_running_lbl_temp_live != NULL) {
		lv_snprintf(buf, sizeof(buf), "温度：%d ℃", (int)g_running_live_temp_c);
		lv_label_set_text(g_running_lbl_temp_live, buf);
	}
}

/* 实机同步运行时水位频率 */
void ui_running_freq_x100_set(uint16_t v)
{
	g_running_freq_x100 = v;
	running_live_params_sync();
}

/* 实机同步运行时温度 */
void ui_running_live_temp_set(int16_t temp_c)
{
	g_running_live_temp_c = temp_c;
	running_live_params_sync();
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
		lv_obj_t * top = create_top_bar(root, &g_lbl_clock_running, g_scr_home, g_group_running,
			&g_running_btn_back);
		g_running_btn_runpause = add_encoder_top_btn(top, "启停", 100, g_group_running);
		lv_obj_add_event_cb(g_running_btn_runpause, cb_running_runpause, LV_EVENT_CLICKED, NULL);
		//lv_obj_add_event_cb(g_running_btn_runpause, cb_runpause, LV_EVENT_CLICKED, NULL);
		lv_obj_add_event_cb(g_running_btn_runpause, cb_runpause_long, LV_EVENT_LONG_PRESSED, NULL);
		g_running_btn_power = add_encoder_top_btn(top, "电源", 180, g_group_running);
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

		/* 倒计时 label：0:MM，由 running_countdown_update_label 更新 */
		lv_obj_t * time_txt = lv_label_create(mid);
		g_running_time_label = time_txt;
		lv_label_set_text(time_txt, "0:00");
		lv_obj_set_style_text_color(time_txt, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(time_txt, s_font_sc_125);

		/* 童锁图标按钮 */
		lv_obj_t * child_lock = lv_button_create(root);
		g_running_child_lock_btn = child_lock;
		lv_obj_set_size(child_lock, 49, 49);
		lv_obj_set_style_radius(child_lock, LV_RADIUS_CIRCLE, LV_PART_MAIN);
		lv_obj_set_style_pad_all(child_lock, 0, LV_PART_MAIN);
		lv_obj_set_style_shadow_width(child_lock, 0, LV_PART_MAIN);
		/* 默认解锁态：浅色圆形底 */
		lv_obj_set_style_bg_opa(child_lock, LV_OPA_COVER, LV_PART_MAIN);
		lv_obj_set_style_bg_color(child_lock, lv_color_hex(0x555555), LV_PART_MAIN);
		lv_obj_set_style_border_width(child_lock, 0, LV_PART_MAIN);
		lv_obj_remove_flag(child_lock, LV_OBJ_FLAG_SCROLLABLE);
		lv_obj_add_event_cb(child_lock, cb_running_child_lock_long, LV_EVENT_LONG_PRESSED, NULL);
		lv_obj_add_event_cb(child_lock, cb_running_child_lock_released, LV_EVENT_RELEASED, NULL);

		lv_obj_t * lock_img = lv_image_create(child_lock);
		g_running_child_lock_img = lock_img;
		lv_image_set_src(lock_img, &child_lock_logo);
		/* 默认解锁态：40% 透明度 */
		lv_obj_set_style_image_opa(lock_img, LV_OPA_40, LV_PART_MAIN);
		lv_obj_center(lock_img);
		ui_encoder_group_add(g_group_running, child_lock);

		/* O3 图标：跟随童锁右侧 */
		g_running_o3_img = lv_image_create(root);
		lv_image_set_src(g_running_o3_img, &O3_logo);
		lv_obj_align_to(g_running_o3_img, g_running_mid, LV_ALIGN_RIGHT_MID, 90, 40);
		if(!ui_ozone_get()) lv_obj_add_flag(g_running_o3_img, LV_OBJ_FLAG_HIDDEN);

		/* 左侧：实时频率/水位/水量/分档 + 温度（SC_30） */
		{
			lv_obj_t * left = lv_obj_create(root);
			lv_obj_set_size(left, 280, LV_SIZE_CONTENT);
			lv_obj_align(left, LV_ALIGN_TOP_LEFT, 60, 110);
			lv_obj_set_style_bg_opa(left, LV_OPA_TRANSP, LV_PART_MAIN);
			lv_obj_set_style_border_width(left, 0, LV_PART_MAIN);
			lv_obj_set_style_pad_all(left, 0, LV_PART_MAIN);
			lv_obj_set_style_pad_row(left, 8, LV_PART_MAIN);
			lv_obj_set_flex_flow(left, LV_FLEX_FLOW_COLUMN);
			lv_obj_set_flex_align(left, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
			lv_obj_t ** lbls[5] = {
				&g_running_lbl_freq, &g_running_lbl_water_mm, &g_running_lbl_water_L,
				&g_running_lbl_water_stage, &g_running_lbl_temp_live
			};
			for(int i = 0; i < 5; i++) {
				lv_obj_t * l = lv_label_create(left);
				*lbls[i] = l;
				lv_obj_set_style_text_color(l, lv_color_hex(COL_TEXT), LV_PART_MAIN);
				ui_set_obj_font(l, s_font_sc_30);
			}
			/* 温度与水位分档之间空一行 */
			lv_obj_set_style_margin_top(g_running_lbl_temp_live, 50, LV_PART_MAIN);
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

		/* 左下：当前阶段（洗涤中/漂洗中/脱水中），由 running_status_sync_labels 按剩余时间刷新 */
		lv_obj_t * st = lv_label_create(bot);
		lv_obj_set_style_text_color(st, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(st, s_font_sc_35);
		lv_obj_align(st, LV_ALIGN_LEFT_MID, 60, 0);
		g_kuaixi_run_lbl_status = st;

		/* 右下：程序阶段条（三阶段/仅脱水/漂洗+脱水），随程序固定 */
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

		running_child_lock_align_btn();  //将童锁按钮对齐到运行页中间栏右侧
		cycle_fault_overlay_create(root, &g_cycle_panel_fault_run, &g_cycle_lbl_fault_alt_run);
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

		lv_obj_t * top = create_top_bar(root, &g_lbl_clock_end, g_scr_home, g_group_end, NULL);
		lv_obj_t * btn_runpause = add_encoder_top_btn(top, "启停", 100, g_group_end);
		lv_obj_add_event_cb(btn_runpause, cb_load_screen, LV_EVENT_CLICKED, g_scr_home);
		lv_obj_add_event_cb(btn_runpause, cb_runpause, LV_EVENT_CLICKED, g_scr_home);
		lv_obj_add_event_cb(btn_runpause, cb_runpause_long, LV_EVENT_LONG_PRESSED, g_scr_home);
		lv_obj_t * btn_power = add_encoder_top_btn(top, "电源", 180, g_group_end);
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

		LV_IMAGE_DECLARE(end);                             /* 结束图标 */
		lv_obj_t * img_end = lv_image_create(center);
		lv_image_set_src(img_end, &end);

		/* 主标题：中「洗涤完成」/ 英「Wash Complete」（STR_END_TITLE） */
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












/* PC 仿真液位；量产时改为寄存器读取 */
static uint8_t g_fluid_softener_pct = 50u;  //柔顺剂液位百分比
static uint8_t g_fluid_detergent_pct = 50u; //洗涤剂液位百分比

//PC仿真缺液/故障，实机需根据情况删掉或改为读寄存器
#define UI_FLUID_LEVEL_LOW_THRESHOLD  20u   //液位低于此阈值视为缺液（0=空 100=满）
#define UI_ALARM_FAULT_COUNT          15u   //故障码 E1..E15
#define UI_ALARM_FAULT_STRIDE         3u    //每种故障占 3 个 STR id：TITLE/LINE1/LINE2
#define UI_ALARM_ROTATE_PERIOD_MS     3000u //轮播间隔 3 秒
#define UI_ALARM_ROTATE_MAX           17u   //最多 15 故障 + 2 缺液
#define UI_ALARM_FAULT_CODE_BLINK_MS  500u  //故障码 Ex 亮/灭各 0.5s
#define UI_ALARM_FAULT_CONTENT_X_SHIFT 200  //仅故障码子页内部元素右移（缺液页不用）
#define COL_ALARM_PHONE               0x3399FFu //故障页电话高亮色

typedef enum {
	ALARM_PANEL_NONE = 0,
	ALARM_PANEL_FAULT_E1 = 1,
	ALARM_PANEL_FAULT_E14 = 14,
	ALARM_PANEL_FAULT_E15 = 15,
	ALARM_PANEL_FLUID_SOFTENER,
	ALARM_PANEL_FLUID_DETERGENT,
} alarm_panel_t;

typedef struct {
	bool active;            //当前条件是否成立
	uint32_t trigger_tick;  //上升沿时刻；清除时置 0
} ui_alarm_trigger_t;

typedef struct {
	lv_obj_t * panel;       //故障子面板根，默认隐藏
	lv_obj_t * lbl_code;    //故障码 E1..E14
	lv_obj_t * lbl_title;   //标题
	lv_obj_t * lbl_line1;   //说明1（故障说明首行；Y 固定）
	lv_obj_t * lbl_line2;   //说明2（故障说明次行；单行故障时隐藏）
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
static lv_timer_t * g_alarm_fault_code_blink_timer; //故障码 Ex 0.5s 闪烁定时器
static uint8_t g_alarm_fault_code_blink_idx = 0xFFu;  //当前闪烁的故障索引；0xFF=无
static alarm_panel_t g_alarm_rotate_cur_panel;    //当前显示的 panel
static bool g_alarm_user_dismissed;               //用户手动返回后暂不再自动弹出
static bool g_alarm_overlay_open;               //弹层是否处于显示状态（含 encoder 切换）

static uint8_t g_home_power_sim_step;             //PC：电源键循环仿真步进

static const char * const g_alarm_fault_codes[UI_ALARM_FAULT_COUNT] = {
	"E1", "E2", "E3", "E4", "E5", "E6", "E7",
	"E8", "E9", "E10", "E11", "E12", "E13", "E14", "E15"
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

//停止故障码 Ex 闪烁，并恢复全部故障码 label 为可见
static void alarm_fault_code_blink_stop(void)
{
	if(g_alarm_fault_code_blink_timer != NULL) {                           //暂停闪烁定时器
		lv_timer_pause(g_alarm_fault_code_blink_timer);
	}
	for(uint8_t i = 0; i < UI_ALARM_FAULT_COUNT; i++) {                    //恢复所有 Ex 可见
		if(g_alarm_fault_ui[i].lbl_code != NULL) {
			lv_obj_remove_flag(g_alarm_fault_ui[i].lbl_code, LV_OBJ_FLAG_HIDDEN);
		}
	}
	g_alarm_fault_code_blink_idx = 0xFFu;                                //无闪烁目标
}

//故障码 Ex 亮灭切换（500ms 一次）
static void alarm_fault_code_blink_cb(lv_timer_t * t)
{
	(void)t;                                                               //未使用
	if(g_alarm_fault_code_blink_idx >= UI_ALARM_FAULT_COUNT) return;       //无有效索引
	lv_obj_t * code = g_alarm_fault_ui[g_alarm_fault_code_blink_idx].lbl_code;
	if(code == NULL) return;
	if(lv_obj_has_flag(code, LV_OBJ_FLAG_HIDDEN)) {                        //当前灭 → 亮
		lv_obj_remove_flag(code, LV_OBJ_FLAG_HIDDEN);
	}
	else {                                                                 //当前亮 → 灭
		lv_obj_add_flag(code, LV_OBJ_FLAG_HIDDEN);
	}
}

//进入故障子页：故障码 Ex 以 0.5s 亮、0.5s 灭闪烁
static void alarm_fault_code_blink_start(uint8_t fault_idx)
{
	if(fault_idx >= UI_ALARM_FAULT_COUNT) return;                          //越界保护
	alarm_fault_code_blink_stop();                                         //先停旧闪烁并恢复 Ex 可见
	g_alarm_fault_code_blink_idx = fault_idx;                              //记录当前 Ex
	if(g_alarm_fault_ui[fault_idx].lbl_code != NULL) {                     //初始为亮
		lv_obj_remove_flag(g_alarm_fault_ui[fault_idx].lbl_code, LV_OBJ_FLAG_HIDDEN);
	}
	if(g_alarm_fault_code_blink_timer == NULL) {                           //首次创建 500ms 定时器
		g_alarm_fault_code_blink_timer = lv_timer_create(
			alarm_fault_code_blink_cb, UI_ALARM_FAULT_CODE_BLINK_MS, NULL);
	}
	else {
		lv_timer_reset(g_alarm_fault_code_blink_timer);                    //重置相位
		lv_timer_resume(g_alarm_fault_code_blink_timer);                   //继续闪烁
	}
}

//显示指定子面板并刷新缺液内容
static void alarm_panel_show(alarm_panel_t panel)
{
	alarm_panel_hide_all();                                                //先全部隐藏
	g_alarm_rotate_cur_panel = panel;                                      //记录当前 panel

	if(panel >= ALARM_PANEL_FAULT_E1 && panel <= ALARM_PANEL_FAULT_E15) {   //故障码子页
		uint8_t idx = (uint8_t)(panel - ALARM_PANEL_FAULT_E1);             //转 fault 索引
		if(g_alarm_fault_ui[idx].panel != NULL) {
			lv_obj_remove_flag(g_alarm_fault_ui[idx].panel, LV_OBJ_FLAG_HIDDEN); //显示目标
		}
		alarm_fault_code_blink_start(idx);                                 //Ex 0.5s 闪烁
		alarm_fault_panel_relayout_content(idx);                           //单行/双行说明 + 页脚重排
	}
	else if(panel == ALARM_PANEL_FLUID_SOFTENER || panel == ALARM_PANEL_FLUID_DETERGENT) {
		alarm_fault_code_blink_stop();                                     //缺液页不闪 Ex
		if(g_alarm_panel_fluid != NULL) {
			lv_obj_remove_flag(g_alarm_panel_fluid, LV_OBJ_FLAG_HIDDEN); //显示缺液 panel
		}
	}
	else {
		alarm_fault_code_blink_stop();                                     //其它情况停止闪烁
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
	lv_obj_t * scr = lv_screen_active();                                   //当前底层 screen
	if(scr == NULL) return;
	if(scr == g_scr_off) ui_set_encoder_group(g_group_off);
	else if(scr == g_scr_home) ui_set_encoder_group(g_group_home);
	else if(scr == g_scr_running) ui_set_encoder_group(g_group_running);
	else if(scr == g_scr_end) ui_set_encoder_group(g_group_end);
	else if(scr == g_scr_pay) ui_set_encoder_group(g_group_pay);
	else if(scr == g_scr_pay_done) ui_set_encoder_group(g_group_pay_done);
	else if(scr == g_scr_admin) ui_set_encoder_group(g_group_admin);
	else ui_set_encoder_group(g_ui_group);                                 //默认组
}

//隐藏报警弹层并恢复底层 encoder 组
static void alarm_overlay_hide(void)
{
	if(g_alarm_overlay == NULL) return;                                    //未构建
	lv_obj_add_flag(g_alarm_overlay, LV_OBJ_FLAG_HIDDEN);                  //隐藏弹层
	alarm_rotate_timer_stop();                                             //停止轮播
	alarm_fault_code_blink_stop();                                         //停止 Ex 闪烁
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
	ui_set_encoder_group(g_group_alarm);                                   //编码器切到报警按钮
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

//PC 仿真：主页电源键短按循环触发 E1 / 缺液，便于验证轮播
static void cb_home_power_alarm_sim(lv_event_t * e)
{
	(void)e;                                                               //未使用
	switch(g_home_power_sim_step % 3u) {                                   //三步循环
	case 0u:
		g_sim_fault_e[0] = !g_sim_fault_e[0];                              //切换 E1
		break;
	case 1u:
		g_fluid_detergent_pct = ui_is_detergent_low() ? 50u : 10u;         //切换洗涤剂缺液
		break;
	default:
		g_sim_fault_e[0] = false;                                          //清除 E1
		g_fluid_detergent_pct = 50u;                                       //清除缺液
		break;
	}
	g_home_power_sim_step++;                                               //步进+1
	g_alarm_user_dismissed = false;                                          //允许再次弹出
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

//构建单个故障码子面板（E1..E14 共用布局）
static void build_alarm_fault_panel(lv_obj_t * root, uint8_t fault_idx)
{
	ui_alarm_fault_ui_t * ui = &g_alarm_fault_ui[fault_idx];               //取 UI 槽位
	const lv_coord_t body_y = (lv_coord_t)(UI_FIXED_H * 10 / 100);         //与管理员子页对齐

	ui->panel = lv_obj_create(root);                                       //子面板根
	lv_obj_set_size(ui->panel, 1600, 400);                                 //1600×400 区域保持居中
	lv_obj_align(ui->panel, LV_ALIGN_TOP_MID, 0, body_y);                  //panel 不整体平移
	lv_obj_set_style_bg_opa(ui->panel, LV_OPA_TRANSP, LV_PART_MAIN);       //透明底
	lv_obj_set_style_border_width(ui->panel, 0, LV_PART_MAIN);             //无边框
	lv_obj_set_style_pad_all(ui->panel, 0, LV_PART_MAIN);                  //无内边距
	lv_obj_set_style_layout(ui->panel, LV_LAYOUT_NONE, LV_PART_MAIN);      //绝对布局
	lv_obj_add_flag(ui->panel, LV_OBJ_FLAG_HIDDEN);                        //默认隐藏

	ui->lbl_code = lv_label_create(ui->panel);                             //故障码
	lv_label_set_text(ui->lbl_code, g_alarm_fault_codes[fault_idx]);         //E1..E14
	lv_obj_set_style_text_color(ui->lbl_code, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(ui->lbl_code, s_font_sc_50);                           //大字码
	lv_obj_set_pos(ui->lbl_code, 120 + UI_ALARM_FAULT_CONTENT_X_SHIFT, 100); //仅故障页内部右移

	ui->lbl_title = lv_label_create(ui->panel);                            //标题
	ui_lang_bind_label(ui->lbl_title, alarm_fault_str_id(fault_idx, 0));   //STR_TITLE
	lv_obj_set_style_text_color(ui->lbl_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(ui->lbl_title, s_font_sc_30);
	lv_obj_align_to(ui->lbl_title, ui->lbl_code, LV_ALIGN_OUT_RIGHT_MID, 60, 0); //与 E1 同一水平线（垂直居中）

	ui->lbl_line1 = lv_label_create(ui->panel);                            //说明1
	ui_lang_bind_label(ui->lbl_line1, alarm_fault_str_id(fault_idx, 1));
	lv_obj_set_style_text_color(ui->lbl_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(ui->lbl_line1, s_font_sc_30);
	lv_obj_align_to(ui->lbl_line1, ui->lbl_title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 60); //Y 固定，不随单行/双行变化

	ui->lbl_line2 = lv_label_create(ui->panel);                            //说明2（单行故障时隐藏）
	ui_lang_bind_label(ui->lbl_line2, alarm_fault_str_id(fault_idx, 2));
	lv_obj_set_style_text_color(ui->lbl_line2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(ui->lbl_line2, s_font_sc_30);

	ui->lbl_line3 = lv_label_create(ui->panel);                            //拨打引导
	ui_lang_bind_label(ui->lbl_line3, STR_ALARM_FAULT_CALL);
	lv_obj_set_style_text_color(ui->lbl_line3, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(ui->lbl_line3, s_font_sc_30);

	ui->lbl_phone = lv_label_create(ui->panel);                            //电话
	ui_lang_bind_label(ui->lbl_phone, STR_ALARM_FAULT_PHONE);
	lv_obj_set_style_text_color(ui->lbl_phone, lv_color_hex(COL_ALARM_PHONE), LV_PART_MAIN);
	ui_set_obj_font(ui->lbl_phone, s_font_sc_30);

	ui->lbl_line4 = lv_label_create(ui->panel);                            //售后句
	ui_lang_bind_label(ui->lbl_line4, STR_ALARM_FAULT_SERVICE);
	lv_obj_set_style_text_color(ui->lbl_line4, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(ui->lbl_line4, s_font_sc_30);

	alarm_fault_panel_relayout_content(fault_idx);                           //按文案排说明2 与页脚
}

//故障说明2 是否为空（单行故障）
static bool alarm_fault_line2_empty(uint8_t fault_idx)
{
	const char * text = ui_translation(alarm_fault_str_id(fault_idx, 2));  //取当前语言 LINE2
	if(text == NULL || text[0] == '\0') return true;                       //空串
	for(const char * p = text; *p != '\0'; p++) {                          //跳过空白
		if(*p != ' ' && *p != '\t' && *p != '\n' && *p != '\r') return false;
	}
	return true;
}

//说明2 显隐 + 页脚电话/售后重排（说明1 Y 不在此函数中改动）
static void alarm_fault_panel_relayout_content(uint8_t fault_idx)
{
	ui_alarm_fault_ui_t * ui = &g_alarm_fault_ui[fault_idx];               //取 UI 槽位
	if(ui->panel == NULL || ui->lbl_line1 == NULL || ui->lbl_line2 == NULL ||
	   ui->lbl_line3 == NULL || ui->lbl_phone == NULL || ui->lbl_line4 == NULL) {
		return;
	}

	lv_obj_t * call_anchor;                                                //拨打句锚点（line1 或 line2）
	const lv_coord_t row_gap = 12;                                         //说明行间距
	const lv_coord_t call_gap = 8;                                         //拨打句与电话换行间距

	if(alarm_fault_line2_empty(fault_idx)) {                               //单行故障
		lv_obj_add_flag(ui->lbl_line2, LV_OBJ_FLAG_HIDDEN);                //隐藏说明2
		call_anchor = ui->lbl_line1;                                       //页脚接说明1
	}
	else {
		lv_obj_remove_flag(ui->lbl_line2, LV_OBJ_FLAG_HIDDEN);             //显示说明2
		lv_obj_align_to(ui->lbl_line2, ui->lbl_line1, LV_ALIGN_OUT_BOTTOM_LEFT, 0, row_gap);
		call_anchor = ui->lbl_line2;                                       //页脚接说明2
	}

	lv_obj_align_to(ui->lbl_line3, call_anchor, LV_ALIGN_OUT_BOTTOM_LEFT, 0, row_gap); //拨打引导

	lv_obj_update_layout(ui->panel);                                       //刷新尺寸后再量宽
	lv_coord_t panel_w = lv_obj_get_width(ui->panel);                      //panel 宽度
	lv_coord_t call_x = lv_obj_get_x(ui->lbl_line3);                       //拨打句起点 X
	lv_coord_t call_w = lv_obj_get_width(ui->lbl_line3);                   //拨打句文本宽
	lv_coord_t phone_w = lv_obj_get_width(ui->lbl_phone);                  //电话文本宽
	const lv_coord_t gap = 8;                                              //同行间距
	const lv_coord_t margin = 16;                                          //右侧留白
	lv_coord_t avail = panel_w - call_x - margin;                          //拨打句以右可用宽

	if(call_w + gap + phone_w <= avail) {                                  //一行放得下：电话紧跟拨打句
		lv_obj_align_to(ui->lbl_phone, ui->lbl_line3, LV_ALIGN_OUT_RIGHT_MID, gap, 0);
		lv_obj_align_to(ui->lbl_line4, ui->lbl_line3, LV_ALIGN_OUT_BOTTOM_LEFT, 0, row_gap);
	}
	else {                                                                 //英文等较长：电话单独一行
		lv_obj_align_to(ui->lbl_phone, ui->lbl_line3, LV_ALIGN_OUT_BOTTOM_LEFT, 0, call_gap);
		lv_obj_align_to(ui->lbl_line4, ui->lbl_phone, LV_ALIGN_OUT_BOTTOM_LEFT, 0, row_gap);
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

	lv_obj_t * top = create_top_bar(g_alarm_overlay, &g_lbl_clock_alarm, NULL, NULL, NULL); //顶栏+状态栏

	LV_IMAGE_DECLARE(back);
	g_alarm_btn_back = lv_imgbtn_create(top);                              //返回
	lv_imgbtn_set_src(g_alarm_btn_back, LV_IMGBTN_STATE_RELEASED, NULL, &back, NULL);
	lv_obj_align(g_alarm_btn_back, LV_ALIGN_LEFT_MID, 20, 0);
	lv_obj_remove_flag(g_alarm_btn_back, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_add_event_cb(g_alarm_btn_back, cb_alarm_back, LV_EVENT_CLICKED, NULL);
	ui_encoder_group_add(g_group_alarm, g_alarm_btn_back);

	g_alarm_btn_runpause = add_encoder_top_btn(top, "启停", 100, g_group_alarm); //启停
	lv_obj_add_event_cb(g_alarm_btn_runpause, cb_alarm_runpause, LV_EVENT_CLICKED, NULL);

	g_alarm_btn_power = add_encoder_top_btn(top, "电源", 180, g_group_alarm);    //电源
	lv_obj_add_event_cb(g_alarm_btn_power, cb_alarm_power, LV_EVENT_CLICKED, NULL);

	g_alarm_img_bar = lv_image_create(g_alarm_overlay);                      //底栏 bar_01
	lv_image_set_src(g_alarm_img_bar, &bar_01);
	lv_obj_align(g_alarm_img_bar, LV_ALIGN_BOTTOM_MID, 0, -80);

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

static const char * program_admin_temp_str(int8_t idx)
{
	switch(idx) {
	case 0: return "COLD";
	case 1: return "30℃";
	case 2: return "40℃";
	case 3: return "60℃";
	case 4: return "90℃";
	default: return "--";
	}
}

static int program_admin_spin_rpm_to_roller(int16_t rpm)
{
	switch(rpm) {
	case 1200: return 0;
	case 800:  return 1;
	case 600:  return 2;
	case 400:  return 3;
	case 0:    return 4;
	default:   return 1;
	}
}

static int16_t program_admin_roller_to_spin_rpm(uint32_t sel)
{
	static const int16_t tbl[] = { 1200, 800, 600, 400, 0 };
	if(sel >= sizeof(tbl) / sizeof(tbl[0])) return 800;
	return tbl[sel];
}

/* 配置层时长 → 运行层秒数（PROG_ADMIN_DEMO_SEC=0 时 ×60） */
static uint32_t program_admin_cfg_to_sec(uint16_t cfg_val)
{
#if PROG_ADMIN_DEMO_SEC
	return cfg_val;
#else
	return (uint32_t)cfg_val * 60u;
#endif
}

/* 根据程序设置 cfg 计算各阶段秒数；总时长 = 洗涤 + 漂洗时长×漂洗次数 + 脱水 */

static void program_admin_calc_stages(const ui_program_admin_t * c,
	uint32_t * wash_out, uint32_t * rinse_total_out, uint32_t * spin_out)
{
	uint32_t wash = 0;
	uint32_t rinse_total = 0;
	uint32_t spin = 0;

	if(c->cap & PROG_CAP_WASH) {
		wash = program_admin_cfg_to_sec(c->wash_min);
	}
	if((c->cap & PROG_CAP_RINSE_DUR) && (c->cap & PROG_CAP_RINSE_CNT) && c->rinse_cnt >= 0) {
		rinse_total = program_admin_cfg_to_sec(c->rinse_min) * (uint32_t)c->rinse_cnt;
	}
	if(c->cap & PROG_CAP_SPIN_DUR) {
		spin = program_admin_cfg_to_sec(c->spin_min);
	}

	if(wash_out != NULL) *wash_out = wash;
	if(rinse_total_out != NULL) *rinse_total_out = rinse_total;
	if(spin_out != NULL) *spin_out = spin;
}

static ui_str_id_t program_admin_stages_bar_id(uint32_t wash, uint32_t rinse_total)
{
	if(wash == 0u && rinse_total == 0u) {
		return STR_RUN_STAGES_SPIN_ONLY;
	}
	if(wash == 0u) {
		return STR_RUN_STAGES_RINSE_SPIN;
	}
	return STR_RUN_STAGES;
}

/* 写入表2.4 程序时限表初值到 g_prog_cfg_factory[]（时长字段为分钟）。 */

static void program_admin_init_factory(void)
{
	static const ui_program_admin_t factory[TOTAL_PROGRAMS] = {
		/* 0 大物：总42min | 洗12(进水3+洗涤9) | 漂11×2 | 脱8(排水1+脱水7) | 800rpm | 40℃ */
		{ 9, 12, 11, 8,  2, 800,  2,  true,
		  PROG_CAP_PRICE | PROG_CAP_WASH | PROG_CAP_RINSE_DUR | PROG_CAP_SPIN_DUR |
		  PROG_CAP_RINSE_CNT | PROG_CAP_SPIN_RPM | PROG_CAP_TEMP | PROG_CAP_WATER },
		/* 1 单脱水：总9min | 脱9(排水1+脱水7+抖散1) | 800rpm */
		{ 5,  0,  0, 9, -1, 800, -1, false,
		  PROG_CAP_PRICE | PROG_CAP_SPIN_DUR | PROG_CAP_SPIN_RPM },
		/* 2 标准洗：总35min | 洗10(进水3+洗涤7) | 漂8×2 | 脱9(排水1+脱水7+抖散1) | 800rpm | 30℃ */
		{ 8, 10,  8, 9,  2, 800,  1,  true,
		  PROG_CAP_PRICE | PROG_CAP_WASH | PROG_CAP_RINSE_DUR | PROG_CAP_SPIN_DUR |
		  PROG_CAP_RINSE_CNT | PROG_CAP_SPIN_RPM | PROG_CAP_TEMP | PROG_CAP_WATER },
		/* 3 筒自洁：总3min | 漂2(排水1+脱水2) | 脱1(10s→1min) | 600rpm */
		{ 1,  0,  2, 1,  1, 600, -1, false,
		  PROG_CAP_PRICE | PROG_CAP_RINSE_DUR | PROG_CAP_SPIN_DUR | PROG_CAP_RINSE_CNT | PROG_CAP_SPIN_RPM },
		/* 4 快洗：总24min | 洗9(进水3+洗涤6) | 漂8×1 | 脱7(排水1+脱水5+抖散1) | 800rpm | COLD */
		{ 7,  9,  8, 7,  1, 800,  0,  true,
		  PROG_CAP_PRICE | PROG_CAP_WASH | PROG_CAP_RINSE_DUR | PROG_CAP_SPIN_DUR |
		  PROG_CAP_RINSE_CNT | PROG_CAP_SPIN_RPM | PROG_CAP_TEMP | PROG_CAP_WATER },
	};
	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		g_prog_cfg_factory[i] = factory[i];
		g_prog_cfg[i] = factory[i];
	}
	program_admin_apply_all();
}

/* 将某一程序的 g_prog_cfg 写入 g_program_profiles / g_program_run_stages（供主页与运行页使用） */

static void program_admin_apply_one(int32_t idx)
{
	idx = wheel_mod_total(idx);
	const ui_program_admin_t * c = &g_prog_cfg[idx];
	uint32_t wash = 0;
	uint32_t rinse_total = 0;
	uint32_t spin = 0;

	program_admin_calc_stages(c, &wash, &rinse_total, &spin);

	g_program_profiles[idx].price = c->price;
	g_program_profiles[idx].temp = program_admin_temp_str(c->temp_idx);
	g_program_profiles[idx].wash_sec = wash + rinse_total + spin;

	g_program_run_stages[idx].wash_sec = wash;
	g_program_run_stages[idx].rinse_sec = rinse_total;
	g_program_run_stages[idx].spin_sec = spin;
	g_program_run_stages[idx].stages_bar_id = program_admin_stages_bar_id(wash, rinse_total);
}

static void program_admin_apply_all(void)
{
	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		program_admin_apply_one(i);
	}
}

static void program_admin_clamp_cfg(ui_program_admin_t * c)
{
	if(c->price > 999) c->price = 999;
	if(c->wash_min > 90) c->wash_min = 90;
	if(c->rinse_min > 90) c->rinse_min = 90;
	if(c->spin_min > 90) c->spin_min = 90;
	if(c->rinse_cnt > 4) c->rinse_cnt = 4;
}

static void program_admin_ui_save_fields(void)
{
	if(g_admin_prog_ui_loading) return;
	if(g_admin_prog_sel < 0 || g_admin_prog_sel >= TOTAL_PROGRAMS) return;
	ui_program_admin_t * c = &g_prog_cfg[g_admin_prog_sel];

	if(g_admin_prog_ta[0] != NULL && (c->cap & PROG_CAP_PRICE)) {
		const char * t = lv_textarea_get_text(g_admin_prog_ta[0]);
		if(t != NULL && t[0] != '\0') c->price = (int32_t)atoi(t);
	}
	if(g_admin_prog_ta[1] != NULL && (c->cap & PROG_CAP_WASH)) {
		const char * t = lv_textarea_get_text(g_admin_prog_ta[1]);
		if(t != NULL && t[0] != '\0') c->wash_min = (uint16_t)atoi(t);
	}
	if(g_admin_prog_ta[2] != NULL && (c->cap & PROG_CAP_RINSE_DUR)) {
		const char * t = lv_textarea_get_text(g_admin_prog_ta[2]);
		if(t != NULL && t[0] != '\0') c->rinse_min = (uint16_t)atoi(t);
	}
	if(g_admin_prog_ta[3] != NULL && (c->cap & PROG_CAP_SPIN_DUR)) {
		const char * t = lv_textarea_get_text(g_admin_prog_ta[3]);
		if(t != NULL && t[0] != '\0') c->spin_min = (uint16_t)atoi(t);
	}
	if(g_admin_prog_roller[0] != NULL && (c->cap & PROG_CAP_RINSE_CNT)) {
		c->rinse_cnt = (int8_t)lv_roller_get_selected(g_admin_prog_roller[0]);
	}
	if(g_admin_prog_roller[1] != NULL && (c->cap & PROG_CAP_SPIN_RPM)) {
		c->spin_rpm = program_admin_roller_to_spin_rpm(lv_roller_get_selected(g_admin_prog_roller[1]));
	}
	if(g_admin_prog_roller[2] != NULL && (c->cap & PROG_CAP_TEMP)) {
		c->temp_idx = (int8_t)lv_roller_get_selected(g_admin_prog_roller[2]);
	}
	program_admin_clamp_cfg(c);
}

static void program_admin_ui_load_fields(void)
{
	if(g_admin_prog_sel < 0 || g_admin_prog_sel >= TOTAL_PROGRAMS) return;
	const ui_program_admin_t * c = &g_prog_cfg[g_admin_prog_sel];
	char buf[16];

	g_admin_prog_ui_loading = true;

	if(g_admin_prog_ta[0] != NULL) {
		if(c->cap & PROG_CAP_PRICE) {
			if(c->price < 0) lv_textarea_set_text(g_admin_prog_ta[0], "");
			else lv_snprintf(buf, sizeof(buf), "%d", c->price), lv_textarea_set_text(g_admin_prog_ta[0], buf);
		}
	}
	if(g_admin_prog_ta[1] != NULL && (c->cap & PROG_CAP_WASH)) {
		lv_snprintf(buf, sizeof(buf), "%u", (unsigned)c->wash_min);
		lv_textarea_set_text(g_admin_prog_ta[1], buf);
	}
	if(g_admin_prog_ta[2] != NULL && (c->cap & PROG_CAP_RINSE_DUR)) {
		lv_snprintf(buf, sizeof(buf), "%u", (unsigned)c->rinse_min);
		lv_textarea_set_text(g_admin_prog_ta[2], buf);
	}
	if(g_admin_prog_ta[3] != NULL && (c->cap & PROG_CAP_SPIN_DUR)) {
		lv_snprintf(buf, sizeof(buf), "%u", (unsigned)c->spin_min);
		lv_textarea_set_text(g_admin_prog_ta[3], buf);
	}
	if(g_admin_prog_roller[0] != NULL && (c->cap & PROG_CAP_RINSE_CNT) && c->rinse_cnt >= 0) {
		lv_roller_set_selected(g_admin_prog_roller[0], (uint32_t)c->rinse_cnt, LV_ANIM_OFF);
	}
	if(g_admin_prog_roller[1] != NULL && (c->cap & PROG_CAP_SPIN_RPM) && c->spin_rpm >= 0) {
		lv_roller_set_selected(g_admin_prog_roller[1], (uint32_t)program_admin_spin_rpm_to_roller(c->spin_rpm), LV_ANIM_OFF);
	}
	if(g_admin_prog_roller[2] != NULL && (c->cap & PROG_CAP_TEMP) && c->temp_idx >= 0) {
		lv_roller_set_selected(g_admin_prog_roller[2], (uint32_t)c->temp_idx, LV_ANIM_OFF);
	}
	if(g_admin_prog_water_lbl != NULL) {
		if(c->cap & PROG_CAP_WATER) lv_label_set_text(g_admin_prog_water_lbl, ui_translation(STR_PROG_WATER_SMART));
		else lv_label_set_text(g_admin_prog_water_lbl, "--");
	}

	g_admin_prog_ui_loading = false;

	program_admin_ui_apply_caps();
	program_admin_sync_prog_pick_ui();
	if(g_admin_view == PROGRAM_SETTINGS) {
		admin_encoder_rebuild();
	}
}

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

static void program_admin_sync_prog_pick_ui(void)
{
	admin_prog_btn_style_init();
	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		lv_obj_t * b = g_admin_prog_btns[i];
		if(b == NULL) continue;
		if(i == g_admin_prog_sel) {
			lv_obj_remove_style(b, &s_admin_prog_btn_style, LV_PART_MAIN);
			lv_obj_add_style(b, &s_admin_prog_btn_sel_style, LV_PART_MAIN);
		} else {
			lv_obj_remove_style(b, &s_admin_prog_btn_sel_style, LV_PART_MAIN);
			lv_obj_add_style(b, &s_admin_prog_btn_style, LV_PART_MAIN);
		}
	}
	program_admin_update_total_display();
}

static void program_admin_update_total_display(void)
{
	if(g_admin_prog_total_val_lbl == NULL) return;
	if(g_admin_prog_sel < 0 || g_admin_prog_sel >= TOTAL_PROGRAMS) return;

	const ui_program_admin_t * c = &g_prog_cfg[g_admin_prog_sel];
	uint32_t wash = 0;
	uint32_t rinse_total = 0;
	uint32_t spin = 0;
	program_admin_calc_stages(c, &wash, &rinse_total, &spin);
	const uint32_t total = wash + rinse_total + spin;

	char buf[16];
	program_format_time_label(total, buf, sizeof(buf));
	lv_label_set_text(g_admin_prog_total_val_lbl, buf);
}

static void cb_admin_prog_time_field_changed(lv_event_t * e)
{
	(void)e;
	if(g_admin_prog_ui_loading) return;
	if(g_admin_view != PROGRAM_SETTINGS) return;
	program_admin_ui_save_fields();
	program_admin_update_total_display();
}

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

static void program_admin_ui_apply_caps(void)
{
	if(g_admin_prog_sel < 0 || g_admin_prog_sel >= TOTAL_PROGRAMS) return;
	const ui_program_admin_t * c = &g_prog_cfg[g_admin_prog_sel];
	static const uint16_t cap_map[8] = {
		PROG_CAP_PRICE, PROG_CAP_WASH, PROG_CAP_RINSE_DUR, PROG_CAP_SPIN_DUR,
		PROG_CAP_RINSE_CNT, PROG_CAP_SPIN_RPM, PROG_CAP_TEMP, PROG_CAP_WATER
	};
	lv_obj_t * widgets[8] = {
		g_admin_prog_ta[0], g_admin_prog_ta[1], g_admin_prog_ta[2], g_admin_prog_ta[3],
		g_admin_prog_roller[0], g_admin_prog_roller[1], g_admin_prog_roller[2], g_admin_prog_water_lbl
	};
	for(int i = 0; i < 8; i++) {
		bool en = (c->cap & cap_map[i]) != 0;
		if(widgets[i] != NULL) {
			if(en) lv_obj_remove_flag(widgets[i], LV_OBJ_FLAG_HIDDEN);
			else lv_obj_add_flag(widgets[i], LV_OBJ_FLAG_HIDDEN);
		}
		if(g_admin_prog_dash[i] != NULL) {
			if(en) lv_obj_add_flag(g_admin_prog_dash[i], LV_OBJ_FLAG_HIDDEN);
			else lv_obj_remove_flag(g_admin_prog_dash[i], LV_OBJ_FLAG_HIDDEN);
		}
	}
}

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

static void cb_admin_prog_ta_focus(lv_event_t * e)
{
	lv_obj_t * ta = lv_event_get_target_obj(e);
	lv_event_code_t code = lv_event_get_code(e);
	if(g_admin_view != PROGRAM_SETTINGS) return;

	if(code == LV_EVENT_FOCUSED) {
		/* 编码器：进入 group 编辑模式后再弹键盘；不因失焦自动关闭 */
		if(g_group_admin != NULL && lv_group_get_editing(g_group_admin)) {
			program_admin_ta_begin_edit(ta);
		}
	} else if(code == LV_EVENT_CLICKED) {
		/* 触摸：点击文本框后进入编辑 */
		program_admin_ta_begin_edit(ta);
		if(g_group_admin != NULL) {
			lv_group_set_editing(g_group_admin, true);
		}
	}
}

static void cb_admin_prog_pick(lv_event_t * e)
{
	program_admin_ui_save_fields();
	g_admin_prog_sel = (int32_t)(intptr_t)lv_event_get_user_data(e);
	program_admin_ui_load_fields();
}

static void cb_admin_prog_reset(lv_event_t * e)
{
	(void)e;
	if(g_admin_prog_sel < 0 || g_admin_prog_sel >= TOTAL_PROGRAMS) return;
	g_prog_cfg[g_admin_prog_sel] = g_prog_cfg_factory[g_admin_prog_sel];
	program_admin_ui_load_fields();
}

static void cb_admin_prog_confirm(lv_event_t * e)
{
	(void)e;
	program_admin_ui_save_fields();
	program_admin_apply_all();   /* 将全部程序 cfg 同步到主页/运行页使用的参数表 */
	home_sync_program_labels();  /* 刷新主页底部时间/温度/金额 */
	pay_sync_price_label();
	program_admin_back_to_menu1(); /* 确认后返回管理员设置页 */
}

static void cb_admin_open_program_settings(lv_event_t * e)
{
	(void)e;
	g_admin_prog_sel = 0;
	admin_panel_show(PROGRAM_SETTINGS);
}

static void program_admin_back_to_menu1(void)
{
	program_admin_ui_save_fields();
	if(g_group_admin != NULL) {
		lv_group_set_editing(g_group_admin, false);
	}
	program_admin_ta_close_kb();
	admin_panel_show(MENU1);
}

/* 构建程序设置子页：上栏程序列表，下栏参数值外框 + 8 参数（label/value/edit） */

static void program_admin_build_panel(lv_obj_t * root, lv_coord_t body_y, lv_coord_t body_h)
{
		static const ui_str_id_t field_ids[8] = {
		STR_PROG_FIELD_PRICE, STR_PROG_FIELD_WASH, STR_PROG_FIELD_RINSE_DUR, STR_PROG_FIELD_SPIN_DUR,
		STR_PROG_FIELD_RINSE_CNT, STR_PROG_FIELD_SPIN_RPM, STR_PROG_FIELD_TEMP, STR_PROG_FIELD_WATER
	};

	g_admin_panel_program = lv_obj_create(root);
	lv_obj_set_size(g_admin_panel_program, LV_PCT(100), body_h);
	lv_obj_align(g_admin_panel_program, LV_ALIGN_TOP_MID, 0, body_y);
	lv_obj_set_style_bg_opa(g_admin_panel_program, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(g_admin_panel_program, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(g_admin_panel_program, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(g_admin_panel_program, LV_LAYOUT_NONE, LV_PART_MAIN);
	lv_obj_add_flag(g_admin_panel_program, LV_OBJ_FLAG_HIDDEN);

	/* 标题 */
	lv_obj_t * title = lv_label_create(g_admin_panel_program);
	ui_lang_bind_label(title, STR_ADMIN_M1_PROGRAM);
	ui_set_obj_font(title, s_font_sc_30);
	lv_obj_set_style_text_color(title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

	/* 上栏：5 个程序按钮横向排列 */
	const lv_coord_t prog_btn_w = 140;
	const lv_coord_t prog_btn_h = 48;
	const lv_coord_t prog_gap = 12;
	const lv_coord_t prog_row_w = TOTAL_PROGRAMS * prog_btn_w + (TOTAL_PROGRAMS - 1) * prog_gap;
	const lv_coord_t prog_row_x0 = (lv_coord_t)((UI_FIXED_W - prog_row_w) / 2);
	const lv_coord_t prog_row_y = 36+10;

	/* 上栏左侧固定：总程序时长（与主页底部时间列相同算法/格式） */
	const lv_coord_t total_box_w = 120;
	const lv_coord_t total_gap = 12;
	const lv_coord_t total_x = prog_row_x0 - total_gap - total_box_w;

	lv_obj_t * total_box = program_admin_make_value_box(g_admin_panel_program, total_box_w, prog_btn_h);
	lv_obj_set_pos(total_box, total_x, prog_row_y);
	g_admin_prog_total_val_lbl = lv_label_create(total_box);
	lv_label_set_text(g_admin_prog_total_val_lbl, "0min");
	ui_set_obj_font(g_admin_prog_total_val_lbl, s_font_sc_30);
	lv_obj_set_style_text_color(g_admin_prog_total_val_lbl, lv_color_hex(0x333333), LV_PART_MAIN);

	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		g_admin_prog_btns[i] = make_admin_prog_btn(g_admin_panel_program, ui_program_name_get(i));
		admin_prog_btn_bind_i18n(g_admin_prog_btns[i], g_mode_name_ids[i]);
		lv_obj_set_size(g_admin_prog_btns[i], prog_btn_w, prog_btn_h);
		lv_obj_set_pos(g_admin_prog_btns[i],
			prog_row_x0 + i * (prog_btn_w + prog_gap), prog_row_y);
		lv_obj_add_event_cb(g_admin_prog_btns[i], cb_admin_prog_pick, LV_EVENT_CLICKED, (void *)(intptr_t)i);
	}

	/* 下栏：左侧 8 参数 + 右侧重置/确认（field_w × field_h 全部一致） */
	const lv_coord_t btn_col_w = 100;
	const lv_coord_t btn_col_x = (lv_coord_t)UI_FIXED_W - btn_col_w - 80;
	const lv_coord_t field_x0 = 100;//左间距
	const lv_coord_t field_gap = 8;//间距
	const lv_coord_t field_h = 60;//高度
	const lv_coord_t field_w = 150;//宽度
	const lv_coord_t field_y_lbl = 100+30;//文字y位置
	const lv_coord_t field_y_val = 132+45;//文本框y位置

	for(int i = 0; i < 8; i++) {
		const lv_coord_t fx = field_x0 + i * (field_w + field_gap);
		lv_obj_t * lbl = lv_label_create(g_admin_panel_program);
		g_admin_prog_field_lbl[i] = lbl;
		ui_lang_bind_label(lbl, field_ids[i]);
		ui_set_obj_font(lbl, s_font_sc_30);
		lv_obj_set_style_text_color(lbl, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		lv_obj_set_width(lbl, field_w);
		lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
		lv_obj_set_pos(lbl, fx, field_y_lbl);

		/* 统一外框：8 列同宽同高，内部控件铺满外框 */
		g_admin_prog_field_box[i] = program_admin_make_value_box(g_admin_panel_program, field_w, field_h);
		lv_obj_set_pos(g_admin_prog_field_box[i], fx, field_y_val);
		lv_obj_add_flag(g_admin_prog_field_box[i], LV_OBJ_FLAG_OVERFLOW_VISIBLE);

		g_admin_prog_dash[i] = lv_label_create(g_admin_prog_field_box[i]);
		lv_label_set_text(g_admin_prog_dash[i], "--");
		ui_set_obj_font(g_admin_prog_dash[i], s_font_sc_30);
		lv_obj_set_style_text_color(g_admin_prog_dash[i], lv_color_hex(0x333333), LV_PART_MAIN);
		lv_obj_add_flag(g_admin_prog_dash[i], LV_OBJ_FLAG_HIDDEN);
	}

	for(int i = 0; i < 4; i++) {
		g_admin_prog_ta[i] = lv_textarea_create(g_admin_prog_field_box[i]);
		program_admin_style_ta(g_admin_prog_ta[i]);
		lv_textarea_set_max_length(g_admin_prog_ta[i], 3);
		lv_textarea_set_accepted_chars(g_admin_prog_ta[i], "0123456789");
		lv_obj_set_style_text_font(g_admin_prog_ta[i], s_font_sc_30, LV_PART_MAIN);
		lv_obj_set_style_text_color(g_admin_prog_ta[i], lv_color_hex(0x000000), LV_PART_MAIN);
		lv_obj_add_flag(g_admin_prog_ta[i], LV_OBJ_FLAG_CLICKABLE);
		lv_obj_add_event_cb(g_admin_prog_ta[i], cb_admin_prog_ta_focus, LV_EVENT_ALL, NULL);
		style_prog_field_encoder_focus_inner(g_admin_prog_ta[i]);
		if(i >= 1 && i <= 3) {
			lv_obj_add_event_cb(g_admin_prog_ta[i], cb_admin_prog_time_field_changed, LV_EVENT_VALUE_CHANGED, NULL);
		}
	}
	lv_textarea_set_max_length(g_admin_prog_ta[0], 3);

	for(int i = 0; i < 3; i++) {
		g_admin_prog_roller[i] = lv_roller_create(g_admin_prog_field_box[4 + i]);
		program_admin_style_field_inner(g_admin_prog_roller[i]);
		lv_roller_set_visible_row_count(g_admin_prog_roller[i], 1);
		lv_obj_set_style_text_font(g_admin_prog_roller[i], s_font_sc_30, LV_PART_MAIN);
		lv_obj_set_style_text_font(g_admin_prog_roller[i], s_font_sc_30, LV_PART_SELECTED);
		lv_obj_set_style_bg_opa(g_admin_prog_roller[i], LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_bg_opa(g_admin_prog_roller[i], LV_OPA_TRANSP, LV_PART_SELECTED);
		lv_obj_set_style_text_color(g_admin_prog_roller[i], lv_color_hex(0x000000), LV_PART_MAIN);
		lv_obj_set_style_text_color(g_admin_prog_roller[i], lv_color_hex(0x000000), LV_PART_SELECTED);
		lv_obj_add_event_cb(g_admin_prog_roller[i], cb_admin_prog_roller_encoder, LV_EVENT_CLICKED, NULL);
		lv_obj_add_event_cb(g_admin_prog_roller[i], cb_admin_prog_roller_encoder, LV_EVENT_DEFOCUSED, NULL);
		style_prog_field_encoder_focus_inner(g_admin_prog_roller[i]);
	}
	lv_roller_set_options(g_admin_prog_roller[0], "0\n1\n2\n3\n4", LV_ROLLER_MODE_NORMAL);
	lv_roller_set_options(g_admin_prog_roller[1], "1200\n800\n600\n400\n0", LV_ROLLER_MODE_NORMAL);
	lv_roller_set_options(g_admin_prog_roller[2], "COLD\n30℃\n40℃\n60℃\n90℃", LV_ROLLER_MODE_NORMAL);
	for(int i = 0; i < 3; i++) {
		lv_obj_add_event_cb(g_admin_prog_roller[i], cb_admin_prog_time_field_changed, LV_EVENT_VALUE_CHANGED, NULL);
	}

	g_admin_prog_water_lbl = lv_label_create(g_admin_prog_field_box[7]);
	lv_label_set_text(g_admin_prog_water_lbl, ui_translation(STR_PROG_WATER_SMART));
	ui_set_obj_font(g_admin_prog_water_lbl, s_font_sc_30);
	lv_obj_set_style_text_color(g_admin_prog_water_lbl, lv_color_hex(0x333333), LV_PART_MAIN);
	lv_obj_set_width(g_admin_prog_water_lbl, LV_PCT(100));
	lv_obj_set_style_text_align(g_admin_prog_water_lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

	/* 下栏右侧：重置（上）、确认（下） */
	g_admin_btn_prog_reset = make_orange_outline_btn(g_admin_panel_program, ui_translation(STR_BTN_RESET), btn_col_w, 44);
	lv_obj_set_pos(g_admin_btn_prog_reset, btn_col_x, field_y_val);
	ui_set_obj_font(lv_obj_get_child(g_admin_btn_prog_reset, 0), s_font_sc_30);
	orange_btn_bind_i18n(g_admin_btn_prog_reset, STR_BTN_RESET);
	lv_obj_add_event_cb(g_admin_btn_prog_reset, cb_admin_prog_reset, LV_EVENT_CLICKED, NULL);

	g_admin_btn_prog_confirm = make_orange_fill_btn(g_admin_panel_program, ui_translation(STR_BTN_CONFIRM), btn_col_w, 44);
	lv_obj_set_pos(g_admin_btn_prog_confirm, btn_col_x, field_y_val + 44 + 10);
	ui_set_obj_font(lv_obj_get_child(g_admin_btn_prog_confirm, 0), s_font_sc_30);
	orange_btn_bind_i18n(g_admin_btn_prog_confirm, STR_BTN_CONFIRM);
	lv_obj_add_event_cb(g_admin_btn_prog_confirm, cb_admin_prog_confirm, LV_EVENT_CLICKED, NULL);

	g_admin_prog_sel = 0;
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

static lv_obj_t * make_admin_menu_btn(lv_obj_t * parent, const char * txt)
{
	admin_menu_style_init();
	lv_obj_t * b = lv_button_create(parent);
	lv_obj_add_style(b, &s_admin_menu_frame_style, LV_PART_MAIN);
	lv_obj_set_style_shadow_width(b, 0, LV_PART_MAIN);

	lv_obj_t * inner = lv_obj_create(b);
	lv_obj_remove_style_all(inner);
	lv_obj_add_style(inner, &s_admin_menu_inner_style, LV_PART_MAIN);
	lv_obj_set_size(inner, LV_PCT(100), LV_PCT(100));
	lv_obj_clear_flag(inner, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

	lv_obj_t * l = lv_label_create(inner);
	lv_label_set_text(l, txt);
	lv_obj_set_style_text_color(l, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(l, s_font_sc_30);
	lv_obj_center(l);
	return b;
}

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
            lv_group_set_editing(g_group_admin, true);
            focus_first = g_admin_kb;
        }
        /* 首次进入不显示焦点框 */
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
    case MACHINE_ID:
        if(g_admin_ta_machine_id != NULL) ui_encoder_group_add(g_group_admin, g_admin_ta_machine_id);
        if(g_admin_btn_machine_confirm != NULL) ui_encoder_group_add(g_group_admin, g_admin_btn_machine_confirm);
        if(g_admin_kb != NULL) admin_encoder_group_add_kb(g_group_admin);
        if(admin_kb_is_visible() && g_admin_kb != NULL &&
           g_admin_kb_ta == g_admin_ta_machine_id) {
            lv_group_set_editing(g_group_admin, true);
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
            static const uint16_t ta_caps[4] = {
                PROG_CAP_PRICE, PROG_CAP_WASH, PROG_CAP_RINSE_DUR, PROG_CAP_SPIN_DUR
            };
            for(int i = 0; i < 4; i++) {
                if(g_admin_prog_ta[i] != NULL && (c->cap & ta_caps[i])) {
                    ui_encoder_group_add_prog_field(g_group_admin, g_admin_prog_ta[i]);
                }
            }
        }
        for(int i = 0; i < 3; i++) {
            if(g_admin_prog_roller[i] != NULL) {
                ui_encoder_group_add_prog_field(g_group_admin, g_admin_prog_roller[i]);
            }
        }
        if(g_admin_btn_prog_reset != NULL) ui_encoder_group_add(g_group_admin, g_admin_btn_prog_reset);
        if(g_admin_btn_prog_confirm != NULL) ui_encoder_group_add(g_group_admin, g_admin_btn_prog_confirm);
        if(g_admin_kb != NULL) admin_encoder_group_add_kb(g_group_admin);
        if(admin_kb_is_visible() && g_admin_ta_prog_active != NULL) {
            lv_group_set_editing(g_group_admin, true);
            focus_first = g_admin_kb;
        } else {
            lv_group_set_editing(g_group_admin, false);
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
    case VENDOR_SERIAL:
        if(g_admin_ta_vendor_serial != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_ta_vendor_serial);
        }
        if(g_admin_kb != NULL) {
            admin_encoder_group_add_kb(g_group_admin);
        }
        if(admin_kb_is_visible() && g_admin_kb != NULL &&
           g_admin_kb_ta == g_admin_ta_vendor_serial) {
            lv_group_set_editing(g_group_admin, true);
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
    case DORMANCY_STANDBY:
        if(g_admin_dormancy_roller != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_dormancy_roller);
        }
        if(g_admin_btn_dormancy_confirm != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_btn_dormancy_confirm);
        }
        focus_first = (g_admin_dormancy_roller != NULL) ? g_admin_dormancy_roller : g_admin_btn_back;
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
    case OZONE:
        if(g_admin_btn_ozone_on != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_btn_ozone_on);
        }
        if(g_admin_btn_ozone_off != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_btn_ozone_off);
        }
        if(!ui_ozone_get() && g_admin_btn_ozone_off != NULL) {
            focus_first = g_admin_btn_ozone_off;
        }
        else if(g_admin_btn_ozone_on != NULL) {
            focus_first = g_admin_btn_ozone_on;
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
        if(g_admin_cb_payment_alipay != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_cb_payment_alipay);
        }
        if(g_admin_cb_payment_wechat != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_cb_payment_wechat);
        }
        if(g_admin_payment_timeout_roller != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_payment_timeout_roller);
        }
        if(g_admin_btn_payment_query != NULL) {
            ui_encoder_group_add(g_group_admin, g_admin_btn_payment_query);
        }
        if(g_group_admin != NULL) {
            lv_group_set_editing(g_group_admin, false);
        }
        focus_first = (g_admin_cb_payment_alipay != NULL) ?
            g_admin_cb_payment_alipay : g_admin_btn_back;
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
            lv_group_set_editing(g_group_admin, false);
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
            lv_group_set_editing(g_group_admin, true);
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
                lv_group_set_editing(g_group_admin, true);
                focus_first = g_admin_kb;
            }
        }
        if(focus_first == g_admin_btn_back) {
            focus_first = (g_admin_pwd_chg_new_step == 1 && g_admin_ta_pwd_chg_new2 != NULL) ?
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
        if(lv_group_get_focused(g_group_admin) == focus_first) {
            lv_obj_add_state(focus_first, LV_STATE_FOCUS_KEY);
        }
        if(focus_first == g_admin_kb && g_group_admin != NULL && lv_group_get_editing(g_group_admin)) {
            admin_kb_encoder_select_first(g_admin_kb);
        }
        /* 待机时间页：初始焦点在 roller 且直接进入编辑，旋转即可改选项 */
        if(g_admin_view == DORMANCY_STANDBY && focus_first == g_admin_dormancy_roller &&
           g_group_admin != NULL) {
            lv_group_set_editing(g_group_admin, true);
        }
        if(g_admin_view == SCREEN_BRIGHTNESS) {
            admin_brightness_slider_sync_focus_frame();
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
    if(g_admin_panel_vendor_serial != NULL) lv_obj_add_flag(g_admin_panel_vendor_serial, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_vendor_menu != NULL) lv_obj_add_flag(g_admin_panel_vendor_menu, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_sound != NULL) lv_obj_add_flag(g_admin_panel_sound, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_dormancy != NULL) lv_obj_add_flag(g_admin_panel_dormancy, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_lang != NULL) lv_obj_add_flag(g_admin_panel_lang, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_factory != NULL) lv_obj_add_flag(g_admin_panel_factory, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_contact != NULL) lv_obj_add_flag(g_admin_panel_contact, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_auto_dispense != NULL) lv_obj_add_flag(g_admin_panel_auto_dispense, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_ozone != NULL) lv_obj_add_flag(g_admin_panel_ozone, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_fresh_air_care != NULL) lv_obj_add_flag(g_admin_panel_fresh_air_care, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_system_upgrade != NULL) lv_obj_add_flag(g_admin_panel_system_upgrade, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_payment != NULL) lv_obj_add_flag(g_admin_panel_payment, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_data != NULL) lv_obj_add_flag(g_admin_panel_data, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_network != NULL) lv_obj_add_flag(g_admin_panel_network, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_wifi != NULL) lv_obj_add_flag(g_admin_panel_wifi, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_4g != NULL) lv_obj_add_flag(g_admin_panel_4g, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_pwd_chg_old != NULL) lv_obj_add_flag(g_admin_panel_pwd_chg_old, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_panel_pwd_chg_new != NULL) lv_obj_add_flag(g_admin_panel_pwd_chg_new, LV_OBJ_FLAG_HIDDEN);

    if(g_admin_lbl_msg_pwd != NULL) {
        lv_label_set_text(g_admin_lbl_msg_pwd, "");
        lv_obj_add_flag(g_admin_lbl_msg_pwd, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_lbl_msg_machine_id != NULL) {
        lv_label_set_text(g_admin_lbl_msg_machine_id, "");
        lv_obj_add_flag(g_admin_lbl_msg_machine_id, LV_OBJ_FLAG_HIDDEN);
    }

    if(view == PASSWORD && g_admin_panel_pwd != NULL) {
        lv_obj_remove_flag(g_admin_panel_pwd, LV_OBJ_FLAG_HIDDEN);
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
            lv_group_set_editing(g_group_admin, false);
        }
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        program_admin_ui_load_fields();
    }
    else if(view == SCREEN_BRIGHTNESS && g_admin_panel_brightness != NULL) {
        lv_obj_remove_flag(g_admin_panel_brightness, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            lv_group_set_editing(g_group_admin, false);
        }
        admin_brightness_sync_ui();
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
    else if(view == DORMANCY_STANDBY && g_admin_panel_dormancy != NULL) {
        lv_obj_remove_flag(g_admin_panel_dormancy, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            lv_group_set_editing(g_group_admin, false);
        }
        admin_dormancy_sync_roller();
    }
    else if(view == LANGUAGE_SETTINGS && g_admin_panel_lang != NULL) {
        lv_obj_remove_flag(g_admin_panel_lang, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            lv_group_set_editing(g_group_admin, false);
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
            lv_group_set_editing(g_group_admin, false);
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
            lv_group_set_editing(g_group_admin, false);
        }
    }
    else if(view == AUTO_DISPENSE && g_admin_panel_auto_dispense != NULL) {
        lv_obj_remove_flag(g_admin_panel_auto_dispense, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            lv_group_set_editing(g_group_admin, false);
        }
        admin_auto_dispense_sync_btn_ui();
    }
    else if(view == OZONE && g_admin_panel_ozone != NULL) {
        lv_obj_remove_flag(g_admin_panel_ozone, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            lv_group_set_editing(g_group_admin, false);
        }
        admin_ozone_sync_btn_ui();
    }
    else if(view == FRESH_AIR_CARE && g_admin_panel_fresh_air_care != NULL) {
        lv_obj_remove_flag(g_admin_panel_fresh_air_care, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            lv_group_set_editing(g_group_admin, false);
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
            lv_group_set_editing(g_group_admin, false);
        }
        admin_system_upgrade_ui_enter();
    }
    else if(view == PAYMENT_SETTINGS && g_admin_panel_payment != NULL) {
        lv_obj_remove_flag(g_admin_panel_payment, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            lv_group_set_editing(g_group_admin, false);
        }
        admin_payment_sync_checkbox_ui();
        admin_payment_sync_timeout_roller_ui();
    }
    else if(view == DATA_SETTINGS && g_admin_panel_data != NULL) {
        lv_obj_remove_flag(g_admin_panel_data, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            lv_group_set_editing(g_group_admin, false);
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
            lv_group_set_editing(g_group_admin, false);
        }
    }
    else if(view == WIFI_SETTINGS && g_admin_panel_wifi != NULL) {
        lv_obj_remove_flag(g_admin_panel_wifi, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            lv_group_set_editing(g_group_admin, false);
        }
    }
    else if(view == SETTINGS_4G && g_admin_panel_4g != NULL) {
        lv_obj_remove_flag(g_admin_panel_4g, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_group_admin != NULL) {
            lv_group_set_editing(g_group_admin, false);
        }
        admin_4g_ui_enter();
    }
    else if(view == PASSWORD_CHANGE_OLD && g_admin_panel_pwd_chg_old != NULL) {
        lv_obj_remove_flag(g_admin_panel_pwd_chg_old, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_kb != NULL) {
            lv_obj_remove_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
            g_admin_kb_ta = g_admin_ta_pwd_chg_old;
        }
    }
    else if(view == PASSWORD_CHANGE_NEW && g_admin_panel_pwd_chg_new != NULL) {
        lv_obj_remove_flag(g_admin_panel_pwd_chg_new, LV_OBJ_FLAG_HIDDEN);
        g_admin_pwd_chg_new_step = 0;
        if(g_admin_ta_pwd_chg_new1 != NULL) lv_textarea_set_text(g_admin_ta_pwd_chg_new1, "");
        if(g_admin_ta_pwd_chg_new2 != NULL) lv_textarea_set_text(g_admin_ta_pwd_chg_new2, "");
        if(g_admin_lbl_msg_pwd_chg_new != NULL) {
            lv_label_set_text(g_admin_lbl_msg_pwd_chg_new, "");
            lv_obj_add_flag(g_admin_lbl_msg_pwd_chg_new, LV_OBJ_FLAG_HIDDEN);
        }
        if(g_admin_kb != NULL && g_admin_ta_pwd_chg_new1 != NULL) {
            lv_obj_remove_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
            g_admin_kb_ta = g_admin_ta_pwd_chg_new1;
        }
    }

    admin_encoder_rebuild();
}

/* 错误提示遮罩：点击任意处关闭，回到输入页 */
static void cb_admin_err_blocker(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    lv_obj_t * blk = lv_event_get_current_target(e);
    if(g_admin_lbl_msg_pwd != NULL) lv_obj_add_flag(g_admin_lbl_msg_pwd, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_ta_pwd != NULL) lv_textarea_set_text(g_admin_ta_pwd, "");
    /* 恢复提示文字和输入框 */
    if(g_admin_lbl_serial_title != NULL) lv_obj_remove_flag(g_admin_lbl_serial_title, LV_OBJ_FLAG_HIDDEN);
    if(g_admin_input_wrap != NULL) lv_obj_remove_flag(g_admin_input_wrap, LV_OBJ_FLAG_HIDDEN);
    if(blk != NULL) lv_obj_del(blk);
}

/* 键盘按钮点击：数字/删除/确认 */
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
    if(lv_strcmp(t, VENDOR_SERIAL_CODE) != 0) { //VENDOR_SERIAL_CODE为宏定义的管理员密码
        if(g_admin_lbl_msg_pwd != NULL) {
            g_admin_pwd_err_id = STR_PWD_WRONG_RETRY;
            lv_label_set_text(g_admin_lbl_msg_pwd, ui_translation(STR_PWD_WRONG_RETRY));
            lv_obj_remove_flag(g_admin_lbl_msg_pwd, LV_OBJ_FLAG_HIDDEN);
        }
        lv_textarea_set_text(g_admin_ta_pwd, "");
        /* 隐藏提示文字、输入框和背景图 */
        if(g_admin_lbl_serial_title != NULL) lv_obj_add_flag(g_admin_lbl_serial_title, LV_OBJ_FLAG_HIDDEN);
        if(g_admin_input_wrap != NULL) lv_obj_add_flag(g_admin_input_wrap, LV_OBJ_FLAG_HIDDEN);
        /* 错误时关闭键盘，点击任意处回输入页 */
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = NULL;
            lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
        }
        /* 全屏透明遮罩：点击即关闭错误提示 */
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
    if(g_admin_lbl_msg_pwd != NULL) lv_obj_add_flag(g_admin_lbl_msg_pwd, LV_OBJ_FLAG_HIDDEN);
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
		admin_pwd_chg_old_try();
	} else if(g_admin_view == PASSWORD_CHANGE_NEW) {
		admin_pwd_chg_new_on_ready();
	} else if(g_admin_view == VENDOR_SERIAL) {
		admin_vendor_serial_try();
	} else if(g_admin_view == MACHINE_ID) {
		/* 键盘 OK：仅保存 ID，不切页；返回设置页请点「确认」或顶栏返回 */
		(void)admin_machine_id_apply();
	}
}

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
    if(g_admin_view == VENDOR_SERIAL) {
        admin_vendor_serial_back_to_menu1();
        return;
    }
    if(g_admin_view == VENDOR_MENU) {
        admin_vendor_menu_back_to_menu1();
        return;
    }
    if(g_admin_view == DORMANCY_STANDBY) {
        admin_dormancy_back_to_menu1();
        return;
    }
    if(g_admin_view == LANGUAGE_SETTINGS) {
        admin_lang_back_to_menu1();
        return;
    }
    if(g_admin_view == FACTORY_RESET) {
        admin_factory_back_to_menu1();
        return;
    }
    if(g_admin_view == CONTACT_US) {
        admin_contact_back_to_menu1();
        return;
    }
    if(g_admin_view == AUTO_DISPENSE) {
        admin_auto_dispense_back_to_menu2();
        return;
    }
    if(g_admin_view == OZONE) {
        admin_ozone_back_to_menu2();
        return;
    }
    if(g_admin_view == FRESH_AIR_CARE) {
        admin_fresh_air_care_back_to_menu2();
        return;
    }
    if(g_admin_view == SYSTEM_UPGRADE) {
        admin_system_upgrade_back_to_menu2();
        return;
    }
    if(g_admin_view == PAYMENT_SETTINGS) {
        admin_payment_back_to_menu2();
        return;
    }
    if(g_admin_view == DATA_SETTINGS) {
        admin_data_back_to_menu2();
        return;
    }
    if(g_admin_view == NETWORK_SETTINGS) {
        admin_network_back_to_menu2();
        return;
    }
    if(g_admin_view == WIFI_SETTINGS) {
        admin_wifi_back_to_network();
        return;
    }
    if(g_admin_view == SETTINGS_4G) {
        admin_4g_back_to_menu2();
        return;
    }
    if(g_admin_view == PASSWORD_CHANGE_NEW || g_admin_view == PASSWORD_CHANGE_OLD) {
        admin_pwd_chg_back_to_menu2();
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
		admin_machine_id_back_to_menu1();
	}
}

static void ui_idle_apply_dormancy_period(void)
{
    if(g_idle_timer == NULL) return;
    if(g_ui_dormancy_timeout_ms == UI_DORMANCY_DISABLED_MS) {
        lv_timer_pause(g_idle_timer);
        return;
    }
    lv_timer_set_period(g_idle_timer, g_ui_dormancy_timeout_ms);
    lv_timer_resume(g_idle_timer);
    ui_idle_reset();
}

static void admin_prog_roller_exit_edit(lv_obj_t * roller)
{
    if(g_group_admin == NULL || roller == NULL) return;
    if(lv_group_get_focused(g_group_admin) != roller) return;
    if(!lv_group_get_editing(g_group_admin)) return;
    uint32_t sel = lv_roller_get_selected(roller);
    lv_roller_set_selected(roller, sel, LV_ANIM_OFF);
    program_admin_ui_save_fields();
    lv_group_set_editing(g_group_admin, false);
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
        if(g_group_admin != NULL && lv_group_get_editing(g_group_admin)) {
            admin_prog_roller_exit_edit(roller);
        }
        return;
    }

    if(admin_kb_is_visible()) return;

    if(code != LV_EVENT_CLICKED) return;
    if(g_group_admin == NULL || lv_group_get_focused(g_group_admin) != roller) return;

    if(lv_group_get_editing(g_group_admin)) {
        admin_prog_roller_exit_edit(roller);
    } else {
        uint32_t sel = lv_roller_get_selected(roller);
        lv_roller_set_selected(roller, sel, LV_ANIM_OFF);
        lv_group_set_editing(g_group_admin, true);
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

static void admin_dormancy_label_update(void)
{
    if(g_admin_lbl_dormancy_cur == NULL) return;
    char buf[48];
    lv_snprintf(buf, sizeof(buf), ui_translation(STR_DORMANCY_CUR_FMT), ui_dormancy_timeout_label(g_ui_dormancy_timeout_ms));
    lv_label_set_text(g_admin_lbl_dormancy_cur, buf);
}

/* 进入待机时间页时：roller 选中项与「当前」标签对齐已保存配置 */

static void admin_dormancy_sync_roller(void)
{
    if(g_admin_dormancy_roller == NULL) return;
    lv_roller_set_selected(g_admin_dormancy_roller, ui_dormancy_roller_index_from_ms(g_ui_dormancy_timeout_ms), LV_ANIM_OFF);
    admin_dormancy_label_update();
}

/* 离开待机时间子页：关闭 group 编辑态并回到管理员菜单 */

static void admin_dormancy_back_to_menu1(void)
{
    if(g_group_admin != NULL) {
        lv_group_set_editing(g_group_admin, false);
    }
    admin_panel_show(MENU1);
}

/* 待机 roller 数值变化：仅更新「当前」预览，不写 g_ui_dormancy_timeout_ms（确认键才保存） */

static void cb_admin_dormancy_roller_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(g_admin_dormancy_roller == NULL || g_admin_lbl_dormancy_cur == NULL) return;
    uint32_t idx = lv_roller_get_selected(g_admin_dormancy_roller);
    if(idx >= DORMANCY_ROLLER_CNT) return;
    char buf[48];
    lv_snprintf(buf, sizeof(buf), ui_translation(STR_DORMANCY_CUR_FMT), ui_dormancy_timeout_label(g_dormancy_timeout_ms_tbl[idx]));
    lv_label_set_text(g_admin_lbl_dormancy_cur, buf);
}

/*
 * 待机时间 roller 退出编码器编辑态。
 * 短按第二次编码器 / 失焦时调用；先同步 LVGL 内部 ori 再 editing=false，避免选项被回滚。
 */

static void admin_dormancy_roller_exit_edit(lv_obj_t * roller)
{
    if(g_group_admin == NULL || roller == NULL) return;
    if(lv_group_get_focused(g_group_admin) != roller) return;
    if(!lv_group_get_editing(g_group_admin)) return;
    /* LVGL：editing→false 触发的 FOCUSED 会把选中项恢复为 sel_opt_id_ori */
    uint32_t sel = lv_roller_get_selected(roller);
    if(sel < DORMANCY_ROLLER_CNT) {
        lv_roller_set_selected(roller, sel, LV_ANIM_OFF);
    }
    lv_group_set_editing(g_group_admin, false);
}

/*
 * 待机时间 roller：外设编码器短按发 LV_EVENT_CLICKED，在编辑/导航间切换。
 * 进页默认编辑态；短按退出后可再短按进入；失焦时自动退出编辑。
 */

static void cb_admin_dormancy_roller_encoder(lv_event_t * e)
{
    lv_obj_t * roller = lv_event_get_target_obj(e);
    lv_event_code_t code = lv_event_get_code(e);

    if(g_admin_view != DORMANCY_STANDBY) return;
    if(roller == NULL || lv_obj_has_flag(roller, LV_OBJ_FLAG_HIDDEN)) return;

    if(code == LV_EVENT_DEFOCUSED) {
        if(g_group_admin != NULL && lv_group_get_editing(g_group_admin)) {
            admin_dormancy_roller_exit_edit(roller);
        }
        return;
    }

    if(code != LV_EVENT_CLICKED) return;
    if(g_group_admin == NULL || lv_group_get_focused(g_group_admin) != roller) return;

    if(lv_group_get_editing(g_group_admin)) {
        admin_dormancy_roller_exit_edit(roller);
    }
    else {
        uint32_t sel = lv_roller_get_selected(roller);
        if(sel < DORMANCY_ROLLER_CNT) {
            lv_roller_set_selected(roller, sel, LV_ANIM_OFF);
        }
        lv_group_set_editing(g_group_admin, true);
    }
    lv_event_stop_processing(e);
}

/* 待机时间「确认」：写入 g_ui_dormancy_timeout_ms、刷新空闲定时器并回菜单 */

static void cb_admin_dormancy_confirm(lv_event_t * e)
{
    (void)e;
    if(g_admin_dormancy_roller == NULL) return;
    uint32_t idx = lv_roller_get_selected(g_admin_dormancy_roller);
    if(idx >= DORMANCY_ROLLER_CNT) idx = DORMANCY_ROLLER_DEFAULT_IDX;
    g_ui_dormancy_timeout_ms = g_dormancy_timeout_ms_tbl[idx];
    ui_idle_apply_dormancy_period();
    admin_dormancy_back_to_menu1();
}

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
#define ADMIN_SW_BORDER_W             3  // 描边宽度（px）
#define ADMIN_SW_KNOB_SIZE           23  // 中间圆圈直径（px）
#define ADMIN_SW_PAD_MAIN             1  // 轨道内边距（px）
#define ADMIN_SW_SIZE_W              70  // 开关总宽度（px）
#define ADMIN_SW_SIZE_H              35  // 开关总高度（px）

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
    admin_panel_apply_switch_layout(g_admin_sw_run_always_on, g_admin_lbl_brightness_title);
}

/* 4G 设置页：刷新 4G 开关布局 */

static void admin_4g_apply_switch_layout(void)
{
    admin_panel_apply_switch_layout(g_admin_sw_4g, g_admin_lbl_4g_title);
}

/* 亮度滑条数值对齐到步进 */

static int32_t admin_brightness_snap_slider(int32_t v)
{
    if(v < 0) v = 0;
    if(v > 100) v = 100;
    return (int32_t)ui_screen_brightness_snap((uint8_t)v);
}

/* 屏幕亮度页：按 ui_screen_run_always_on_get 刷新常亮开关状态 */

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

static void admin_brightness_slider_sync_bar(int32_t v)
{
    if(g_admin_brightness_bar == NULL) return;
    v = admin_brightness_snap_slider(v);
    lv_bar_set_value(g_admin_brightness_bar, v, LV_ANIM_OFF);
}

/* 将编码器焦点态映射到上层焦点框 */

static void admin_brightness_slider_sync_focus_frame(void)
{
    if(g_admin_brightness_slider_focus == NULL || g_admin_slider_brightness == NULL) return;
    const bool enc_focus = lv_obj_has_state(g_admin_slider_brightness, LV_STATE_FOCUSED) ||
                           lv_obj_has_state(g_admin_slider_brightness, LV_STATE_FOCUS_KEY);
    if(enc_focus) {
        lv_obj_add_state(g_admin_brightness_slider_focus, LV_STATE_FOCUSED);
        lv_obj_add_state(g_admin_brightness_slider_focus, LV_STATE_FOCUS_KEY);
    }
    else {
        lv_obj_remove_state(g_admin_brightness_slider_focus, LV_STATE_FOCUSED | LV_STATE_FOCUS_KEY);
    }
}

static void cb_admin_brightness_slider_focus_frame(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code != LV_EVENT_FOCUSED && code != LV_EVENT_DEFOCUSED) return;
    if(code == LV_EVENT_FOCUSED && g_admin_slider_brightness != NULL) {
        lv_obj_add_state(g_admin_slider_brightness, LV_STATE_FOCUS_KEY);
    }
    admin_brightness_slider_sync_focus_frame();
}

/* 屏幕亮度页：按 ui_screen_brightness_get 刷新滑动条（不触发硬件回调） */

static void admin_brightness_sync_slider_ui(void)
{
    if(g_admin_slider_brightness == NULL) return;
    g_admin_brightness_ui_loading = true;
    int32_t v = (int32_t)ui_screen_brightness_get();
    lv_slider_set_value(g_admin_slider_brightness, v, LV_ANIM_OFF);
    admin_brightness_slider_sync_bar(v);
    g_admin_brightness_ui_loading = false;
}

/* 屏幕亮度页：刷新开关与滑条 */

static void admin_brightness_sync_ui(void)
{
    admin_brightness_apply_switch_layout();
    admin_brightness_sync_switch_ui();
    admin_brightness_sync_slider_ui();
}

/* 常亮开关切换：更新状态并通知硬件（本期不改变熄屏逻辑） */

static void cb_admin_brightness_switch_changed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if(g_admin_view != SCREEN_BRIGHTNESS) return;
    lv_obj_t * sw = lv_event_get_target_obj(e);
    bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);
    ui_screen_run_always_on_set(on);
}

/* 亮度滑条拖动：按步进 10 对齐并通知硬件 */

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
    admin_brightness_slider_sync_bar(snapped);
}

/*
 * 亮度滑条退出编码器编辑态。
 * 短按第二次 / 失焦时调用；先对齐步进再 editing=false，避免退出后数值未 snap。
 */

static void admin_brightness_slider_exit_edit(lv_obj_t * slider)
{
    if(g_group_admin == NULL || slider == NULL) return;
    if(lv_group_get_focused(g_group_admin) != slider) return;
    if(!lv_group_get_editing(g_group_admin)) return;
    int32_t snapped = admin_brightness_snap_slider(lv_slider_get_value(slider));
    if(snapped != lv_slider_get_value(slider)) {
        g_admin_brightness_ui_loading = true;
        lv_slider_set_value(slider, snapped, LV_ANIM_OFF);
        g_admin_brightness_ui_loading = false;
    }
    lv_group_set_editing(g_group_admin, false);
}

/*
 * 亮度滑条编码器编辑态：拦截 LV_KEY_LEFT/RIGHT，每次旋转按 UI_SCREEN_BRIGHTNESS_STEP 调值。
 * 须在 LV_EVENT_PREPROCESS 注册，避免 LVGL 默认每次只 ±1。
 */

static void cb_admin_brightness_slider_key(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_KEY) return;
    if(g_admin_view != SCREEN_BRIGHTNESS) return;
    if(g_group_admin == NULL || !lv_group_get_editing(g_group_admin)) return;

    lv_obj_t * slider = lv_event_get_target_obj(e);
    if(slider == NULL || slider != g_admin_slider_brightness) return;
    if(lv_group_get_focused(g_group_admin) != slider) return;

    uint32_t c = lv_event_get_key(e);
    int32_t v = lv_slider_get_value(slider);
    if(c == LV_KEY_RIGHT || c == LV_KEY_UP) {
        v += UI_SCREEN_BRIGHTNESS_STEP;
    }
    else if(c == LV_KEY_LEFT || c == LV_KEY_DOWN) {
        v -= UI_SCREEN_BRIGHTNESS_STEP;
    }
    else {
        return;
    }

    v = admin_brightness_snap_slider(v);
    g_admin_brightness_ui_loading = true;
    lv_slider_set_value(slider, v, LV_ANIM_OFF);
    g_admin_brightness_ui_loading = false;
    ui_screen_brightness_set((uint8_t)v);
    admin_brightness_slider_sync_bar(v);
    lv_event_stop_processing(e);
}

/*
 * 亮度滑条：外设编码器短按发 LV_EVENT_CLICKED，在编辑/导航间切换。
 * 导航态：旋转在返回/启停/电源/常亮开关/滑条间移动焦点；短按进入编辑。
 * 编辑态：旋转调亮度；再短按退回导航；失焦自动退出编辑。
 */

static void cb_admin_brightness_slider_encoder(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target_obj(e);
    lv_event_code_t code = lv_event_get_code(e);

    if(g_admin_view != SCREEN_BRIGHTNESS) return;
    if(slider == NULL || lv_obj_has_flag(slider, LV_OBJ_FLAG_HIDDEN)) return;

    if(code == LV_EVENT_DEFOCUSED) {
        if(g_group_admin != NULL && lv_group_get_editing(g_group_admin)) {
            admin_brightness_slider_exit_edit(slider);
        }
        return;
    }

    if(code != LV_EVENT_CLICKED) return;
    if(g_group_admin == NULL || lv_group_get_focused(g_group_admin) != slider) return;

    if(lv_group_get_editing(g_group_admin)) {
        admin_brightness_slider_exit_edit(slider);
    }
    else {
        lv_group_set_editing(g_group_admin, true);
    }
    lv_event_stop_processing(e);
}

/* 离开屏幕亮度子页：回管理员 8 宫格菜单 */

static void admin_brightness_back_to_menu1(void)
{
    if(g_group_admin != NULL) {
        lv_group_set_editing(g_group_admin, false);
    }
    admin_panel_show(MENU1);
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

/* 声音控制页：按 ui_touch_sound_get 刷新触控声音开关 */

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

/* 同步音量下层渐变动条（与滑条数值一致） */

static void admin_sound_volume_slider_sync_bar(int32_t v)
{
    if(g_admin_sound_volume_bar == NULL) return;
    v = admin_sound_snap_volume_slider(v);
    lv_bar_set_value(g_admin_sound_volume_bar, v, LV_ANIM_OFF);
}

/* 同步触控声音下层渐变动条（与滑条数值一致） */

static void admin_sound_touch_sound_volume_slider_sync_bar(int32_t v)
{
    if(g_admin_touch_sound_volume_bar == NULL) return;
    v = admin_sound_snap_touch_sound_volume_slider(v);
    lv_bar_set_value(g_admin_touch_sound_volume_bar, v, LV_ANIM_OFF);
}

/* 将编码器焦点态映射到音量滑条上层焦点框 */

static void admin_sound_volume_slider_sync_focus_frame(void)
{
    if(g_admin_sound_volume_slider_focus == NULL || g_admin_slider_sound_volume == NULL) return;
    const bool enc_focus = lv_obj_has_state(g_admin_slider_sound_volume, LV_STATE_FOCUSED) ||
                           lv_obj_has_state(g_admin_slider_sound_volume, LV_STATE_FOCUS_KEY);
    if(enc_focus) {
        lv_obj_add_state(g_admin_sound_volume_slider_focus, LV_STATE_FOCUSED);
        lv_obj_add_state(g_admin_sound_volume_slider_focus, LV_STATE_FOCUS_KEY);
    }
    else {
        lv_obj_remove_state(g_admin_sound_volume_slider_focus, LV_STATE_FOCUSED | LV_STATE_FOCUS_KEY);
    }
}

/* 将编码器焦点态映射到触控声音滑条上层焦点框 */

static void admin_sound_touch_sound_volume_slider_sync_focus_frame(void)
{
    if(g_admin_touch_sound_volume_slider_focus == NULL || g_admin_slider_touch_sound_volume == NULL) return;
    const bool enc_focus = lv_obj_has_state(g_admin_slider_touch_sound_volume, LV_STATE_FOCUSED) ||
                           lv_obj_has_state(g_admin_slider_touch_sound_volume, LV_STATE_FOCUS_KEY);
    if(enc_focus) {
        lv_obj_add_state(g_admin_touch_sound_volume_slider_focus, LV_STATE_FOCUSED);
        lv_obj_add_state(g_admin_touch_sound_volume_slider_focus, LV_STATE_FOCUS_KEY);
    }
    else {
        lv_obj_remove_state(g_admin_touch_sound_volume_slider_focus, LV_STATE_FOCUSED | LV_STATE_FOCUS_KEY);
    }
}

/* 音量滑条获焦/失焦：刷新上层白色焦点框 */

static void cb_admin_sound_volume_slider_focus_frame(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code != LV_EVENT_FOCUSED && code != LV_EVENT_DEFOCUSED) return;
    if(code == LV_EVENT_FOCUSED && g_admin_slider_sound_volume != NULL) {
        lv_obj_add_state(g_admin_slider_sound_volume, LV_STATE_FOCUS_KEY);
    }
    admin_sound_volume_slider_sync_focus_frame();
}

/* 触控声音滑条获焦/失焦：刷新上层白色焦点框 */

static void cb_admin_sound_touch_sound_volume_slider_focus_frame(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code != LV_EVENT_FOCUSED && code != LV_EVENT_DEFOCUSED) return;
    if(code == LV_EVENT_FOCUSED && g_admin_slider_touch_sound_volume != NULL) {
        lv_obj_add_state(g_admin_slider_touch_sound_volume, LV_STATE_FOCUS_KEY);
    }
    admin_sound_touch_sound_volume_slider_sync_focus_frame();
}

/* 声音控制页：按 ui_sound_volume_get 刷新音量滑条（不触发硬件回调） */

static void admin_sound_sync_volume_slider_ui(void)
{
    if(g_admin_slider_sound_volume == NULL) return;
    g_admin_sound_volume_ui_loading = true;
    int32_t v = (int32_t)ui_sound_volume_get();
    lv_slider_set_value(g_admin_slider_sound_volume, v, LV_ANIM_OFF);
    admin_sound_volume_slider_sync_bar(v);
    g_admin_sound_volume_ui_loading = false;
}

/* 声音控制页：按 ui_touch_sound_volume_get 刷新触控声音滑条（不触发硬件回调） */

static void admin_sound_sync_touch_sound_volume_slider_ui(void)
{
    if(g_admin_slider_touch_sound_volume == NULL) return;
    g_admin_touch_sound_volume_ui_loading = true;
    int32_t v = (int32_t)ui_touch_sound_volume_get();
    lv_slider_set_value(g_admin_slider_touch_sound_volume, v, LV_ANIM_OFF);
    admin_sound_touch_sound_volume_slider_sync_bar(v);
    g_admin_touch_sound_volume_ui_loading = false;
}

/* 声音控制页：刷新两个开关与两条滑条 */

static void admin_sound_sync_ui(void)
{
    admin_sound_apply_switch_layout(g_admin_sw_touch_sound, g_admin_lbl_sound_line1);
    admin_sound_apply_switch_layout(g_admin_sw_voice_broadcast, g_admin_lbl_sound_line2);
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

/* 音量滑条拖动：按步进 10 对齐并通知硬件 */

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

/* 触控声音滑条拖动：按步进 10 对齐并通知硬件 */

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

/*
 * 音量滑条退出编码器编辑态。
 * 短按第二次 / 失焦时调用；先对齐步进再 editing=false。
 */

static void admin_sound_volume_slider_exit_edit(lv_obj_t * slider)
{
    if(g_group_admin == NULL || slider == NULL) return;
    if(lv_group_get_focused(g_group_admin) != slider) return;
    if(!lv_group_get_editing(g_group_admin)) return;
    int32_t snapped = admin_sound_snap_volume_slider(lv_slider_get_value(slider));
    if(snapped != lv_slider_get_value(slider)) {
        g_admin_sound_volume_ui_loading = true;
        lv_slider_set_value(slider, snapped, LV_ANIM_OFF);
        g_admin_sound_volume_ui_loading = false;
    }
    lv_group_set_editing(g_group_admin, false);
}

/* 触控声音滑条退出编码器编辑态（逻辑同音量滑条） */

static void admin_sound_touch_sound_volume_slider_exit_edit(lv_obj_t * slider)
{
    if(g_group_admin == NULL || slider == NULL) return;
    if(lv_group_get_focused(g_group_admin) != slider) return;
    if(!lv_group_get_editing(g_group_admin)) return;
    int32_t snapped = admin_sound_snap_touch_sound_volume_slider(lv_slider_get_value(slider));
    if(snapped != lv_slider_get_value(slider)) {
        g_admin_touch_sound_volume_ui_loading = true;
        lv_slider_set_value(slider, snapped, LV_ANIM_OFF);
        g_admin_touch_sound_volume_ui_loading = false;
    }
    lv_group_set_editing(g_group_admin, false);
}

/*
 * 音量滑条编码器编辑态：拦截 LV_KEY_LEFT/RIGHT，每次旋转按 UI_SOUND_VOLUME_STEP 调值。
 * 须在 LV_EVENT_PREPROCESS 注册，避免 LVGL 默认每次只 ±1。
 */

static void cb_admin_sound_volume_slider_key(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_KEY) return;
    if(g_admin_view != SOUND_CONTROL) return;
    if(g_group_admin == NULL || !lv_group_get_editing(g_group_admin)) return;

    lv_obj_t * slider = lv_event_get_target_obj(e);
    if(slider == NULL || slider != g_admin_slider_sound_volume) return;
    if(lv_group_get_focused(g_group_admin) != slider) return;

    uint32_t c = lv_event_get_key(e);
    int32_t v = lv_slider_get_value(slider);
    if(c == LV_KEY_RIGHT || c == LV_KEY_UP) {
        v += UI_SOUND_VOLUME_STEP;
    }
    else if(c == LV_KEY_LEFT || c == LV_KEY_DOWN) {
        v -= UI_SOUND_VOLUME_STEP;
    }
    else {
        return;
    }

    v = admin_sound_snap_volume_slider(v);
    g_admin_sound_volume_ui_loading = true;
    lv_slider_set_value(slider, v, LV_ANIM_OFF);
    g_admin_sound_volume_ui_loading = false;
    ui_sound_volume_set((uint8_t)v);
    admin_sound_volume_slider_sync_bar(v);
    lv_event_stop_processing(e);
}

/*
 * 触控声音滑条编码器编辑态：拦截 LV_KEY_LEFT/RIGHT，每次旋转按 UI_TOUCH_SOUND_VOLUME_STEP 调值。
 */

static void cb_admin_sound_touch_sound_volume_slider_key(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_KEY) return;
    if(g_admin_view != SOUND_CONTROL) return;
    if(g_group_admin == NULL || !lv_group_get_editing(g_group_admin)) return;

    lv_obj_t * slider = lv_event_get_target_obj(e);
    if(slider == NULL || slider != g_admin_slider_touch_sound_volume) return;
    if(lv_group_get_focused(g_group_admin) != slider) return;

    uint32_t c = lv_event_get_key(e);
    int32_t v = lv_slider_get_value(slider);
    if(c == LV_KEY_RIGHT || c == LV_KEY_UP) {
        v += UI_TOUCH_SOUND_VOLUME_STEP;
    }
    else if(c == LV_KEY_LEFT || c == LV_KEY_DOWN) {
        v -= UI_TOUCH_SOUND_VOLUME_STEP;
    }
    else {
        return;
    }

    v = admin_sound_snap_touch_sound_volume_slider(v);
    g_admin_touch_sound_volume_ui_loading = true;
    lv_slider_set_value(slider, v, LV_ANIM_OFF);
    g_admin_touch_sound_volume_ui_loading = false;
    ui_touch_sound_volume_set((uint8_t)v);
    admin_sound_touch_sound_volume_slider_sync_bar(v);
    lv_event_stop_processing(e);
}

/*
 * 音量滑条：编码器短按在编辑/导航间切换。
 * 导航态：旋转移动焦点；短按进入编辑。编辑态：旋转调值；再短按退回导航。
 */

static void cb_admin_sound_volume_slider_encoder(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target_obj(e);
    lv_event_code_t code = lv_event_get_code(e);

    if(g_admin_view != SOUND_CONTROL) return;
    if(slider == NULL || lv_obj_has_flag(slider, LV_OBJ_FLAG_HIDDEN)) return;

    if(code == LV_EVENT_DEFOCUSED) {
        if(g_group_admin != NULL && lv_group_get_editing(g_group_admin)) {
            admin_sound_volume_slider_exit_edit(slider);
        }
        return;
    }

    if(code != LV_EVENT_CLICKED) return;
    if(g_group_admin == NULL || lv_group_get_focused(g_group_admin) != slider) return;

    if(lv_group_get_editing(g_group_admin)) {
        admin_sound_volume_slider_exit_edit(slider);
    }
    else {
        lv_group_set_editing(g_group_admin, true);
    }
    lv_event_stop_processing(e);
}

/* 触控声音滑条：编码器短按切换编辑/导航（逻辑同音量滑条） */

static void cb_admin_sound_touch_sound_volume_slider_encoder(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target_obj(e);
    lv_event_code_t code = lv_event_get_code(e);

    if(g_admin_view != SOUND_CONTROL) return;
    if(slider == NULL || lv_obj_has_flag(slider, LV_OBJ_FLAG_HIDDEN)) return;

    if(code == LV_EVENT_DEFOCUSED) {
        if(g_group_admin != NULL && lv_group_get_editing(g_group_admin)) {
            admin_sound_touch_sound_volume_slider_exit_edit(slider);
        }
        return;
    }

    if(code != LV_EVENT_CLICKED) return;
    if(g_group_admin == NULL || lv_group_get_focused(g_group_admin) != slider) return;

    if(lv_group_get_editing(g_group_admin)) {
        admin_sound_touch_sound_volume_slider_exit_edit(slider);
    }
    else {
        lv_group_set_editing(g_group_admin, true);
    }
    lv_event_stop_processing(e);
}

/* 厂商维护：出厂序列号页返回 menu1 */

static void admin_vendor_serial_back_to_menu1(void)
{
    if(g_admin_ta_vendor_serial != NULL) {
        lv_textarea_set_text(g_admin_ta_vendor_serial, "");
    }
    if(g_admin_kb != NULL) {
        g_admin_kb_ta = NULL;
        lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
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

static void cb_admin_open_vendor_maint(lv_event_t * e)
{
    (void)e;
    admin_panel_show(VENDOR_SERIAL);
}

static void cb_admin_open_selfcheck(lv_event_t * e)
{
    (void)e;
    selfcheck_ui_reset();
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

/* 语言页按钮：选中=橙色填充+白字，未选中=橙色描边+白字 */

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

/* 语言页中/英按钮与当前 g_ui_lang 对齐 */

static void admin_lang_sync_btn_ui(void)
{
    bool zh = (ui_lang_get() == UI_LANG_ZH);
    admin_lang_btn_set_selected(g_admin_btn_lang_zh, zh);
    admin_lang_btn_set_selected(g_admin_btn_lang_en, !zh);
}

/* 离开语言设置页：回管理员菜单 */

static void admin_lang_back_to_menu1(void)
{
    admin_panel_show(MENU1);
}

/* 管理员菜单「语言设置」入口 */

static void cb_admin_open_language_settings(lv_event_t * e)
{
    (void)e;
    admin_panel_show(LANGUAGE_SETTINGS);
}

/* 语言页「中文」：已是中文则无操作 */

static void cb_admin_lang_zh(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != LANGUAGE_SETTINGS) return;
    if(ui_lang_get() == UI_LANG_ZH) return;
    ui_lang_set(UI_LANG_ZH);
    admin_lang_sync_btn_ui();
}

/* 语言页「英文」：已是英文则无操作 */

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
    ui_idle_apply_dormancy_period();
    g_machine_id = 1u;
    g_ui_lang = UI_LANG_ZH;
    ui_lang_apply_all();
    ui_auto_dispense_set(true);
    ui_ozone_set(true);
    ui_fresh_air_care_set(true);
    ui_screen_run_always_on_set(false);
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
    ui_4g_set(false);
    ui_data_upload_basic_set(true);
    ui_data_upload_sensor_set(true);
    ui_data_upload_fault_set(true);
    ui_data_upload_auto_dispense_set(false);
    ui_data_upload_payment_order_set(false);
    ui_data_upload_user_op_set(false);
    ui_data_upload_device_set(true);
    ui_data_upload_strategy_set(UI_DATA_STRATEGY_4G_ONLY);
}

/* 恢复默认子页：切换确认前 / 恢复中 / 完成 的控件可见性与状态文案 */

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
        }
    }

    if(g_group_admin != NULL) {
        lv_group_set_editing(g_group_admin, false);
    }
    admin_encoder_rebuild();
}

/* 进入恢复默认页：停止定时器并复位为确认前态 */

static void admin_factory_reset_ui_enter(void)
{
    admin_factory_timer_stop();
    admin_factory_set_phase(ADMIN_FACTORY_PHASE_PROMPT);
}

/* 离开恢复默认页：回管理员 8 宫格菜单 */

static void admin_factory_back_to_menu1(void)
{
    admin_factory_timer_stop();
    if(g_group_admin != NULL) {
        lv_group_set_editing(g_group_admin, false);
    }
    admin_panel_show(MENU1);
}

/* 恢复默认「取消」：返回管理员菜单 */

static void cb_admin_factory_cancel(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    admin_factory_back_to_menu1();
}

/* 2s 定时器：恢复中 → 完成态 */

static void cb_admin_factory_timer(lv_timer_t * t)
{
    (void)t;
    g_admin_factory_timer = NULL;
    if(g_admin_view == FACTORY_RESET) {
        admin_factory_set_phase(ADMIN_FACTORY_PHASE_DONE);
    }
}

/* 恢复默认「确定」：进入恢复中 UI 并写回出厂数据，2s 后显示完成 */

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

/* 离开联系我们页：回管理员 8 宫格菜单 */

static void admin_contact_back_to_menu1(void)
{
    admin_panel_show(MENU1);
}

/* 管理员菜单「联系我们」入口 */

static void cb_admin_open_contact_us(lv_event_t * e)
{
    (void)e;
    admin_panel_show(CONTACT_US);
}

/* 自投功能页：按 ui_auto_dispense_get 刷新「开启」「关闭」橙色填充/描边 */

static void admin_auto_dispense_sync_btn_ui(void)
{
    bool on = ui_auto_dispense_get();
    admin_lang_btn_set_selected(g_admin_btn_auto_dispense_on, on);
    admin_lang_btn_set_selected(g_admin_btn_auto_dispense_off, !on);
}

/* 离开自投功能页：回管理员 menu2 */

static void admin_auto_dispense_back_to_menu2(void)
{
    admin_panel_show(MENU2);
}

/* 管理员 menu2「自投功能」入口 */

static void cb_admin_open_auto_dispense(lv_event_t * e)
{
    (void)e;
    admin_panel_show(AUTO_DISPENSE);
}

/* 自投功能「开启」：已是开启则无操作，否则 set 并通知硬件 */

static void cb_admin_auto_dispense_on(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != AUTO_DISPENSE) return;
    (void)ui_auto_dispense_set(true);
}

/* 自投功能「关闭」：已是关闭则无操作，否则 set 并通知硬件 */

static void cb_admin_auto_dispense_off(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != AUTO_DISPENSE) return;
    (void)ui_auto_dispense_set(false);
}

/* 臭氧功能页：按 ui_ozone_get 刷新「开启」「关闭」橙色填充/描边 */

static void admin_ozone_sync_btn_ui(void)
{
    bool on = ui_ozone_get();
    admin_lang_btn_set_selected(g_admin_btn_ozone_on, on);
    admin_lang_btn_set_selected(g_admin_btn_ozone_off, !on);
}

/* 离开臭氧功能页：回管理员 menu2 */

static void admin_ozone_back_to_menu2(void)
{
    admin_panel_show(MENU2);
}

/* 管理员 menu2「臭氧功能」入口 */

static void cb_admin_open_ozone(lv_event_t * e)
{
    (void)e;
    admin_panel_show(OZONE);
}

/* 臭氧功能「开启」：已是开启则无操作，否则 set 并通知硬件 */

static void cb_admin_ozone_on(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != OZONE) return;
    (void)ui_ozone_set(true);
}

/* 臭氧功能「关闭」：已是关闭则无操作，否则 set 并通知硬件 */

static void cb_admin_ozone_off(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if(g_admin_view != OZONE) return;
    (void)ui_ozone_set(false);
}

/* 新风护理页：按 ui_fresh_air_care_get 刷新「开启」「关闭」橙色填充/描边 */

static void admin_fresh_air_care_sync_btn_ui(void)
{
    bool on = ui_fresh_air_care_get();
    admin_lang_btn_set_selected(g_admin_btn_fresh_air_care_on, on);
    admin_lang_btn_set_selected(g_admin_btn_fresh_air_care_off, !on);
}

/* 离开新风护理页：回管理员 menu2 */

static void admin_fresh_air_care_back_to_menu2(void)
{
    admin_panel_show(MENU2);
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

/* 系统升级子页：切换确认前 / 升级中 / 完成 的控件可见性与状态文案 */

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
        }
    }

    if(g_group_admin != NULL) {
        lv_group_set_editing(g_group_admin, false);
    }
    admin_encoder_rebuild();
}

/* 进入系统升级页：停止定时器并复位为确认前态 */

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
        lv_group_set_editing(g_group_admin, false);
    }
    admin_panel_show(MENU2);
}

/* 2s 定时器：升级中 → 完成态 */

static void cb_admin_system_upgrade_timer(lv_timer_t * t)
{
    (void)t;
    g_admin_system_upgrade_timer = NULL;
    if(g_admin_view == SYSTEM_UPGRADE) {
        admin_system_upgrade_set_phase(ADMIN_SYSTEM_UPGRADE_PHASE_DONE);
    }
}

/* 系统升级「确认」：进入升级中 UI，2s 后显示完成 */

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

/* 停止 4G 配网页的 2s 完成态定时器（离开页或再次进入前调用） */

static void admin_4g_timer_stop(void)
{
    if(g_admin_4g_timer != NULL) {
        lv_timer_delete(g_admin_4g_timer);
        g_admin_4g_timer = NULL;
    }
}

/* 4G 设置页：按 ui_4g_get 刷新开关状态（不触发配网流程） */

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

/* 4G 设置子页：切换说明 / 配网中 / 成功 的控件可见性 */

static void admin_4g_set_phase(admin_4g_phase_t phase)
{
    g_admin_4g_phase = phase;
    if(g_admin_view != SETTINGS_4G) return;

    const bool prompt = (phase == ADMIN_4G_PHASE_PROMPT);
    const bool provisioning = (phase == ADMIN_4G_PHASE_PROVISIONING);
    const bool done = (phase == ADMIN_4G_PHASE_DONE);

    if(g_admin_lbl_4g_title != NULL) {
        if(done) lv_obj_add_flag(g_admin_lbl_4g_title, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_remove_flag(g_admin_lbl_4g_title, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_sw_4g != NULL) {
        if(done) lv_obj_add_flag(g_admin_sw_4g, LV_OBJ_FLAG_HIDDEN);
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
        if(done) lv_obj_remove_flag(g_admin_4g_done_center, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_admin_4g_done_center, LV_OBJ_FLAG_HIDDEN);
    }

    if(g_group_admin != NULL) {
        lv_group_set_editing(g_group_admin, false);
    }
    admin_encoder_rebuild();
}

/* 待机/支付 roller 选项随语言切换（保持当前选中项） */

static void ui_lang_refresh_rollers(void)
{
    if(g_admin_dormancy_roller != NULL) {
        uint32_t sel = lv_roller_get_selected(g_admin_dormancy_roller);
        lv_roller_set_options(g_admin_dormancy_roller, ui_translation(STR_DORMANCY_ROLLER), LV_ROLLER_MODE_NORMAL);
        if(sel < DORMANCY_ROLLER_CNT) {
            lv_roller_set_selected(g_admin_dormancy_roller, sel, LV_ANIM_OFF);
        }
    }
    if(g_admin_payment_timeout_roller != NULL) {
        uint32_t sel = lv_roller_get_selected(g_admin_payment_timeout_roller);
        lv_roller_set_options(g_admin_payment_timeout_roller, ui_translation(STR_PAYMENT_TIMEOUT_ROLLER),
                              LV_ROLLER_MODE_NORMAL);
        if(sel < PAYMENT_TIMEOUT_ROLLER_CNT) {
            lv_roller_set_selected(g_admin_payment_timeout_roller, sel, LV_ANIM_OFF);
        }
    }
}

/* 程序设置页：字段名与程序 Tab 文案 */

static void program_admin_refresh_i18n(void)
{
    unsigned i;
    for(i = 0; i < TOTAL_PROGRAMS; i++) {
        if(g_admin_prog_btns[i] != NULL) {
            lv_obj_t * lbl = lv_obj_get_child(g_admin_prog_btns[i], 0);
            if(lbl != NULL) {
                lv_label_set_text(lbl, ui_translation(g_mode_name_ids[i]));
            }
        }
    }
    if(g_admin_prog_water_lbl != NULL && g_admin_prog_sel >= 0 && g_admin_prog_sel < TOTAL_PROGRAMS) {
        const ui_program_admin_t * c = &g_prog_cfg[g_admin_prog_sel];
        if(c->cap & PROG_CAP_WATER) {
            lv_label_set_text(g_admin_prog_water_lbl, ui_translation(STR_PROG_WATER_SMART));
        }
        else {
            lv_label_set_text(g_admin_prog_water_lbl, "--");
        }
    }
}

/* 恢复出厂/系统升级/4G 等可见动态状态文案 */

static void admin_refresh_visible_status_text(void)
{
    if(g_admin_lbl_msg_pwd != NULL && g_admin_pwd_err_id < STR_COUNT
       && !lv_obj_has_flag(g_admin_lbl_msg_pwd, LV_OBJ_FLAG_HIDDEN)) {
        lv_label_set_text(g_admin_lbl_msg_pwd, ui_translation(g_admin_pwd_err_id));
    }
    if(g_admin_lbl_msg_pwd_chg_old != NULL && g_admin_pwd_chg_old_err_id < STR_COUNT
       && !lv_obj_has_flag(g_admin_lbl_msg_pwd_chg_old, LV_OBJ_FLAG_HIDDEN)) {
        lv_label_set_text(g_admin_lbl_msg_pwd_chg_old, ui_translation(g_admin_pwd_chg_old_err_id));
    }
    if(g_admin_lbl_msg_pwd_chg_new != NULL && g_admin_pwd_chg_new_msg_id < STR_COUNT
       && !lv_obj_has_flag(g_admin_lbl_msg_pwd_chg_new, LV_OBJ_FLAG_HIDDEN)) {
        lv_label_set_text(g_admin_lbl_msg_pwd_chg_new, ui_translation(g_admin_pwd_chg_new_msg_id));
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
    admin_dormancy_label_update();
    admin_machine_id_label_update();
    program_admin_refresh_i18n();
    admin_payment_sync_checkbox_ui();
    admin_data_sync_upload_items_ui();
    admin_data_sync_strategy_ui();
    admin_refresh_visible_status_text();
    carousel_update_card_images();
    running_screen_sync_mode_name();
    running_status_sync_labels();
    alarm_content_update();
    alarm_fault_panels_relayout_content();
    home_sync_program_labels();
    pay_sync_price_label();
    pay_sync_pay_ui();
    cycle_run_count_label_sync();
}

/* 进入 4G 设置页：停止定时器、复位为说明态并同步开关 */

static void admin_4g_ui_enter(void)
{
    admin_4g_timer_stop();
    admin_4g_apply_switch_layout();
    admin_4g_sync_switch_ui();
    admin_4g_set_phase(ADMIN_4G_PHASE_PROMPT);
}

/* 离开 4G 设置页：回 menu2 */

static void admin_4g_back_to_menu2(void)
{
    admin_4g_timer_stop();
    if(g_group_admin != NULL) {
        lv_group_set_editing(g_group_admin, false);
    }
    admin_panel_show(MENU2);
}

/* 2s 定时器：配网中 → 成功态 */

static void cb_admin_4g_timer(lv_timer_t * t)
{
    (void)t;
    g_admin_4g_timer = NULL;
    if(g_admin_view == SETTINGS_4G) {
        admin_4g_set_phase(ADMIN_4G_PHASE_DONE);
    }
}

/* 4G 开关 OFF→ON：进入配网中 UI，2s 后显示成功 */

static void admin_4g_start_provisioning(void)
{
    admin_4g_timer_stop();
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

/* menu2「网络设置」入口 */

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

/* 离开网络设置页：回 menu2 */

static void admin_network_back_to_menu2(void)
{
    if(g_group_admin != NULL) {
        lv_group_set_editing(g_group_admin, false);
    }
    admin_panel_show(MENU2);
}

/* 离开 WIFI 设置页：回网络设置 */

static void admin_wifi_back_to_network(void)
{
    if(g_group_admin != NULL) {
        lv_group_set_editing(g_group_admin, false);
    }
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

static uint32_t ui_payment_timeout_roller_index_from_sec(uint16_t sec)
{
    for(uint32_t i = 0; i < PAYMENT_TIMEOUT_ROLLER_CNT; i++) {
        if(g_payment_timeout_sec_tbl[i] == sec) return i;
    }
    return PAYMENT_TIMEOUT_ROLLER_DEFAULT_IDX;
}

/* 支付设置页：按 g_ui_payment_* 刷新支付方式复选框与文案 */

static void cb_admin_payment_alipay_changed(lv_event_t * e)
{
    lv_obj_t * cb = lv_event_get_target(e);
    ui_payment_alipay_set(lv_obj_has_state(cb, LV_STATE_CHECKED));
    pay_sync_pay_ui();
}

static void cb_admin_payment_wechat_changed(lv_event_t * e)
{
    lv_obj_t * cb = lv_event_get_target(e);
    ui_payment_wechat_set(lv_obj_has_state(cb, LV_STATE_CHECKED));
    pay_sync_pay_ui();
}

static void admin_payment_sync_checkbox_ui(void)
{
    if(g_admin_cb_payment_alipay != NULL) {
        lv_checkbox_set_text(g_admin_cb_payment_alipay, ui_translation(STR_PAYMENT_ALIPAY));
        if(g_ui_payment_alipay_enabled) {
            lv_obj_add_state(g_admin_cb_payment_alipay, LV_STATE_CHECKED);
        }
        else {
            lv_obj_remove_state(g_admin_cb_payment_alipay, LV_STATE_CHECKED);
        }
    }
    if(g_admin_cb_payment_wechat != NULL) {
        lv_checkbox_set_text(g_admin_cb_payment_wechat, ui_translation(STR_PAYMENT_WECHAT));
        if(g_ui_payment_wechat_enabled) {
            lv_obj_add_state(g_admin_cb_payment_wechat, LV_STATE_CHECKED);
        }
        else {
            lv_obj_remove_state(g_admin_cb_payment_wechat, LV_STATE_CHECKED);
        }
    }
}

/* 支付设置页：按 g_ui_payment_timeout_sec 刷新支付超时 roller */

static void admin_payment_sync_timeout_roller_ui(void)
{
    if(g_admin_payment_timeout_roller == NULL) return;
    uint32_t idx = ui_payment_timeout_roller_index_from_sec(g_ui_payment_timeout_sec);
    lv_roller_set_options(g_admin_payment_timeout_roller, ui_translation(STR_PAYMENT_TIMEOUT_ROLLER),
                          LV_ROLLER_MODE_NORMAL);
    lv_roller_set_selected(g_admin_payment_timeout_roller, idx, LV_ANIM_OFF);
}

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

    for(i = 0; i < 7; i++) {
        if(g_admin_cb_data_upload[i] == NULL) continue;
        lv_checkbox_set_text(g_admin_cb_data_upload[i], ui_translation(g_data_upload_item_str_ids[i]));
        if(*vars[i]) {
            lv_obj_add_state(g_admin_cb_data_upload[i], LV_STATE_CHECKED);
        }
        else {
            lv_obj_remove_state(g_admin_cb_data_upload[i], LV_STATE_CHECKED);
        }
    }
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

/* 数据设置页：按 g_ui_data_upload_strategy 刷新右栏互斥复选框（选中一项，其余 unchecked+disabled） */

static void admin_data_sync_strategy_ui(void)
{
    unsigned i;

    g_admin_data_strategy_ui_loading = true;
    for(i = 0; i < 5; i++) {
        if(g_admin_cb_data_strategy[i] == NULL) continue;
        lv_checkbox_set_text(g_admin_cb_data_strategy[i], ui_translation(g_data_strategy_str_ids[i]));
        if((ui_data_upload_strategy_t)i == g_ui_data_upload_strategy) {
            lv_obj_add_state(g_admin_cb_data_strategy[i], LV_STATE_CHECKED);
            lv_obj_remove_state(g_admin_cb_data_strategy[i], LV_STATE_DISABLED);
        }
        else {
            lv_obj_remove_state(g_admin_cb_data_strategy[i], LV_STATE_CHECKED);
            lv_obj_add_state(g_admin_cb_data_strategy[i], LV_STATE_DISABLED);
        }
    }
    g_admin_data_strategy_ui_loading = false;
}

/* 由策略复选框对象反查下标；未找到返回 -1 */

static int admin_data_strategy_index_from_cb(lv_obj_t * cb)
{
    unsigned i;

    for(i = 0; i < 5; i++) {
        if(g_admin_cb_data_strategy[i] == cb) return (int)i;
    }
    return -1;
}

/* 数据设置页：策略项按下时先解除禁用，便于切换到其它策略 */

static void cb_admin_data_strategy_pressed(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_PRESSED) return;
    if(g_admin_view != DATA_SETTINGS) return;
    admin_data_strategy_enable_all();
}

/* 数据设置页：策略项 VALUE_CHANGED 互斥回调（仅更新 g_ui 状态，不接上传后端） */

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
    }
    else if((ui_data_upload_strategy_t)idx == g_ui_data_upload_strategy) {
        g_admin_data_strategy_ui_loading = true;
        lv_obj_add_state(cb, LV_STATE_CHECKED);
        g_admin_data_strategy_ui_loading = false;
    }
}

/*
 * 支付超时 roller 退出编码器编辑态。
 * 短按第二次编码器 / 失焦时调用；先同步 LVGL 内部 ori 再 editing=false，避免选项被回滚。
 */

static void admin_payment_timeout_roller_exit_edit(lv_obj_t * roller)
{
    if(g_group_admin == NULL || roller == NULL) return;
    if(lv_group_get_focused(g_group_admin) != roller) return;
    if(!lv_group_get_editing(g_group_admin)) return;
    uint32_t sel = lv_roller_get_selected(roller);
    if(sel < PAYMENT_TIMEOUT_ROLLER_CNT) {
        lv_roller_set_selected(roller, sel, LV_ANIM_OFF);
    }
    lv_group_set_editing(g_group_admin, false);
}

/*
 * 支付超时 roller：外设编码器短按发 LV_EVENT_CLICKED，在编辑/导航间切换。
 * 短按进入编辑（旋转改值），再按退回选择（旋转切焦点）；失焦时自动退出编辑。
 */

static void cb_admin_payment_timeout_roller_encoder(lv_event_t * e)
{
    lv_obj_t * roller = lv_event_get_target_obj(e);
    lv_event_code_t code = lv_event_get_code(e);

    if(g_admin_view != PAYMENT_SETTINGS) return;
    if(roller == NULL || lv_obj_has_flag(roller, LV_OBJ_FLAG_HIDDEN)) return;

    if(code == LV_EVENT_DEFOCUSED) {
        if(g_group_admin != NULL && lv_group_get_editing(g_group_admin)) {
            admin_payment_timeout_roller_exit_edit(roller);
        }
        return;
    }

    if(code != LV_EVENT_CLICKED) return;
    if(g_group_admin == NULL || lv_group_get_focused(g_group_admin) != roller) return;

    if(lv_group_get_editing(g_group_admin)) {
        admin_payment_timeout_roller_exit_edit(roller);
    }
    else {
        uint32_t sel = lv_roller_get_selected(roller);
        if(sel < PAYMENT_TIMEOUT_ROLLER_CNT) {
            lv_roller_set_selected(roller, sel, LV_ANIM_OFF);
        }
        lv_group_set_editing(g_group_admin, true);
    }
    lv_event_stop_processing(e);
}

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

/* 离开数据设置页：回管理员 menu2 */

static void admin_data_back_to_menu2(void)
{
    admin_panel_show(MENU2);
}

/* 管理员 menu2「数据设置」入口 */

static void cb_admin_open_data_settings(lv_event_t * e)
{
    (void)e;
    admin_panel_show(DATA_SETTINGS);
}

/* 管理员菜单按钮内文字 label（button → inner → label） */

static lv_obj_t * admin_menu_btn_get_label(lv_obj_t * btn)
{
    if(btn == NULL) return NULL;
    lv_obj_t * inner = lv_obj_get_child(btn, 0);
    if(inner == NULL) return NULL;
    return lv_obj_get_child(inner, 0);
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
    lv_obj_t * focused = lv_group_get_focused(g_group_admin);
    if(focused == g_admin_menu1_btns[7]) {
        admin_panel_show(MENU2);
    }
}

/* menu2 第 1 钮左转落到电源时：回到 menu1 第 8 钮 */

static void admin_group_focus_cb(lv_group_t * group)
{
    (void)group;
    if(g_group_admin == NULL) return;
    lv_obj_t * focused = lv_group_get_focused(g_group_admin);
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

/* 按当前管理员子页重建编码器 focus 顺序与初始焦点 */

static void admin_pwd_chg_enter(void)
{
    if(g_admin_ta_pwd_chg_old != NULL) lv_textarea_set_text(g_admin_ta_pwd_chg_old, "");
    if(g_admin_lbl_msg_pwd_chg_old != NULL) {
        lv_label_set_text(g_admin_lbl_msg_pwd_chg_old, "");
        lv_obj_add_flag(g_admin_lbl_msg_pwd_chg_old, LV_OBJ_FLAG_HIDDEN);
    }
    admin_panel_show(PASSWORD_CHANGE_OLD);
}

/* 密码修改步骤一：键盘 OK 校验原密码，成功进入双新密码页 */

static void admin_pwd_chg_old_try(void)
{
    if(g_admin_ta_pwd_chg_old == NULL) return;
    const char * t = lv_textarea_get_text(g_admin_ta_pwd_chg_old);
    if(t == NULL || lv_strlen(t) != 6) return;
    if(lv_strcmp(t, g_admin_pwd) != 0) {
        if(g_admin_lbl_msg_pwd_chg_old != NULL) {
            g_admin_pwd_chg_old_err_id = STR_PWD_WRONG_REENTER;
            lv_label_set_text(g_admin_lbl_msg_pwd_chg_old, ui_translation(STR_PWD_WRONG_REENTER));
            lv_obj_remove_flag(g_admin_lbl_msg_pwd_chg_old, LV_OBJ_FLAG_HIDDEN);
        }
        lv_textarea_set_text(g_admin_ta_pwd_chg_old, "");
        return;
    }
    lv_textarea_set_text(g_admin_ta_pwd_chg_old, "");
    if(g_admin_lbl_msg_pwd_chg_old != NULL) {
        lv_label_set_text(g_admin_lbl_msg_pwd_chg_old, "");
        lv_obj_add_flag(g_admin_lbl_msg_pwd_chg_old, LV_OBJ_FLAG_HIDDEN);
    }
    admin_pwd_chg_show_new();
}

/* 密码修改步骤二：切换到双新密码输入页 */

static void admin_pwd_chg_show_new(void)
{
    admin_panel_show(PASSWORD_CHANGE_NEW);
}

/* 密码修改步骤二：第一框 OK 进第二框；第二框 OK 比对并保存 */

static void admin_pwd_chg_new_on_ready(void)
{
    if(g_admin_ta_pwd_chg_new1 == NULL || g_admin_ta_pwd_chg_new2 == NULL) return;

    if(g_admin_pwd_chg_new_step == 0) {
        const char * t = lv_textarea_get_text(g_admin_ta_pwd_chg_new1);
        if(t == NULL || lv_strlen(t) != 6) return;
        g_admin_pwd_chg_new_step = 1;
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = g_admin_ta_pwd_chg_new2;
        }
        if(g_group_admin != NULL) {
            lv_group_focus_obj(g_admin_ta_pwd_chg_new2);
        }
        return;
    }

    const char * t1 = lv_textarea_get_text(g_admin_ta_pwd_chg_new1);
    const char * t2 = lv_textarea_get_text(g_admin_ta_pwd_chg_new2);
    if(t1 == NULL || t2 == NULL || lv_strlen(t1) != 6 || lv_strlen(t2) != 6) return;

    if(lv_strcmp(t1, t2) != 0) {
        if(g_admin_lbl_msg_pwd_chg_new != NULL) {
            g_admin_pwd_chg_new_msg_id = STR_PWD_MISMATCH;
            lv_label_set_text(g_admin_lbl_msg_pwd_chg_new, ui_translation(STR_PWD_MISMATCH));
            lv_obj_remove_flag(g_admin_lbl_msg_pwd_chg_new, LV_OBJ_FLAG_HIDDEN);
        }
        lv_textarea_set_text(g_admin_ta_pwd_chg_new1, "");
        lv_textarea_set_text(g_admin_ta_pwd_chg_new2, "");
        g_admin_pwd_chg_new_step = 0;
        if(g_admin_kb != NULL) {
            g_admin_kb_ta = g_admin_ta_pwd_chg_new1;
        }
        if(g_group_admin != NULL && g_admin_ta_pwd_chg_new1 != NULL) {
            lv_group_focus_obj(g_admin_ta_pwd_chg_new1);
        }
        return;
    }

    lv_strcpy(g_admin_pwd, t1);
    if(g_admin_lbl_msg_pwd_chg_new != NULL) {
        g_admin_pwd_chg_new_msg_id = STR_PWD_CHANGE_OK;
        lv_label_set_text(g_admin_lbl_msg_pwd_chg_new, ui_translation(STR_PWD_CHANGE_OK));
        lv_obj_remove_flag(g_admin_lbl_msg_pwd_chg_new, LV_OBJ_FLAG_HIDDEN);
    }
    lv_textarea_set_text(g_admin_ta_pwd_chg_new1, "");
    lv_textarea_set_text(g_admin_ta_pwd_chg_new2, "");
    g_admin_pwd_chg_new_step = 0;
    if(g_admin_kb != NULL && g_admin_ta_pwd_chg_new1 != NULL) {
        g_admin_kb_ta = g_admin_ta_pwd_chg_new1;
    }
}

/* 离开密码修改（步骤一）：回 menu2 */

static void admin_pwd_chg_back_to_menu2(void)
{
    if(g_admin_ta_pwd_chg_old != NULL) lv_textarea_set_text(g_admin_ta_pwd_chg_old, "");
    if(g_admin_lbl_msg_pwd_chg_old != NULL) {
        lv_label_set_text(g_admin_lbl_msg_pwd_chg_old, "");
        lv_obj_add_flag(g_admin_lbl_msg_pwd_chg_old, LV_OBJ_FLAG_HIDDEN);
    }
    if(g_admin_ta_pwd_chg_new1 != NULL) lv_textarea_set_text(g_admin_ta_pwd_chg_new1, "");
    if(g_admin_ta_pwd_chg_new2 != NULL) lv_textarea_set_text(g_admin_ta_pwd_chg_new2, "");
    if(g_admin_lbl_msg_pwd_chg_new != NULL) {
        lv_label_set_text(g_admin_lbl_msg_pwd_chg_new, "");
        lv_obj_add_flag(g_admin_lbl_msg_pwd_chg_new, LV_OBJ_FLAG_HIDDEN);
    }
    g_admin_pwd_chg_new_step = 0;
    admin_panel_show(MENU2);
}

/* menu2「密码修改」入口 */

static void cb_admin_open_password_change(lv_event_t * e)
{
    (void)e;
    admin_pwd_chg_enter();
}

//构建管理员页，顶栏返回/启停/电源 + 状态栏；密码 VENDOR_SERIAL_CODE，进入管理员设置页面
//密码子面板：密码标题/密码输入框/密码错误提示
//机器ID子面板：机器ID标题/机器ID当前值/机器ID输入框/确认按钮/机器ID错误提示
//程序设置子面板：
//屏幕亮度子面板：
//声音控制子面板：
//语言设置子面板：
//待机时间子面板：
//恢复默认子面板：
//联系我们子面板：

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

    lv_obj_t * top = create_top_bar(root, &g_lbl_clock_admin, NULL, NULL, NULL);

    LV_IMAGE_DECLARE(back);
    g_admin_btn_back = lv_imgbtn_create(top);
    lv_imgbtn_set_src(g_admin_btn_back, LV_IMGBTN_STATE_RELEASED, NULL, &back, NULL);
    lv_obj_align(g_admin_btn_back, LV_ALIGN_LEFT_MID, 20, 0);
    lv_obj_remove_flag(g_admin_btn_back, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(g_admin_btn_back, cb_admin_back, LV_EVENT_CLICKED, NULL);

    g_admin_btn_runpause = add_encoder_top_btn(top, "启停", 100, NULL);
    g_admin_btn_power = add_encoder_top_btn(top, "电源", 180, NULL);
    lv_obj_add_event_cb(g_admin_btn_power, cb_power_long, LV_EVENT_LONG_PRESSED, NULL);

    const lv_coord_t body_y = (lv_coord_t)(UI_FIXED_H * 10 / 100);
    const lv_coord_t body_h = (lv_coord_t)UI_FIXED_H - body_y - 200 + 60;

    /* 序列号面板 */
    g_admin_panel_pwd = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_pwd, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_pwd, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_pwd, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_pwd, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_pwd, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_pwd, LV_LAYOUT_NONE, LV_PART_MAIN);

    /* 标题：管理员设置 */
    lv_obj_t * lbl_admin_title = lv_label_create(g_admin_panel_pwd);
    lv_label_set_text(lbl_admin_title, "管理员设置");
    lv_obj_set_style_text_color(lbl_admin_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_admin_title, s_font_sc_30);
    lv_obj_align(lbl_admin_title, LV_ALIGN_TOP_MID, 0, 10);

    /* title_bar 横条 */
    lv_obj_t * img_title_bar = lv_image_create(g_admin_panel_pwd);
    lv_image_set_src(img_title_bar, &title_bar);
    lv_obj_align(img_title_bar, LV_ALIGN_TOP_MID, 0, 50);

    /* 提示文字 */
    lv_obj_t * lbl_serial_title = lv_label_create(g_admin_panel_pwd);
    g_admin_lbl_serial_title = lbl_serial_title;
    lv_label_set_text(lbl_serial_title, "请输入特殊出厂序列号");
    lv_obj_set_style_text_color(lbl_serial_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_serial_title, s_font_sc_40);
    lv_obj_align(lbl_serial_title, LV_ALIGN_TOP_MID, 0, 170);

    /* input_box 背景图 + 输入框 居中对齐 */
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
        lv_obj_add_event_cb(g_admin_ta_pwd, cb_admin_ta_ready, LV_EVENT_READY, NULL);
        lv_obj_add_event_cb(g_admin_ta_pwd, cb_admin_ta_key_enter, LV_EVENT_KEY | LV_EVENT_PREPROCESS, NULL);
        lv_obj_add_event_cb(g_admin_ta_pwd, cb_admin_ta_kb_focus, LV_EVENT_ALL, NULL);
    }

    /* 错误提示标签 */
    g_admin_lbl_msg_pwd = lv_label_create(g_admin_panel_pwd);
    lv_obj_set_width(g_admin_lbl_msg_pwd, LV_PCT(80));
    lv_obj_set_style_text_color(g_admin_lbl_msg_pwd, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_align(g_admin_lbl_msg_pwd, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_msg_pwd, s_font_sc_50);
    lv_obj_align(g_admin_lbl_msg_pwd, LV_ALIGN_CENTER, 0, 40);
    lv_obj_add_flag(g_admin_lbl_msg_pwd, LV_OBJ_FLAG_HIDDEN);

    /* 管理员设置页1（8 宫格） */
    g_admin_panel_menu1 = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_menu1, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_menu1, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_menu1, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_menu1, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_menu1, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_menu1, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_menu1, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_menu1_title = lv_label_create(g_admin_panel_menu1);
    lv_obj_set_style_text_color(g_admin_lbl_menu1_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_menu1_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_menu1_title, LV_ALIGN_TOP_MID, 0, 8);
    ui_lang_bind_label(g_admin_lbl_menu1_title, STR_ADMIN_MENU_TITLE);

    static const ui_str_id_t menu1_ids[8] = {
        STR_ADMIN_M1_MACHINE_ID, STR_ADMIN_M1_PROGRAM, STR_ADMIN_M1_BRIGHTNESS, STR_ADMIN_M1_VENDOR_MAINT,
        STR_ADMIN_M1_LANGUAGE, STR_ADMIN_M1_STANDBY, STR_ADMIN_M1_FACTORY_RESET, STR_ADMIN_M1_CONTACT
    };
    const lv_coord_t btn_w = 360;   //按钮宽度
    const lv_coord_t btn_h = 120;   //按钮高度
    const lv_coord_t gap_x = 24;    //按钮间距
    const lv_coord_t gap_y = 20+10;    //按钮间距
    const lv_coord_t grid_w = btn_w * 4 + gap_x * 3; //网格宽度
    const lv_coord_t grid_x0 = (lv_coord_t)((UI_FIXED_W - grid_w) / 2); //网格起始x坐标
    const lv_coord_t grid_y0 = 56+50;  //网格起始y坐标

    for(int i = 0; i < 8; i++) {
        int row = i / 4;
        int col = i % 4;
        g_admin_menu1_btns[i] = make_admin_menu_btn(g_admin_panel_menu1, ui_translation(menu1_ids[i]));
        lv_obj_set_size(g_admin_menu1_btns[i], btn_w, btn_h);
        lv_obj_set_pos(g_admin_menu1_btns[i],
            grid_x0 + col * (btn_w + gap_x),
            grid_y0 + row * (btn_h + gap_y));
        admin_menu_btn_bind_i18n(g_admin_menu1_btns[i], menu1_ids[i]);
        if(i == 0) {
            lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_machine_id, LV_EVENT_CLICKED, NULL);
        } else if(i == 1) {
            lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_program_settings, LV_EVENT_CLICKED, NULL);
        } else if(i == 2) {
            lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_brightness, LV_EVENT_CLICKED, NULL);
        } else if(i == 3) {
            lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_vendor_maint, LV_EVENT_CLICKED, NULL);
        } else if(i == 4) {
            lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_language_settings, LV_EVENT_CLICKED, NULL);
        } else if(i == 5) {
            lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_dormancy_standby, LV_EVENT_CLICKED, NULL);
        } else if(i == 6) {
            lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_factory_reset, LV_EVENT_CLICKED, NULL);
        } else if(i == 7) {
            lv_obj_add_event_cb(g_admin_menu1_btns[i], cb_admin_open_contact_us, LV_EVENT_CLICKED, NULL);
        }
    }

    /* 管理员设置页2（8 宫格） */
    g_admin_panel_menu2 = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_menu2, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_menu2, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_menu2, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_menu2, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_menu2, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_menu2, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_menu2, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_menu2_title = lv_label_create(g_admin_panel_menu2);
    lv_obj_set_style_text_color(g_admin_lbl_menu2_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_menu2_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_menu2_title, LV_ALIGN_TOP_MID, 0, 8);
    ui_lang_bind_label(g_admin_lbl_menu2_title, STR_ADMIN_MENU_TITLE);

    static const ui_str_id_t menu2_ids[8] = {
        STR_ADMIN_M2_AUTO_DISPENSE, STR_ADMIN_M2_OZONE, STR_ADMIN_M2_FRESH_AIR, STR_ADMIN_M2_UPGRADE,
        STR_ADMIN_M2_NETWORK, STR_ADMIN_M2_DATA, STR_ADMIN_M2_PAYMENT, STR_ADMIN_M2_PASSWORD
    };

    for(int i = 0; i < 8; i++) {
        int row = i / 4;
        int col = i % 4;
        g_admin_menu2_btns[i] = make_admin_menu_btn(g_admin_panel_menu2, ui_translation(menu2_ids[i]));
        lv_obj_set_size(g_admin_menu2_btns[i], btn_w, btn_h);
        lv_obj_set_pos(g_admin_menu2_btns[i],
            grid_x0 + col * (btn_w + gap_x),
            grid_y0 + row * (btn_h + gap_y));
        admin_menu_btn_bind_i18n(g_admin_menu2_btns[i], menu2_ids[i]);
        if(i == 0) {
            lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_auto_dispense, LV_EVENT_CLICKED, NULL);
        }
        else if(i == 1) {
            lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_ozone, LV_EVENT_CLICKED, NULL);
        }
        else if(i == 2) {
            lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_fresh_air_care, LV_EVENT_CLICKED, NULL);
        }
        else if(i == 3) {
            lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_system_upgrade, LV_EVENT_CLICKED, NULL);
        }
        else if(i == 4) {
            lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_network_settings, LV_EVENT_CLICKED, NULL);
        }
        else if(i == 5) {
            lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_data_settings, LV_EVENT_CLICKED, NULL);
        }
        else if(i == 6) {
            lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_payment_settings, LV_EVENT_CLICKED, NULL);
        }
        else if(i == 7) {
            lv_obj_add_event_cb(g_admin_menu2_btns[i], cb_admin_open_password_change, LV_EVENT_CLICKED, NULL);
        }
    }

    /* 待机时间子面板（1600×600 屏：顶栏下 y=60，内容区高 400） */
    g_admin_panel_dormancy = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_dormancy, 1600, 400);
    lv_obj_align(g_admin_panel_dormancy, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(g_admin_panel_dormancy, LV_OPA_TRANSP, LV_PART_MAIN);//20%透明
    lv_obj_set_style_border_width(g_admin_panel_dormancy, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_dormancy, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_dormancy, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_dormancy, LV_OBJ_FLAG_OVERFLOW_VISIBLE);//溢出可见
    lv_obj_add_flag(g_admin_panel_dormancy, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t * lbl_dormancy_title = lv_label_create(g_admin_panel_dormancy);
    ui_lang_bind_label(lbl_dormancy_title, STR_DORMANCY_TITLE);
    lv_obj_set_style_text_color(lbl_dormancy_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_dormancy_title, s_font_sc_30);
    lv_obj_align(lbl_dormancy_title, LV_ALIGN_TOP_MID, 0, 16);

    g_admin_lbl_dormancy_cur = lv_label_create(g_admin_panel_dormancy);//当前：几分钟
    lv_obj_set_style_text_color(g_admin_lbl_dormancy_cur, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_dormancy_cur, s_font_sc_30);
    lv_obj_align(g_admin_lbl_dormancy_cur, LV_ALIGN_TOP_MID, 0, 105);

    lv_obj_t * dormancy_box = lv_obj_create(g_admin_panel_dormancy);//滚动条区域
    lv_obj_set_size(dormancy_box, 420, 100);
    lv_obj_align(dormancy_box, LV_ALIGN_TOP_MID, 0, 165);
    lv_obj_set_style_radius(dormancy_box, 8, LV_PART_MAIN);
    lv_obj_set_style_bg_color(dormancy_box, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(dormancy_box, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(dormancy_box, 0, LV_PART_MAIN);
    lv_obj_add_flag(dormancy_box, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    g_admin_dormancy_roller = lv_roller_create(dormancy_box);
    lv_obj_set_size(g_admin_dormancy_roller, 380, 100);
    lv_roller_set_options(g_admin_dormancy_roller, ui_translation(STR_DORMANCY_ROLLER), LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(g_admin_dormancy_roller,1);//可见行数
    lv_obj_set_style_text_font(g_admin_dormancy_roller, s_font_sc_30, LV_PART_MAIN);
    lv_obj_set_style_text_font(g_admin_dormancy_roller, s_font_sc_30, LV_PART_SELECTED);
    lv_obj_set_style_bg_opa(g_admin_dormancy_roller, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_admin_dormancy_roller, LV_OPA_TRANSP, LV_PART_SELECTED);
    lv_obj_set_style_text_color(g_admin_dormancy_roller, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_color(g_admin_dormancy_roller, lv_color_hex(0x000000), LV_PART_SELECTED);
    lv_obj_set_style_border_width(g_admin_dormancy_roller, 0, LV_PART_MAIN);
    lv_obj_center(g_admin_dormancy_roller);
    lv_obj_add_event_cb(g_admin_dormancy_roller, cb_admin_dormancy_roller_changed, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(g_admin_dormancy_roller, cb_admin_dormancy_roller_encoder, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(g_admin_dormancy_roller, cb_admin_dormancy_roller_encoder, LV_EVENT_DEFOCUSED, NULL);

    g_admin_btn_dormancy_confirm = make_orange_fill_btn(g_admin_panel_dormancy, ui_translation(STR_BTN_CONFIRM), 160, 44);
    lv_obj_align(g_admin_btn_dormancy_confirm, LV_ALIGN_BOTTOM_MID, 0, -50);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_dormancy_confirm, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_dormancy_confirm, STR_BTN_CONFIRM);
    lv_obj_add_event_cb(g_admin_btn_dormancy_confirm, cb_admin_dormancy_confirm, LV_EVENT_CLICKED, NULL);

    /* 屏幕亮度子面板（1600×400，布局同恢复默认；标题左、开关与标题同行右侧） */
    g_admin_panel_brightness = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_brightness, 1600, 400);
    lv_obj_align(g_admin_panel_brightness, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_bg_opa(g_admin_panel_brightness, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_brightness, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_brightness, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_brightness, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_brightness, LV_OBJ_FLAG_HIDDEN);
    /* 预留：标题左侧装饰图 g_admin_img_brightness_deco、标题下分隔图（后期用图片添加） */

    g_admin_lbl_brightness_title = lv_label_create(g_admin_panel_brightness);
    ui_lang_bind_label(g_admin_lbl_brightness_title, STR_ADMIN_M1_BRIGHTNESS);
    lv_obj_set_style_text_color(g_admin_lbl_brightness_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_brightness_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_brightness_title, LV_ALIGN_TOP_LEFT, 350, 30);

    g_admin_sw_run_always_on = lv_switch_create(g_admin_panel_brightness);
    admin_brightness_apply_switch_layout();
    lv_obj_add_event_cb(g_admin_sw_run_always_on, cb_admin_brightness_switch_changed, LV_EVENT_VALUE_CHANGED, NULL);

    g_admin_lbl_brightness_line1 = lv_label_create(g_admin_panel_brightness);
    ui_lang_bind_label(g_admin_lbl_brightness_line1, STR_BRIGHTNESS_LINE1);
    lv_obj_set_style_text_color(g_admin_lbl_brightness_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_brightness_line1, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_brightness_line1, 400, 150);

    g_admin_lbl_brightness_line2 = lv_label_create(g_admin_panel_brightness);
    ui_lang_bind_label(g_admin_lbl_brightness_line2, STR_BRIGHTNESS_LINE2);
    lv_obj_set_style_text_color(g_admin_lbl_brightness_line2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_brightness_line2, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_brightness_line2, 400, 205);

    admin_menu_style_init();
    brightness_slider_style_init();
    lv_obj_t * brightness_slider_frame = lv_obj_create(g_admin_panel_brightness);
    lv_obj_set_size(brightness_slider_frame, 815, 60);
    lv_obj_align(brightness_slider_frame, LV_ALIGN_BOTTOM_RIGHT, -400, -40);
    lv_obj_add_style(brightness_slider_frame, &s_brightness_slider_frame_style, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(brightness_slider_frame, 0, LV_PART_MAIN);
    lv_obj_remove_flag(brightness_slider_frame, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * brightness_slider_inner = lv_obj_create(brightness_slider_frame);
    lv_obj_remove_style_all(brightness_slider_inner);
    lv_obj_add_style(brightness_slider_inner, &s_brightness_slider_inner_style, LV_PART_MAIN);
    lv_obj_set_size(brightness_slider_inner, LV_PCT(100), LV_PCT(100));
    lv_obj_clear_flag(brightness_slider_inner, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    /* 下层：渐变动条（lv_bar 仅负责显示） */
    g_admin_brightness_bar = lv_bar_create(brightness_slider_inner);
    lv_obj_set_size(g_admin_brightness_bar, LV_PCT(100), LV_PCT(100));
    lv_bar_set_range(g_admin_brightness_bar, 0, 100);
    lv_obj_set_style_bg_opa(g_admin_brightness_bar, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_brightness_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(g_admin_brightness_bar, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_brightness_bar, 2, LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_admin_brightness_bar, lv_color_hex(BRIGHTNESS_SLIDER_BLACK), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_color(g_admin_brightness_bar, lv_color_hex(BRIGHTNESS_SLIDER_BLUE), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_dir(g_admin_brightness_bar, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(g_admin_brightness_bar, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(g_admin_brightness_bar, 2, LV_PART_INDICATOR);
    lv_obj_remove_flag(g_admin_brightness_bar, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    /* 中层：透明滑条，承接触摸/编码器 */
    g_admin_slider_brightness = lv_slider_create(brightness_slider_inner);
    lv_obj_set_size(g_admin_slider_brightness, LV_PCT(100), LV_PCT(100));
    lv_slider_set_range(g_admin_slider_brightness, 0, 100);
    lv_obj_set_style_bg_opa(g_admin_slider_brightness, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_slider_brightness, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(g_admin_slider_brightness, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_slider_brightness, 2, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_admin_slider_brightness, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(g_admin_slider_brightness, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_border_opa(g_admin_slider_brightness, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_shadow_opa(g_admin_slider_brightness, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_width(g_admin_slider_brightness, 0, LV_PART_KNOB);
    lv_obj_set_style_height(g_admin_slider_brightness, 0, LV_PART_KNOB);
    lv_obj_set_style_pad_all(g_admin_slider_brightness, 0, LV_PART_KNOB);

    /* 上层：白色焦点框（不拦截点击，叠在动条之上） */
    g_admin_brightness_slider_focus = lv_obj_create(brightness_slider_inner);
    lv_obj_remove_style_all(g_admin_brightness_slider_focus);
    lv_obj_set_size(g_admin_brightness_slider_focus, LV_PCT(100), LV_PCT(100));
    style_brightness_slider_encoder_focus_inner(g_admin_brightness_slider_focus);
    lv_obj_remove_flag(g_admin_brightness_slider_focus, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_admin_brightness_slider_focus, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_move_foreground(g_admin_brightness_slider_focus);

    lv_obj_add_event_cb(g_admin_slider_brightness, cb_admin_brightness_slider_focus_frame, LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(g_admin_slider_brightness, cb_admin_brightness_slider_focus_frame, LV_EVENT_DEFOCUSED, NULL);
    lv_obj_add_event_cb(g_admin_slider_brightness, cb_admin_brightness_slider_changed, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(g_admin_slider_brightness, cb_admin_brightness_slider_key,
                        LV_EVENT_KEY | LV_EVENT_PREPROCESS, NULL);
    lv_obj_add_event_cb(g_admin_slider_brightness, cb_admin_brightness_slider_encoder, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(g_admin_slider_brightness, cb_admin_brightness_slider_encoder, LV_EVENT_DEFOCUSED, NULL);

    admin_brightness_sync_ui();

    /* 声音控制子面板（1600×400，两行开关 + 两条渐变滑条，滑条左上方符号图标） */
    g_admin_panel_sound = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_sound, 1600, 400);
    lv_obj_align(g_admin_panel_sound, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_bg_opa(g_admin_panel_sound, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_sound, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_sound, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_sound, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_sound, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_sound_title = lv_label_create(g_admin_panel_sound);
    ui_lang_bind_label(g_admin_lbl_sound_title, STR_ADMIN_M1_SOUND);
    lv_obj_set_style_text_color(g_admin_lbl_sound_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_sound_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_sound_title, LV_ALIGN_TOP_LEFT, 350, 30);

    g_admin_lbl_sound_line1 = lv_label_create(g_admin_panel_sound);
    ui_lang_bind_label(g_admin_lbl_sound_line1, STR_SOUND_TOUCH);
    lv_obj_set_style_text_color(g_admin_lbl_sound_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_sound_line1, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_sound_line1, 400, 120);

    g_admin_sw_touch_sound = lv_switch_create(g_admin_panel_sound);
    lv_obj_add_event_cb(g_admin_sw_touch_sound, cb_admin_sound_touch_switch_changed, LV_EVENT_VALUE_CHANGED, NULL);

    g_admin_lbl_sound_line2 = lv_label_create(g_admin_panel_sound);
    ui_lang_bind_label(g_admin_lbl_sound_line2, STR_SOUND_VOICE);
    lv_obj_set_style_text_color(g_admin_lbl_sound_line2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_sound_line2, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_sound_line2, 400, 175);

    g_admin_sw_voice_broadcast = lv_switch_create(g_admin_panel_sound);
    lv_obj_add_event_cb(g_admin_sw_voice_broadcast, cb_admin_sound_voice_broadcast_switch_changed,
                        LV_EVENT_VALUE_CHANGED, NULL);

    brightness_slider_style_init();

    /* 音量滑条（上方，先创建） */
    lv_obj_t * sound_volume_slider_frame = lv_obj_create(g_admin_panel_sound);
    lv_obj_set_size(sound_volume_slider_frame, 815, 60);
    lv_obj_align(sound_volume_slider_frame, LV_ALIGN_BOTTOM_RIGHT, -400, -128+50);
    lv_obj_add_style(sound_volume_slider_frame, &s_brightness_slider_frame_style, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(sound_volume_slider_frame, 0, LV_PART_MAIN);
    lv_obj_remove_flag(sound_volume_slider_frame, LV_OBJ_FLAG_SCROLLABLE);
    admin_gradient_slider_fill_inner(sound_volume_slider_frame, &g_admin_sound_volume_bar,
                                     &g_admin_slider_sound_volume, &g_admin_sound_volume_slider_focus);

    lv_obj_add_event_cb(g_admin_slider_sound_volume, cb_admin_sound_volume_slider_focus_frame, LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(g_admin_slider_sound_volume, cb_admin_sound_volume_slider_focus_frame, LV_EVENT_DEFOCUSED,
                        NULL);
    lv_obj_add_event_cb(g_admin_slider_sound_volume, cb_admin_sound_volume_slider_changed, LV_EVENT_VALUE_CHANGED,
                        NULL);
    lv_obj_add_event_cb(g_admin_slider_sound_volume, cb_admin_sound_volume_slider_key,
                        LV_EVENT_KEY | LV_EVENT_PREPROCESS, NULL);
    lv_obj_add_event_cb(g_admin_slider_sound_volume, cb_admin_sound_volume_slider_encoder, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(g_admin_slider_sound_volume, cb_admin_sound_volume_slider_encoder, LV_EVENT_DEFOCUSED, NULL);

    /* 触控声音滑条（靠下，同亮度页 BOTTOM_MID, 0, -40） */
    lv_obj_t * touch_sound_slider_frame = lv_obj_create(g_admin_panel_sound);
    lv_obj_set_size(touch_sound_slider_frame, 815, 60);
    lv_obj_align(touch_sound_slider_frame, LV_ALIGN_BOTTOM_RIGHT, -400, -40+40);
    lv_obj_add_style(touch_sound_slider_frame, &s_brightness_slider_frame_style, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(touch_sound_slider_frame, 0, LV_PART_MAIN);
    lv_obj_remove_flag(touch_sound_slider_frame, LV_OBJ_FLAG_SCROLLABLE);
    admin_gradient_slider_fill_inner(touch_sound_slider_frame, &g_admin_touch_sound_volume_bar,
                                     &g_admin_slider_touch_sound_volume,
                                     &g_admin_touch_sound_volume_slider_focus);

    g_admin_lbl_sound_vol_icon = lv_label_create(g_admin_panel_sound);
    lv_label_set_text(g_admin_lbl_sound_vol_icon, LV_SYMBOL_VOLUME_MAX);
    lv_obj_set_style_text_color(g_admin_lbl_sound_vol_icon, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(g_admin_lbl_sound_vol_icon, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_align(g_admin_lbl_sound_vol_icon, LV_ALIGN_TOP_LEFT, 400, 176+100);

    g_admin_lbl_touch_sound_icon = lv_label_create(g_admin_panel_sound);
    lv_label_set_text(g_admin_lbl_touch_sound_icon, LV_SYMBOL_GPS);
    lv_obj_set_style_text_color(g_admin_lbl_touch_sound_icon, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(g_admin_lbl_touch_sound_icon, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_align(g_admin_lbl_touch_sound_icon, LV_ALIGN_TOP_LEFT, 400, 254+100);

    lv_obj_add_event_cb(g_admin_slider_touch_sound_volume, cb_admin_sound_touch_sound_volume_slider_focus_frame,
                        LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(g_admin_slider_touch_sound_volume, cb_admin_sound_touch_sound_volume_slider_focus_frame,
                        LV_EVENT_DEFOCUSED, NULL);
    lv_obj_add_event_cb(g_admin_slider_touch_sound_volume, cb_admin_sound_touch_sound_volume_slider_changed,
                        LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(g_admin_slider_touch_sound_volume, cb_admin_sound_touch_sound_volume_slider_key,
                        LV_EVENT_KEY | LV_EVENT_PREPROCESS, NULL);
    lv_obj_add_event_cb(g_admin_slider_touch_sound_volume, cb_admin_sound_touch_sound_volume_slider_encoder,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(g_admin_slider_touch_sound_volume, cb_admin_sound_touch_sound_volume_slider_encoder,
                        LV_EVENT_DEFOCUSED, NULL);

    admin_sound_sync_ui();

    /* 语言设置子面板（1600×400，左文右钮竖排，布局同恢复默认） */
    g_admin_panel_lang = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_lang, 1600, 400);
    lv_obj_align(g_admin_panel_lang, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_bg_opa(g_admin_panel_lang, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_lang, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_lang, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_lang, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_lang, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_lang_title = lv_label_create(g_admin_panel_lang);
    lv_obj_set_style_text_color(g_admin_lbl_lang_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_lang_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_lang_title, LV_ALIGN_TOP_LEFT, 350, 30);
    ui_lang_bind_label(g_admin_lbl_lang_title, STR_ADMIN_LANG_TITLE);

    g_admin_lbl_lang_line1 = lv_label_create(g_admin_panel_lang);
    lv_obj_set_style_text_color(g_admin_lbl_lang_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_lang_line1, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_lang_line1, 400, 150+20);
    ui_lang_bind_label(g_admin_lbl_lang_line1, STR_ADMIN_LANG_HINT);

    g_admin_btn_lang_zh = make_orange_fill_btn(g_admin_panel_lang, ui_translation(STR_ADMIN_LANG_BTN_ZH), 160, 44);
    lv_obj_set_pos(g_admin_btn_lang_zh, 1100-60, 140);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_lang_zh, 0), s_font_sc_30);
    lv_obj_add_event_cb(g_admin_btn_lang_zh, cb_admin_lang_zh, LV_EVENT_CLICKED, NULL);
    ui_lang_bind_label(lv_obj_get_child(g_admin_btn_lang_zh, 0), STR_ADMIN_LANG_BTN_ZH);

    g_admin_btn_lang_en = make_orange_fill_btn(g_admin_panel_lang, ui_translation(STR_ADMIN_LANG_BTN_EN), 160, 44);
    lv_obj_set_pos(g_admin_btn_lang_en, 1100-60, 200);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_lang_en, 0), s_font_sc_30);
    lv_obj_add_event_cb(g_admin_btn_lang_en, cb_admin_lang_en, LV_EVENT_CLICKED, NULL);
    ui_lang_bind_label(lv_obj_get_child(g_admin_btn_lang_en, 0), STR_ADMIN_LANG_BTN_EN);

    admin_lang_sync_btn_ui();

    /* 恢复默认子面板（1600×400，左文右钮竖排） */
    g_admin_panel_factory = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_factory, 1600, 400);
    lv_obj_align(g_admin_panel_factory, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_bg_opa(g_admin_panel_factory, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_factory, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_factory, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_factory, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_factory, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_factory_title = lv_label_create(g_admin_panel_factory);
    ui_lang_bind_label(g_admin_lbl_factory_title, STR_ADMIN_M1_FACTORY_RESET);
    lv_obj_set_style_text_color(g_admin_lbl_factory_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_factory_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_factory_title, LV_ALIGN_TOP_LEFT, 350, 30);

    g_admin_lbl_factory_line1 = lv_label_create(g_admin_panel_factory);
    ui_lang_bind_label(g_admin_lbl_factory_line1, STR_FACTORY_CONFIRM_Q);
    lv_obj_set_style_text_color(g_admin_lbl_factory_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_factory_line1, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_factory_line1, 400, 150);

    g_admin_lbl_factory_line2 = lv_label_create(g_admin_panel_factory);
    ui_lang_bind_label(g_admin_lbl_factory_line2, STR_FACTORY_CONFIRM_HINT);
    lv_obj_set_style_text_color(g_admin_lbl_factory_line2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_factory_line2, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_factory_line2, 400-15, 205);

    g_admin_lbl_factory_status = lv_label_create(g_admin_panel_factory);
    lv_label_set_text(g_admin_lbl_factory_status, ui_translation(STR_FACTORY_RESTORING));
    lv_obj_set_style_text_color(g_admin_lbl_factory_status, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_factory_status, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_factory_status, 1200);
    lv_label_set_long_mode(g_admin_lbl_factory_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(g_admin_lbl_factory_status, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(g_admin_lbl_factory_status, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(g_admin_lbl_factory_status, LV_OBJ_FLAG_HIDDEN);

    g_admin_btn_factory_ok = make_orange_fill_btn(g_admin_panel_factory, ui_translation(STR_BTN_OK), 160, 44);
    lv_obj_set_pos(g_admin_btn_factory_ok, 1100-60, 140);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_factory_ok, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_factory_ok, STR_BTN_OK);
    lv_obj_add_event_cb(g_admin_btn_factory_ok, cb_admin_factory_ok, LV_EVENT_CLICKED, NULL);

    g_admin_btn_factory_cancel = make_orange_outline_btn(g_admin_panel_factory, ui_translation(STR_BTN_CANCEL), 160, 44);
    lv_obj_set_pos(g_admin_btn_factory_cancel, 1100-60, 200);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_factory_cancel, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_factory_cancel, STR_BTN_CANCEL);
    lv_obj_add_event_cb(g_admin_btn_factory_cancel, cb_admin_factory_cancel, LV_EVENT_CLICKED, NULL);

    /* 联系我们子面板（1600×400，左文右二维码，布局同恢复默认） */
    g_admin_panel_contact = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_contact, 1600, 400);
    lv_obj_align(g_admin_panel_contact, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_bg_opa(g_admin_panel_contact, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_contact, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_contact, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_contact, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_contact, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_contact_title = lv_label_create(g_admin_panel_contact);
    ui_lang_bind_label(g_admin_lbl_contact_title, STR_ADMIN_M1_CONTACT);
    lv_obj_set_style_text_color(g_admin_lbl_contact_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_contact_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_contact_title, LV_ALIGN_TOP_LEFT, 350, 30);

    g_admin_lbl_contact_line1 = lv_label_create(g_admin_panel_contact);
    ui_lang_bind_label(g_admin_lbl_contact_line1, STR_CONTACT_HOTLINE);
    lv_obj_set_style_text_color(g_admin_lbl_contact_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_contact_line1, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_contact_line1, 400, 150);

    g_admin_lbl_contact_line2 = lv_label_create(g_admin_panel_contact);
    ui_lang_bind_label(g_admin_lbl_contact_line2, STR_CONTACT_SLOGAN);
    lv_obj_set_style_text_color(g_admin_lbl_contact_line2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_contact_line2, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_contact_line2, 400, 205);

    g_admin_img_contact_qr = lv_image_create(g_admin_panel_contact);
    lv_image_set_src(g_admin_img_contact_qr, &QR_code_xiaoya);
    lv_obj_set_size(g_admin_img_contact_qr, 160, 160);//小鸭二维码大小
    lv_obj_set_pos(g_admin_img_contact_qr, 1100-60, 120-10);
    lv_image_set_inner_align(g_admin_img_contact_qr, LV_IMAGE_ALIGN_STRETCH);

    /* 自投功能子面板（1600×400，左文右钮竖排，布局同恢复默认/语言设置） */
    g_admin_panel_auto_dispense = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_auto_dispense, 1600, 400);
    lv_obj_align(g_admin_panel_auto_dispense, LV_ALIGN_TOP_MID, 0, 100);
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

    g_admin_lbl_auto_dispense_line1 = lv_label_create(g_admin_panel_auto_dispense);
    ui_lang_bind_label(g_admin_lbl_auto_dispense_line1, STR_AUTO_DISP_LINE1);
    lv_obj_set_style_text_color(g_admin_lbl_auto_dispense_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_auto_dispense_line1, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_auto_dispense_line1, 400, 150);

    g_admin_lbl_auto_dispense_line2 = lv_label_create(g_admin_panel_auto_dispense);
    ui_lang_bind_label(g_admin_lbl_auto_dispense_line2, STR_AUTO_DISP_LINE2);
    lv_obj_set_style_text_color(g_admin_lbl_auto_dispense_line2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_auto_dispense_line2, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_auto_dispense_line2, 400, 205);

    g_admin_btn_auto_dispense_on = make_orange_fill_btn(g_admin_panel_auto_dispense, ui_translation(STR_BTN_ON), 160, 44);
    lv_obj_set_pos(g_admin_btn_auto_dispense_on, 1100 - 60, 140);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_auto_dispense_on, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_auto_dispense_on, STR_BTN_ON);
    lv_obj_add_event_cb(g_admin_btn_auto_dispense_on, cb_admin_auto_dispense_on, LV_EVENT_CLICKED, NULL);

    g_admin_btn_auto_dispense_off = make_orange_fill_btn(g_admin_panel_auto_dispense, ui_translation(STR_BTN_OFF), 160, 44);
    lv_obj_set_pos(g_admin_btn_auto_dispense_off, 1100 - 60, 200);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_auto_dispense_off, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_auto_dispense_off, STR_BTN_OFF);
    lv_obj_add_event_cb(g_admin_btn_auto_dispense_off, cb_admin_auto_dispense_off, LV_EVENT_CLICKED, NULL);

    admin_auto_dispense_sync_btn_ui();

    /* 臭氧功能子面板（1600×400，左文右钮竖排，布局同自投功能） */
    g_admin_panel_ozone = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_ozone, 1600, 400);
    lv_obj_align(g_admin_panel_ozone, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_bg_opa(g_admin_panel_ozone, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_ozone, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_ozone, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_ozone, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_ozone, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_ozone_title = lv_label_create(g_admin_panel_ozone);
    ui_lang_bind_label(g_admin_lbl_ozone_title, STR_ADMIN_M2_OZONE);
    lv_obj_set_style_text_color(g_admin_lbl_ozone_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_ozone_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_ozone_title, LV_ALIGN_TOP_LEFT, 350, 30);

    g_admin_lbl_ozone_line1 = lv_label_create(g_admin_panel_ozone);
    ui_lang_bind_label(g_admin_lbl_ozone_line1, STR_OZONE_LINE1);
    lv_obj_set_style_text_color(g_admin_lbl_ozone_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_ozone_line1, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_ozone_line1, 400, 150);

    g_admin_lbl_ozone_line2 = lv_label_create(g_admin_panel_ozone);
    ui_lang_bind_label(g_admin_lbl_ozone_line2, STR_OZONE_LINE2);
    lv_obj_set_style_text_color(g_admin_lbl_ozone_line2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_ozone_line2, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_ozone_line2, 400, 205);

    g_admin_btn_ozone_on = make_orange_fill_btn(g_admin_panel_ozone, ui_translation(STR_BTN_ON), 160, 44);
    lv_obj_set_pos(g_admin_btn_ozone_on, 1100 - 60, 140);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_ozone_on, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_ozone_on, STR_BTN_ON);
    lv_obj_add_event_cb(g_admin_btn_ozone_on, cb_admin_ozone_on, LV_EVENT_CLICKED, NULL);

    g_admin_btn_ozone_off = make_orange_fill_btn(g_admin_panel_ozone, ui_translation(STR_BTN_OFF), 160, 44);
    lv_obj_set_pos(g_admin_btn_ozone_off, 1100 - 60, 200);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_ozone_off, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_ozone_off, STR_BTN_OFF);
    lv_obj_add_event_cb(g_admin_btn_ozone_off, cb_admin_ozone_off, LV_EVENT_CLICKED, NULL);

    admin_ozone_sync_btn_ui();

    /* 新风护理子面板（1600×400，左文右钮竖排，布局同自投功能） */
    g_admin_panel_fresh_air_care = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_fresh_air_care, 1600, 400);
    lv_obj_align(g_admin_panel_fresh_air_care, LV_ALIGN_TOP_MID, 0, 100);
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

    g_admin_lbl_fresh_air_care_line1 = lv_label_create(g_admin_panel_fresh_air_care);
    ui_lang_bind_label(g_admin_lbl_fresh_air_care_line1, STR_FRESH_AIR_LINE1);
    lv_obj_set_style_text_color(g_admin_lbl_fresh_air_care_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_fresh_air_care_line1, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_fresh_air_care_line1, 400, 150);

    g_admin_lbl_fresh_air_care_line2 = lv_label_create(g_admin_panel_fresh_air_care);
    ui_lang_bind_label(g_admin_lbl_fresh_air_care_line2, STR_FRESH_AIR_LINE2);
    lv_obj_set_style_text_color(g_admin_lbl_fresh_air_care_line2, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_fresh_air_care_line2, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_fresh_air_care_line2, 400, 205);

    g_admin_btn_fresh_air_care_on = make_orange_fill_btn(g_admin_panel_fresh_air_care, ui_translation(STR_BTN_ON), 160, 44);
    lv_obj_set_pos(g_admin_btn_fresh_air_care_on, 1100 - 60, 140);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_fresh_air_care_on, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_fresh_air_care_on, STR_BTN_ON);
    lv_obj_add_event_cb(g_admin_btn_fresh_air_care_on, cb_admin_fresh_air_care_on, LV_EVENT_CLICKED, NULL);

    g_admin_btn_fresh_air_care_off = make_orange_fill_btn(g_admin_panel_fresh_air_care, ui_translation(STR_BTN_OFF), 160, 44);
    lv_obj_set_pos(g_admin_btn_fresh_air_care_off, 1100 - 60, 200);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_fresh_air_care_off, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_fresh_air_care_off, STR_BTN_OFF);
    lv_obj_add_event_cb(g_admin_btn_fresh_air_care_off, cb_admin_fresh_air_care_off, LV_EVENT_CLICKED, NULL);

    admin_fresh_air_care_sync_btn_ui();

    /* 系统升级子面板（1600×400，左文右钮，布局同恢复默认） */
    g_admin_panel_system_upgrade = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_system_upgrade, 1600, 400);
    lv_obj_align(g_admin_panel_system_upgrade, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_bg_opa(g_admin_panel_system_upgrade, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_system_upgrade, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_system_upgrade, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_system_upgrade, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_system_upgrade, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_system_upgrade_title = lv_label_create(g_admin_panel_system_upgrade);
    ui_lang_bind_label(g_admin_lbl_system_upgrade_title, STR_ADMIN_M2_UPGRADE);
    lv_obj_set_style_text_color(g_admin_lbl_system_upgrade_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_system_upgrade_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_system_upgrade_title, LV_ALIGN_TOP_LEFT, 350, 30);

    g_admin_lbl_system_upgrade_line1 = lv_label_create(g_admin_panel_system_upgrade);
    ui_lang_bind_label(g_admin_lbl_system_upgrade_line1, STR_UPGRADE_CONFIRM_Q);
    lv_obj_set_style_text_color(g_admin_lbl_system_upgrade_line1, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_system_upgrade_line1, s_font_sc_30);
    lv_obj_set_pos(g_admin_lbl_system_upgrade_line1, 400, (150+205)/2);

    g_admin_lbl_system_upgrade_status = lv_label_create(g_admin_panel_system_upgrade);
    lv_label_set_text(g_admin_lbl_system_upgrade_status, ui_translation(STR_UPGRADE_IN_PROGRESS));
    lv_obj_set_style_text_color(g_admin_lbl_system_upgrade_status, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_system_upgrade_status, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_system_upgrade_status, 1200);
    lv_label_set_long_mode(g_admin_lbl_system_upgrade_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(g_admin_lbl_system_upgrade_status, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(g_admin_lbl_system_upgrade_status, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(g_admin_lbl_system_upgrade_status, LV_OBJ_FLAG_HIDDEN);

    g_admin_btn_system_upgrade_ok = make_orange_outline_btn(g_admin_panel_system_upgrade, ui_translation(STR_BTN_CONFIRM), 160, 44);
    lv_obj_set_pos(g_admin_btn_system_upgrade_ok, 1100 - 60, (140+200)/2);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_system_upgrade_ok, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_system_upgrade_ok, STR_BTN_CONFIRM);
    lv_obj_add_event_cb(g_admin_btn_system_upgrade_ok, cb_admin_system_upgrade_ok, LV_EVENT_CLICKED, NULL);

    /* 支付设置子面板（1600×400，三栏：支付方式 / 支付超时 / 订单查询） */
    g_admin_panel_payment = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_payment, 1600, 400);
    lv_obj_align(g_admin_panel_payment, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_bg_opa(g_admin_panel_payment, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_payment, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_payment, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_payment, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_payment, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_payment_title = lv_label_create(g_admin_panel_payment);
    ui_lang_bind_label(g_admin_lbl_payment_title, STR_ADMIN_M2_PAYMENT);
    lv_obj_set_style_text_color(g_admin_lbl_payment_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_payment_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_payment_title, LV_ALIGN_TOP_MID, 0, 0);

    admin_payment_add_divider(g_admin_panel_payment, 520);
    admin_payment_add_divider(g_admin_panel_payment, 1080);

    g_admin_lbl_payment_method_hdr = lv_label_create(g_admin_panel_payment);
    ui_lang_bind_label(g_admin_lbl_payment_method_hdr, STR_PAYMENT_METHOD);
    lv_obj_set_style_text_color(g_admin_lbl_payment_method_hdr, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_payment_method_hdr, s_font_sc_30);
    lv_obj_align(g_admin_lbl_payment_method_hdr, LV_ALIGN_TOP_MID, -500, 70);

    g_admin_lbl_payment_timeout_hdr = lv_label_create(g_admin_panel_payment);
    ui_lang_bind_label(g_admin_lbl_payment_timeout_hdr, STR_PAYMENT_TIMEOUT);
    lv_obj_set_style_text_color(g_admin_lbl_payment_timeout_hdr, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_payment_timeout_hdr, s_font_sc_30);
    lv_obj_align(g_admin_lbl_payment_timeout_hdr, LV_ALIGN_TOP_MID, 0, 70);

    g_admin_lbl_payment_order_hdr = lv_label_create(g_admin_panel_payment);
    ui_lang_bind_label(g_admin_lbl_payment_order_hdr, STR_PAYMENT_ORDER);
    lv_obj_set_style_text_color(g_admin_lbl_payment_order_hdr, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_payment_order_hdr, s_font_sc_30);
    lv_obj_align(g_admin_lbl_payment_order_hdr, LV_ALIGN_TOP_MID, 520, 70);

    g_admin_cb_payment_alipay = lv_checkbox_create(g_admin_panel_payment);
    lv_checkbox_set_text(g_admin_cb_payment_alipay, ui_translation(STR_PAYMENT_ALIPAY));
    admin_payment_style_checkbox(g_admin_cb_payment_alipay);
    lv_obj_set_width(g_admin_cb_payment_alipay, 400);
    lv_obj_set_pos(g_admin_cb_payment_alipay, 100, 130);
    lv_obj_add_event_cb(g_admin_cb_payment_alipay, cb_admin_payment_alipay_changed, LV_EVENT_VALUE_CHANGED, NULL);

    g_admin_cb_payment_wechat = lv_checkbox_create(g_admin_panel_payment);
    lv_checkbox_set_text(g_admin_cb_payment_wechat, ui_translation(STR_PAYMENT_WECHAT));
    admin_payment_style_checkbox(g_admin_cb_payment_wechat);
    lv_obj_set_width(g_admin_cb_payment_wechat, 400);
    lv_obj_set_pos(g_admin_cb_payment_wechat, 100, 200);
    lv_obj_add_event_cb(g_admin_cb_payment_wechat, cb_admin_payment_wechat_changed, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t * payment_timeout_box = lv_obj_create(g_admin_panel_payment);
    lv_obj_set_size(payment_timeout_box, 420, 100);
    lv_obj_align(payment_timeout_box, LV_ALIGN_TOP_MID, 0, 130);
    lv_obj_set_style_radius(payment_timeout_box, 8, LV_PART_MAIN);
    lv_obj_set_style_bg_color(payment_timeout_box, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(payment_timeout_box, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(payment_timeout_box, 0, LV_PART_MAIN);
    lv_obj_add_flag(payment_timeout_box, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_obj_remove_flag(payment_timeout_box, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    g_admin_payment_timeout_roller = lv_roller_create(payment_timeout_box);
    lv_obj_set_size(g_admin_payment_timeout_roller, 380, 100);
    lv_roller_set_options(g_admin_payment_timeout_roller, ui_translation(STR_PAYMENT_TIMEOUT_ROLLER), LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(g_admin_payment_timeout_roller, 1);
    lv_obj_set_style_text_font(g_admin_payment_timeout_roller, s_font_sc_30, LV_PART_MAIN);
    lv_obj_set_style_text_font(g_admin_payment_timeout_roller, s_font_sc_30, LV_PART_SELECTED);
    lv_obj_set_style_bg_opa(g_admin_payment_timeout_roller, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_admin_payment_timeout_roller, LV_OPA_TRANSP, LV_PART_SELECTED);
    lv_obj_set_style_text_color(g_admin_payment_timeout_roller, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_color(g_admin_payment_timeout_roller, lv_color_hex(0x000000), LV_PART_SELECTED);
    lv_obj_set_style_border_width(g_admin_payment_timeout_roller, 0, LV_PART_MAIN);
    lv_obj_center(g_admin_payment_timeout_roller);
    lv_obj_add_event_cb(g_admin_payment_timeout_roller, cb_admin_payment_timeout_roller_encoder, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(g_admin_payment_timeout_roller, cb_admin_payment_timeout_roller_encoder, LV_EVENT_DEFOCUSED, NULL);

    g_admin_lbl_payment_order_hint = lv_label_create(g_admin_panel_payment);
    ui_lang_bind_label(g_admin_lbl_payment_order_hint, STR_PAYMENT_ORDER_HINT);
    lv_obj_set_style_text_color(g_admin_lbl_payment_order_hint, lv_color_hex(COL_DIM), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_payment_order_hint, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_payment_order_hint, 360);
    lv_label_set_long_mode(g_admin_lbl_payment_order_hint, LV_LABEL_LONG_WRAP);
    lv_obj_align(g_admin_lbl_payment_order_hint, LV_ALIGN_TOP_MID, 520, 130);

    g_admin_btn_payment_query = make_orange_fill_btn(g_admin_panel_payment, ui_translation(STR_BTN_QUERY), 160, 44);
    lv_obj_align(g_admin_btn_payment_query, LV_ALIGN_TOP_MID, 520, 200);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_payment_query, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_payment_query, STR_BTN_QUERY);

    admin_payment_sync_checkbox_ui();
    admin_payment_sync_timeout_roller_ui();

    /* 数据设置子面板（1600×400，两栏：上传项 / 上传策略） */
    g_admin_panel_data = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_data, 1600, 500);
    lv_obj_align(g_admin_panel_data, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_bg_opa(g_admin_panel_data, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_data, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_data, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_data, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_data, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_data_title = lv_label_create(g_admin_panel_data);
    ui_lang_bind_label(g_admin_lbl_data_title, STR_ADMIN_M2_DATA);
    lv_obj_set_style_text_color(g_admin_lbl_data_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_data_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_data_title, LV_ALIGN_TOP_MID, 0, 0);

    admin_payment_add_divider(g_admin_panel_data, 800);

    g_admin_lbl_data_upload_hdr = lv_label_create(g_admin_panel_data);
    ui_lang_bind_label(g_admin_lbl_data_upload_hdr, STR_DATA_UPLOAD_HDR);
    lv_obj_set_style_text_color(g_admin_lbl_data_upload_hdr, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_data_upload_hdr, s_font_sc_30);
    lv_obj_align(g_admin_lbl_data_upload_hdr, LV_ALIGN_TOP_MID, -400, 70);

    g_admin_lbl_data_strategy_hdr = lv_label_create(g_admin_panel_data);
    ui_lang_bind_label(g_admin_lbl_data_strategy_hdr, STR_DATA_STRATEGY_HDR);
    lv_obj_set_style_text_color(g_admin_lbl_data_strategy_hdr, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_data_strategy_hdr, s_font_sc_30);
    lv_obj_align(g_admin_lbl_data_strategy_hdr, LV_ALIGN_TOP_MID, 400, 70);

    for(int i = 0; i < 7; i++) {
        g_admin_cb_data_upload[i] = lv_checkbox_create(g_admin_panel_data);
        lv_checkbox_set_text(g_admin_cb_data_upload[i], ui_translation(g_data_upload_item_str_ids[i]));
        admin_data_style_checkbox(g_admin_cb_data_upload[i]);
        lv_obj_set_width(g_admin_cb_data_upload[i], 400);
        lv_obj_set_pos(g_admin_cb_data_upload[i], 200, 130 + i * 45);
    }

    for(int i = 0; i < 5; i++) {
        g_admin_cb_data_strategy[i] = lv_checkbox_create(g_admin_panel_data);
        lv_checkbox_set_text(g_admin_cb_data_strategy[i], ui_translation(g_data_strategy_str_ids[i]));
        admin_data_style_checkbox(g_admin_cb_data_strategy[i]);
        lv_obj_set_width(g_admin_cb_data_strategy[i], 380);
        lv_obj_set_pos(g_admin_cb_data_strategy[i], 960, 130 + i * 45);
        lv_obj_add_event_cb(g_admin_cb_data_strategy[i], cb_admin_data_strategy_pressed, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(g_admin_cb_data_strategy[i], cb_admin_data_strategy_changed, LV_EVENT_VALUE_CHANGED, NULL);
    }

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

    {
        const lv_coord_t net_btn_w = 280;
        const lv_coord_t net_btn_h = 90;
        const lv_coord_t net_gap = 48;
        const lv_coord_t net_row_w = net_btn_w * 2 + net_gap;
        const lv_coord_t net_x0 = (lv_coord_t)((UI_FIXED_W - net_row_w) / 2);
        const lv_coord_t net_y = 150;

        g_admin_btn_network_wifi = make_admin_menu_btn(g_admin_panel_network, ui_translation(STR_WIFI_SETTINGS));
        lv_obj_set_size(g_admin_btn_network_wifi, net_btn_w, net_btn_h);
        lv_obj_set_pos(g_admin_btn_network_wifi, net_x0, net_y);
        admin_menu_btn_bind_i18n(g_admin_btn_network_wifi, STR_WIFI_SETTINGS);
        lv_obj_add_event_cb(g_admin_btn_network_wifi, cb_admin_open_wifi_settings, LV_EVENT_CLICKED, NULL);

        g_admin_btn_network_4g = make_admin_menu_btn(g_admin_panel_network, ui_translation(STR_4G_SETTINGS));
        lv_obj_set_size(g_admin_btn_network_4g, net_btn_w, net_btn_h);
        lv_obj_set_pos(g_admin_btn_network_4g, net_x0 + net_btn_w + net_gap, net_y);
        admin_menu_btn_bind_i18n(g_admin_btn_network_4g, STR_4G_SETTINGS);
        lv_obj_add_event_cb(g_admin_btn_network_4g, cb_admin_open_4g_settings, LV_EVENT_CLICKED, NULL);
    }

    /* WIFI 设置子面板（1600×400，占位） */
    g_admin_panel_wifi = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_wifi, 1600, 400);
    lv_obj_align(g_admin_panel_wifi, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_bg_opa(g_admin_panel_wifi, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_wifi, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_wifi, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_wifi, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_wifi, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_wifi_title = lv_label_create(g_admin_panel_wifi);
    ui_lang_bind_label(g_admin_lbl_wifi_title, STR_WIFI_SETTINGS);
    lv_obj_set_style_text_color(g_admin_lbl_wifi_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_wifi_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_wifi_title, LV_ALIGN_TOP_LEFT, 350, 30);

    /* 4G 设置子面板（1600×400，三步配网） */
    g_admin_panel_4g = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_4g, 1600, 400);
    lv_obj_align(g_admin_panel_4g, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_bg_opa(g_admin_panel_4g, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_4g, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_4g, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_4g, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_4g, LV_OBJ_FLAG_HIDDEN);

    g_admin_lbl_4g_title = lv_label_create(g_admin_panel_4g);
    ui_lang_bind_label(g_admin_lbl_4g_title, STR_4G_SETTINGS);
    lv_obj_set_style_text_color(g_admin_lbl_4g_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_4g_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_4g_title, LV_ALIGN_TOP_LEFT, 350, 30);

    g_admin_sw_4g = lv_switch_create(g_admin_panel_4g);
    admin_4g_apply_switch_layout();
    lv_obj_add_event_cb(g_admin_sw_4g, cb_admin_4g_switch_changed, LV_EVENT_VALUE_CHANGED, NULL);

    g_admin_lbl_4g_prompt = lv_label_create(g_admin_panel_4g);
    ui_lang_bind_label(g_admin_lbl_4g_prompt, STR_4G_PROMPT);
    lv_obj_set_style_text_color(g_admin_lbl_4g_prompt, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_4g_prompt, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_4g_prompt, 1200);
    lv_label_set_long_mode(g_admin_lbl_4g_prompt, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(g_admin_lbl_4g_prompt, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(g_admin_lbl_4g_prompt, LV_ALIGN_CENTER, 0, 0);

    g_admin_lbl_4g_status = lv_label_create(g_admin_panel_4g);
    lv_label_set_text(g_admin_lbl_4g_status, ui_translation(STR_4G_PROVISIONING));
    lv_obj_set_style_text_color(g_admin_lbl_4g_status, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_4g_status, s_font_sc_30);
    lv_obj_set_width(g_admin_lbl_4g_status, 1200);
    lv_label_set_long_mode(g_admin_lbl_4g_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(g_admin_lbl_4g_status, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(g_admin_lbl_4g_status, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(g_admin_lbl_4g_status, LV_OBJ_FLAG_HIDDEN);

    g_admin_4g_done_center = lv_obj_create(g_admin_panel_4g);
    lv_obj_set_size(g_admin_4g_done_center, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(g_admin_4g_done_center, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(g_admin_4g_done_center, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_4g_done_center, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_4g_done_center, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(g_admin_4g_done_center, 16, LV_PART_MAIN);
    lv_obj_set_flex_flow(g_admin_4g_done_center, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(g_admin_4g_done_center, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(g_admin_4g_done_center, LV_OBJ_FLAG_HIDDEN);

    LV_IMAGE_DECLARE(end);
    g_admin_img_4g_done = lv_image_create(g_admin_4g_done_center);
    lv_image_set_src(g_admin_img_4g_done, &end);

    g_admin_lbl_4g_done = lv_label_create(g_admin_4g_done_center);
    ui_lang_bind_label(g_admin_lbl_4g_done, STR_4G_SUCCESS);
    lv_obj_set_style_text_color(g_admin_lbl_4g_done, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_4g_done, s_font_sc_30);

    /* 密码修改：步骤一原密码（布局仿管理员登录密码页） */
    g_admin_panel_pwd_chg_old = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_pwd_chg_old, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_pwd_chg_old, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_pwd_chg_old, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_pwd_chg_old, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_pwd_chg_old, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_pwd_chg_old, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_pwd_chg_old, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t * lbl_pwd_chg_old_title = lv_label_create(g_admin_panel_pwd_chg_old);
    ui_lang_bind_label(lbl_pwd_chg_old_title, STR_PWD_ENTER_OLD);
    lv_obj_set_style_text_color(lbl_pwd_chg_old_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_pwd_chg_old_title, s_font_sc_30);
    lv_obj_align(lbl_pwd_chg_old_title, LV_ALIGN_TOP_MID, 0, 24+50);

    g_admin_ta_pwd_chg_old = lv_textarea_create(g_admin_panel_pwd_chg_old);
    lv_obj_set_size(g_admin_ta_pwd_chg_old, 320, 48);
    lv_obj_align(g_admin_ta_pwd_chg_old, LV_ALIGN_TOP_MID, 0, 80+50);
    lv_textarea_set_one_line(g_admin_ta_pwd_chg_old, true);
    lv_textarea_set_password_mode(g_admin_ta_pwd_chg_old, true);
    lv_textarea_set_max_length(g_admin_ta_pwd_chg_old, 6);
    lv_textarea_set_accepted_chars(g_admin_ta_pwd_chg_old, "0123456789");
    lv_obj_set_style_text_font(g_admin_ta_pwd_chg_old, s_font_sc_30, LV_PART_MAIN);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_old, cb_admin_ta_ready, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_old, cb_admin_ta_key_enter, LV_EVENT_KEY | LV_EVENT_PREPROCESS, NULL);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_old, cb_admin_ta_kb_focus, LV_EVENT_ALL, NULL);

    g_admin_lbl_msg_pwd_chg_old = lv_label_create(g_admin_panel_pwd_chg_old);
    lv_obj_set_width(g_admin_lbl_msg_pwd_chg_old, LV_PCT(80));
    lv_obj_set_style_text_color(g_admin_lbl_msg_pwd_chg_old, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_align(g_admin_lbl_msg_pwd_chg_old, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_msg_pwd_chg_old, s_font_sc_30);
    lv_obj_align(g_admin_lbl_msg_pwd_chg_old, LV_ALIGN_TOP_MID, 0, 140+65);
    lv_obj_add_flag(g_admin_lbl_msg_pwd_chg_old, LV_OBJ_FLAG_HIDDEN);

    /* 密码修改：步骤二双新密码框，键盘 OK 切换/提交 */
    g_admin_panel_pwd_chg_new = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_pwd_chg_new, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_pwd_chg_new, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_pwd_chg_new, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_pwd_chg_new, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_pwd_chg_new, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_pwd_chg_new, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_pwd_chg_new, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t * lbl_pwd_chg_new_title = lv_label_create(g_admin_panel_pwd_chg_new);
    ui_lang_bind_label(lbl_pwd_chg_new_title, STR_PWD_ENTER_NEW);
    lv_obj_set_style_text_color(lbl_pwd_chg_new_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_pwd_chg_new_title, s_font_sc_30);
    lv_obj_align(lbl_pwd_chg_new_title, LV_ALIGN_TOP_MID, 0, 24+30);

    g_admin_ta_pwd_chg_new1 = lv_textarea_create(g_admin_panel_pwd_chg_new);
    lv_obj_set_size(g_admin_ta_pwd_chg_new1, 320, 48);//输入框1
    lv_obj_align(g_admin_ta_pwd_chg_new1, LV_ALIGN_TOP_MID, 0, 72+30);
    lv_textarea_set_one_line(g_admin_ta_pwd_chg_new1, true);
    lv_textarea_set_password_mode(g_admin_ta_pwd_chg_new1, true);
    lv_textarea_set_max_length(g_admin_ta_pwd_chg_new1, 6);
    lv_textarea_set_accepted_chars(g_admin_ta_pwd_chg_new1, "0123456789");
    lv_obj_set_style_text_font(g_admin_ta_pwd_chg_new1, s_font_sc_30, LV_PART_MAIN);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_new1, cb_admin_ta_ready, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_new1, cb_admin_ta_key_enter, LV_EVENT_KEY | LV_EVENT_PREPROCESS, NULL);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_new1, cb_admin_ta_kb_focus, LV_EVENT_ALL, NULL);

    g_admin_ta_pwd_chg_new2 = lv_textarea_create(g_admin_panel_pwd_chg_new);
    lv_obj_set_size(g_admin_ta_pwd_chg_new2, 320, 48);//输入框2
    lv_obj_align(g_admin_ta_pwd_chg_new2, LV_ALIGN_TOP_MID, 0, 142+30);
    lv_textarea_set_one_line(g_admin_ta_pwd_chg_new2, true);
    lv_textarea_set_password_mode(g_admin_ta_pwd_chg_new2, true);
    lv_textarea_set_max_length(g_admin_ta_pwd_chg_new2, 6);
    lv_textarea_set_accepted_chars(g_admin_ta_pwd_chg_new2, "0123456789");
    lv_obj_set_style_text_font(g_admin_ta_pwd_chg_new2, s_font_sc_30, LV_PART_MAIN);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_new2, cb_admin_ta_ready, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_new2, cb_admin_ta_key_enter, LV_EVENT_KEY | LV_EVENT_PREPROCESS, NULL);
    lv_obj_add_event_cb(g_admin_ta_pwd_chg_new2, cb_admin_ta_kb_focus, LV_EVENT_ALL, NULL);

    g_admin_lbl_msg_pwd_chg_new = lv_label_create(g_admin_panel_pwd_chg_new);
    lv_obj_set_width(g_admin_lbl_msg_pwd_chg_new, LV_PCT(80));
    lv_obj_set_style_text_color(g_admin_lbl_msg_pwd_chg_new, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_align(g_admin_lbl_msg_pwd_chg_new, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_msg_pwd_chg_new, s_font_sc_30);
    lv_obj_align(g_admin_lbl_msg_pwd_chg_new, LV_ALIGN_TOP_MID, 0, 212+30);
    lv_obj_add_flag(g_admin_lbl_msg_pwd_chg_new, LV_OBJ_FLAG_HIDDEN);

    /* 机器 ID 子面板 */
    g_admin_panel_machine_id = lv_obj_create(root);
    lv_obj_set_size(g_admin_panel_machine_id, LV_PCT(100), body_h);
    lv_obj_align(g_admin_panel_machine_id, LV_ALIGN_TOP_MID, 0, body_y);
    lv_obj_set_style_bg_opa(g_admin_panel_machine_id, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_admin_panel_machine_id, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_admin_panel_machine_id, 0, LV_PART_MAIN);
    lv_obj_set_style_layout(g_admin_panel_machine_id, LV_LAYOUT_NONE, LV_PART_MAIN);
    lv_obj_add_flag(g_admin_panel_machine_id, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t * lbl_mid_title = lv_label_create(g_admin_panel_machine_id);//机器ID标题
    ui_lang_bind_label(lbl_mid_title, STR_MACHINE_ID_TITLE);
    lv_obj_set_style_text_color(lbl_mid_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_mid_title, s_font_sc_30);
    lv_obj_align(lbl_mid_title, LV_ALIGN_TOP_MID, 0, 16);

    g_admin_lbl_machine_id_cur = lv_label_create(g_admin_panel_machine_id);//机器ID当前值
    lv_obj_set_style_text_color(g_admin_lbl_machine_id_cur, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_machine_id_cur, s_font_sc_30);
    lv_obj_align(g_admin_lbl_machine_id_cur, LV_ALIGN_TOP_MID, 0, 56+10);
    admin_machine_id_label_update();

    g_admin_ta_machine_id = lv_textarea_create(g_admin_panel_machine_id);//机器ID输入框
    lv_obj_set_size(g_admin_ta_machine_id, 320, 48);
    lv_obj_align(g_admin_ta_machine_id, LV_ALIGN_TOP_MID, 0, 100+10);
    lv_textarea_set_one_line(g_admin_ta_machine_id, true);
    lv_textarea_set_max_length(g_admin_ta_machine_id, 6);
    lv_textarea_set_accepted_chars(g_admin_ta_machine_id, "0123456789");
    lv_obj_set_style_text_font(g_admin_ta_machine_id, s_font_sc_30, LV_PART_MAIN);
    lv_obj_add_event_cb(g_admin_ta_machine_id, cb_admin_ta_ready, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(g_admin_ta_machine_id, cb_admin_ta_kb_focus, LV_EVENT_ALL, NULL);

    g_admin_btn_machine_confirm = make_orange_fill_btn(g_admin_panel_machine_id, ui_translation(STR_BTN_CONFIRM), 160, 44);//确认按钮
    lv_obj_align(g_admin_btn_machine_confirm, LV_ALIGN_TOP_MID, 0, 165+15);
    ui_set_obj_font(lv_obj_get_child(g_admin_btn_machine_confirm, 0), s_font_sc_30);
    orange_btn_bind_i18n(g_admin_btn_machine_confirm, STR_BTN_CONFIRM);
    lv_obj_add_event_cb(g_admin_btn_machine_confirm, cb_admin_machine_confirm, LV_EVENT_CLICKED, NULL);

    g_admin_lbl_msg_machine_id = lv_label_create(g_admin_panel_machine_id);//机器ID错误提示
    lv_obj_set_width(g_admin_lbl_msg_machine_id, LV_PCT(80));
    lv_obj_set_style_text_color(g_admin_lbl_msg_machine_id, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_align(g_admin_lbl_msg_machine_id, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_msg_machine_id, s_font_sc_30);
    lv_obj_align(g_admin_lbl_msg_machine_id, LV_ALIGN_TOP_MID, 0, 220+15);
    lv_obj_add_flag(g_admin_lbl_msg_machine_id, LV_OBJ_FLAG_HIDDEN);

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
    ui_lang_bind_label(g_admin_lbl_vendor_serial_title, STR_ADMIN_M1_VENDOR_MAINT);
    lv_obj_set_style_text_color(g_admin_lbl_vendor_serial_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(g_admin_lbl_vendor_serial_title, s_font_sc_30);
    lv_obj_align(g_admin_lbl_vendor_serial_title, LV_ALIGN_TOP_MID, 0, 24 + 50);

    lv_obj_t * lbl_vendor_hint = lv_label_create(g_admin_panel_vendor_serial);
    ui_lang_bind_label(lbl_vendor_hint, STR_VENDOR_SERIAL_HINT);
    lv_obj_set_style_text_color(lbl_vendor_hint, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_vendor_hint, s_font_sc_30);
    lv_obj_align(lbl_vendor_hint, LV_ALIGN_TOP_MID, 0, 64 + 50);

    g_admin_ta_vendor_serial = lv_textarea_create(g_admin_panel_vendor_serial);
    lv_obj_set_size(g_admin_ta_vendor_serial, 320, 48);
    lv_obj_align(g_admin_ta_vendor_serial, LV_ALIGN_TOP_MID, 0, 110 + 50);
    lv_textarea_set_one_line(g_admin_ta_vendor_serial, true);
    lv_textarea_set_max_length(g_admin_ta_vendor_serial, 6);
    lv_textarea_set_accepted_chars(g_admin_ta_vendor_serial, "0123456789");
    lv_obj_set_style_text_font(g_admin_ta_vendor_serial, s_font_sc_30, LV_PART_MAIN);
    lv_obj_add_event_cb(g_admin_ta_vendor_serial, cb_admin_ta_ready, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(g_admin_ta_vendor_serial, cb_admin_ta_key_enter, LV_EVENT_KEY | LV_EVENT_PREPROCESS, NULL);
    lv_obj_add_event_cb(g_admin_ta_vendor_serial, cb_admin_ta_kb_focus, LV_EVENT_ALL, NULL);

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
    ui_lang_bind_label(lbl_vendor_menu_title, STR_ADMIN_M1_VENDOR_MAINT);
    lv_obj_set_style_text_color(lbl_vendor_menu_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
    ui_set_obj_font(lbl_vendor_menu_title, s_font_sc_30);
    lv_obj_align(lbl_vendor_menu_title, LV_ALIGN_TOP_MID, 0, 8);

    g_admin_btn_vendor_self_check = make_admin_menu_btn(g_admin_panel_vendor_menu, ui_translation(STR_VENDOR_SELF_CHECK));
    lv_obj_set_size(g_admin_btn_vendor_self_check, btn_w, btn_h);
    lv_obj_align(g_admin_btn_vendor_self_check, LV_ALIGN_CENTER, 0, -30);
    admin_menu_btn_bind_i18n(g_admin_btn_vendor_self_check, STR_VENDOR_SELF_CHECK);
    lv_obj_add_event_cb(g_admin_btn_vendor_self_check, cb_admin_open_selfcheck, LV_EVENT_CLICKED, NULL);

    g_admin_btn_vendor_self_learn = make_admin_menu_btn(g_admin_panel_vendor_menu, ui_translation(STR_VENDOR_SELF_LEARN));
    lv_obj_set_size(g_admin_btn_vendor_self_learn, btn_w, btn_h);
    lv_obj_align(g_admin_btn_vendor_self_learn, LV_ALIGN_CENTER, 0, 110);
    admin_menu_btn_bind_i18n(g_admin_btn_vendor_self_learn, STR_VENDOR_SELF_LEARN);
    lv_obj_add_event_cb(g_admin_btn_vendor_self_learn, cb_admin_open_cycle, LV_EVENT_CLICKED, NULL);

    program_admin_build_panel(root, body_y, body_h);

    /* 自定义键盘区：1600×150，底部对齐，12 个独立按钮 */
    {
        g_admin_kb = lv_obj_create(root);
        lv_obj_set_size(g_admin_kb, 1600, 150);
        lv_obj_align(g_admin_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_color(g_admin_kb, lv_color_hex(0x0c0c0c), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(g_admin_kb, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(g_admin_kb, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(g_admin_kb, 0, LV_PART_MAIN);
        lv_obj_set_style_layout(g_admin_kb, LV_LAYOUT_NONE, LV_PART_MAIN);

        /* 计算总宽使其居中：10×64 + 2×108 + 11×52 = 1428, 左右各 (1600-1428)/2 = 86 */
        const lv_coord_t btn_w = 64, btn_h = 64, btn_w_icon = 108, gap = 52;
        const lv_coord_t total_w = 10 * btn_w + 2 * btn_w_icon + 11 * gap;
        const lv_coord_t start_x = (1600 - total_w) / 2;
        const lv_coord_t y_center = (150 - btn_h) / 2;
        lv_coord_t x = start_x;

        static const char * const digits[] = {"1","2","3","4","5","6","7","8","9","0"};
        for(int i = 0; i < 10; i++) {
            lv_obj_t * btn = lv_button_create(g_admin_kb);
            lv_obj_set_size(btn, btn_w, btn_h);
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
            x += btn_w + gap;
        }
        /* 删除按钮 */
        {
            lv_obj_t * btn = lv_button_create(g_admin_kb);
            lv_obj_set_size(btn, btn_w_icon, btn_h);
            lv_obj_set_pos(btn, x, y_center);
            lv_obj_set_style_radius(btn, 8, LV_PART_MAIN);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x545454), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN);
            lv_obj_add_event_cb(btn, cb_admin_kb_btn, LV_EVENT_CLICKED, "DEL");
            lv_obj_t * img = lv_image_create(btn);
            lv_image_set_src(img, &delete);
            lv_obj_center(img);
            x += btn_w_icon + gap;
        }
        /* 确认按钮 */
        {
            lv_obj_t * btn = lv_button_create(g_admin_kb);
            lv_obj_set_size(btn, btn_w_icon, btn_h);
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
        lv_obj_add_flag(g_admin_kb, LV_OBJ_FLAG_HIDDEN);
    }

    admin_panel_show(PASSWORD);
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

		lv_obj_t * top = create_top_bar(root, &g_lbl_clock_pay, g_scr_home, g_group_pay, &g_pay_btn_back);
		lv_obj_t * btn_runpause = add_encoder_top_btn(top, "启停", 100, g_group_pay);
#if UI_DEBUG_PAY_SKIP_TO_DONE
		lv_obj_add_event_cb(btn_runpause, cb_pay_debug_skip_to_done, LV_EVENT_CLICKED, NULL);
#else
		lv_obj_add_event_cb(btn_runpause, cb_runpause, LV_EVENT_CLICKED, g_scr_home);
#endif
		lv_obj_add_event_cb(btn_runpause, cb_runpause_long, LV_EVENT_LONG_PRESSED, g_scr_home);
		lv_obj_t * btn_power = add_encoder_top_btn(top, "电源", 180, g_group_pay);
		lv_obj_add_event_cb(btn_power, cb_power_long, LV_EVENT_LONG_PRESSED, NULL);

		lv_obj_t * center = lv_obj_create(root);           /* 纵向：二维码 → 金额 → 提示 */
		lv_obj_set_size(center, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
		lv_obj_align(center, LV_ALIGN_CENTER, 0, 0);
		lv_obj_set_style_bg_opa(center, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(center, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(center, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_row(center, 20, LV_PART_MAIN);
		lv_obj_set_flex_flow(center, LV_FLEX_FLOW_COLUMN);
		lv_obj_set_flex_align(center, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

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

		lv_obj_t * top = create_top_bar(root, &g_lbl_clock_pay_done, g_scr_home, g_group_pay_done, NULL);
		lv_obj_t * btn_runpause = add_encoder_top_btn(top, "启停", 100, g_group_pay_done);
		lv_obj_add_event_cb(btn_runpause, cb_runpause, LV_EVENT_CLICKED, NULL);
		lv_obj_add_event_cb(btn_runpause, cb_runpause_long, LV_EVENT_LONG_PRESSED, NULL);
		lv_obj_t * btn_power = add_encoder_top_btn(top, "电源", 180, g_group_pay_done);
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

		LV_IMAGE_DECLARE(end);
		lv_obj_t * img_done = lv_image_create(center);      /* 与结束页共用 end 图标 */
		lv_image_set_src(img_done, &end);

		/* 主文字：中「支付完成」/ 英「Payment Done」（STR_PAY_DONE） */
		g_lbl_pay_done = lv_label_create(center);
		lv_obj_set_style_text_color(g_lbl_pay_done, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(g_lbl_pay_done, s_font_sc_50);
		ui_lang_bind_label(g_lbl_pay_done, STR_PAY_DONE);
}








/* ========== 工厂自检 / 循环程序（自烘干版移植，商用洗 cycle 适配） ========== */

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

static void selfcheck_indicator_blink_stop(void);

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
	if(g_selfcheck_lbl_phase != NULL) {
		lv_obj_set_style_text_opa(g_selfcheck_lbl_phase, LV_OPA_COVER, LV_PART_MAIN);
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

/* 初始化页三列占位：转速 / 温度 / 漂洗次数，三闪时整行同步亮灭 */
static void selfcheck_init_flash_set(bool on)
{
	const lv_opa_t opa = on ? LV_OPA_COVER : LV_OPA_TRANSP;

	if(g_selfcheck_lbl_init_rpm != NULL) {
		lv_obj_set_style_text_opa(g_selfcheck_lbl_init_rpm, opa, LV_PART_MAIN);
	}
	if(g_selfcheck_lbl_init_temp != NULL) {
		lv_obj_set_style_text_opa(g_selfcheck_lbl_init_temp, opa, LV_PART_MAIN);
	}
	if(g_selfcheck_lbl_init_rinse != NULL) {
		lv_obj_set_style_text_opa(g_selfcheck_lbl_init_rinse, opa, LV_PART_MAIN);
	}
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
	if(g_selfcheck_lbl_phase != NULL) {
		lv_obj_add_flag(g_selfcheck_lbl_phase, LV_OBJ_FLAG_HIDDEN);
	}

	if(step == SELFCHECK_UI_MODEL || step == SELFCHECK_UI_FLASH) {
		if(g_selfcheck_lbl_doorlock != NULL) {
			selfcheck_doorlock_set(g_selfcheck_lbl_doorlock, true);
			lv_obj_remove_flag(g_selfcheck_lbl_doorlock, LV_OBJ_FLAG_HIDDEN);
			lv_obj_move_foreground(g_selfcheck_lbl_doorlock);
		}
		return;
	}

	if(step == SELFCHECK_UI_INIT) {
		return;
	}

	if(step != SELFCHECK_UI_S1 && step != SELFCHECK_UI_S2 && step != SELFCHECK_UI_S3 &&
	   step != SELFCHECK_UI_S4 && step != SELFCHECK_UI_S5) {
		return;
	}

	if(g_selfcheck_lbl_doorlock != NULL) {
		lv_obj_remove_flag(g_selfcheck_lbl_doorlock, LV_OBJ_FLAG_HIDDEN);
		lv_obj_move_foreground(g_selfcheck_lbl_doorlock);
		selfcheck_doorlock_set(g_selfcheck_lbl_doorlock, step == SELFCHECK_UI_S1);
	}

	if(step >= SELFCHECK_UI_S2 && step <= SELFCHECK_UI_S5) {
		if(g_selfcheck_row_status != NULL) {
			lv_obj_remove_flag(g_selfcheck_row_status, LV_OBJ_FLAG_HIDDEN);
		}
		if(g_selfcheck_lbl_temp_live != NULL) {
			selfcheck_temp_label_sync();
			lv_obj_remove_flag(g_selfcheck_lbl_temp_live, LV_OBJ_FLAG_HIDDEN);
		}
		if(g_selfcheck_row_indicator != NULL) {
			lv_obj_remove_flag(g_selfcheck_row_indicator, LV_OBJ_FLAG_HIDDEN);
		}
	}

	if(g_selfcheck_lbl_phase != NULL) {
		if(step == SELFCHECK_UI_S2 || step == SELFCHECK_UI_S3) {
			lv_label_set_text(g_selfcheck_lbl_phase, "wash");
			lv_obj_remove_flag(g_selfcheck_lbl_phase, LV_OBJ_FLAG_HIDDEN);
			selfcheck_indicator_blink_start(g_selfcheck_lbl_phase);
		}
		else if(step == SELFCHECK_UI_S4 || step == SELFCHECK_UI_S5) {
			lv_label_set_text(g_selfcheck_lbl_phase, "spin-drying");
			lv_obj_remove_flag(g_selfcheck_lbl_phase, LV_OBJ_FLAG_HIDDEN);
			selfcheck_indicator_blink_start(g_selfcheck_lbl_phase);
		}
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
		g_selfcheck_init_flash_on = !g_selfcheck_init_flash_on;
		if(g_selfcheck_init_flash_on) {
			selfcheck_init_flash_set(true);
		}
		else {
			selfcheck_init_flash_set(false);
			g_selfcheck_init_flash_count++;
		}
		if(g_selfcheck_init_flash_count >= 3u) {
			selfcheck_ui_goto_step(SELFCHECK_UI_S1);
			return;
		}
		g_selfcheck_timer = lv_timer_create(cb_selfcheck_timer, SELFCHECK_FLASH_MS, NULL);
		return;
	}

	if(g_selfcheck_ui_state == SELFCHECK_UI_S1) {
		selfcheck_ui_goto_step(SELFCHECK_UI_S2);
	}
	else if(g_selfcheck_ui_state == SELFCHECK_UI_S2) {
		selfcheck_ui_goto_step(SELFCHECK_UI_S3);
	}
	else if(g_selfcheck_ui_state == SELFCHECK_UI_S3) {
		selfcheck_ui_goto_step(SELFCHECK_UI_S4);
	}
	else if(g_selfcheck_ui_state == SELFCHECK_UI_S4) {
		selfcheck_ui_goto_step(SELFCHECK_UI_S5);
	}
	else if(g_selfcheck_ui_state == SELFCHECK_UI_S5) {
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
		selfcheck_panel_hide_all();
		g_selfcheck_init_flash_count = 0u;
		g_selfcheck_init_flash_on = false;
		if(g_selfcheck_panel_init != NULL) {
			lv_obj_remove_flag(g_selfcheck_panel_init, LV_OBJ_FLAG_HIDDEN);
			lv_obj_move_foreground(g_selfcheck_panel_init);
		}
		selfcheck_init_flash_set(false);
		g_selfcheck_timer = lv_timer_create(cb_selfcheck_timer, SELFCHECK_FLASH_MS, NULL);
		break;
	case SELFCHECK_UI_S1:
		selfcheck_panel_hide_all();
		if(g_selfcheck_panel_steps != NULL) {
			lv_obj_remove_flag(g_selfcheck_panel_steps, LV_OBJ_FLAG_HIDDEN);
			lv_obj_move_foreground(g_selfcheck_panel_steps);
		}
		selfcheck_step_highlight(0);
		selfcheck_schedule_step_timer();
		break;
	case SELFCHECK_UI_S2:
		selfcheck_step_highlight(1);
		selfcheck_schedule_step_timer();
		break;
	case SELFCHECK_UI_S3:
		selfcheck_step_highlight(2);
		selfcheck_schedule_step_timer();
		break;
	case SELFCHECK_UI_S4:
		selfcheck_step_highlight(3);
		selfcheck_schedule_step_timer();
		break;
	case SELFCHECK_UI_S5:
		selfcheck_step_highlight(4);
		selfcheck_schedule_step_timer();
		break;
	case SELFCHECK_UI_DONE:
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
	g_selfcheck_flash_count = 0u;
	g_selfcheck_flash_on = false;
	g_selfcheck_init_flash_count = 0u;
	g_selfcheck_init_flash_on = false;
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

	lv_obj_t * top = create_top_bar(root, &g_lbl_clock_selfcheck, NULL, NULL, NULL);

	LV_IMAGE_DECLARE(back);
	g_selfcheck_btn_back = lv_imgbtn_create(top);
	lv_imgbtn_set_src(g_selfcheck_btn_back, LV_IMGBTN_STATE_RELEASED, NULL, &back, NULL);
	lv_obj_align(g_selfcheck_btn_back, LV_ALIGN_LEFT_MID, 20, 0);
	lv_obj_remove_flag(g_selfcheck_btn_back, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_add_event_cb(g_selfcheck_btn_back, cb_selfcheck_back, LV_EVENT_CLICKED, NULL);

	g_selfcheck_btn_runpause = add_encoder_top_btn(top, "启停", 100, NULL);
	lv_obj_add_event_cb(g_selfcheck_btn_runpause, cb_selfcheck_runpause, LV_EVENT_CLICKED, NULL);
	g_selfcheck_btn_power = add_encoder_top_btn(top, "电源", 180, NULL);

	lv_obj_t * lbl_title = lv_label_create(root);
	ui_lang_bind_label(lbl_title, STR_SELF_CHECK_TITLE);
	lv_obj_set_style_text_color(lbl_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(lbl_title, s_font_sc_30);
	lv_obj_align(lbl_title, LV_ALIGN_TOP_MID, 0, body_y + 8);

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

	g_selfcheck_lbl_init_rpm = lv_label_create(g_selfcheck_panel_init);
	lv_label_set_text(g_selfcheck_lbl_init_rpm, "—— —— ——");
	lv_obj_set_width(g_selfcheck_lbl_init_rpm, LV_SIZE_CONTENT);
	lv_obj_set_style_text_color(g_selfcheck_lbl_init_rpm, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_set_style_text_align(g_selfcheck_lbl_init_rpm, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	ui_set_obj_font(g_selfcheck_lbl_init_rpm, s_font_sc_35);
	lv_obj_align(g_selfcheck_lbl_init_rpm, LV_ALIGN_CENTER, -280, 0);
	lv_obj_set_style_text_opa(g_selfcheck_lbl_init_rpm, LV_OPA_TRANSP, LV_PART_MAIN);

	g_selfcheck_lbl_init_temp = lv_label_create(g_selfcheck_panel_init);
	lv_label_set_text(g_selfcheck_lbl_init_temp, "HOT WARM COLD");
	lv_obj_set_width(g_selfcheck_lbl_init_temp, LV_SIZE_CONTENT);
	lv_obj_set_style_text_color(g_selfcheck_lbl_init_temp, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_set_style_text_align(g_selfcheck_lbl_init_temp, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	ui_set_obj_font(g_selfcheck_lbl_init_temp, s_font_sc_35);
	lv_obj_align(g_selfcheck_lbl_init_temp, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_text_opa(g_selfcheck_lbl_init_temp, LV_OPA_TRANSP, LV_PART_MAIN);

	g_selfcheck_lbl_init_rinse = lv_label_create(g_selfcheck_panel_init);
	lv_label_set_text(g_selfcheck_lbl_init_rinse, "—— —— ——");
	lv_obj_set_width(g_selfcheck_lbl_init_rinse, LV_SIZE_CONTENT);
	lv_obj_set_style_text_color(g_selfcheck_lbl_init_rinse, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	lv_obj_set_style_text_align(g_selfcheck_lbl_init_rinse, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	ui_set_obj_font(g_selfcheck_lbl_init_rinse, s_font_sc_35);
	lv_obj_align(g_selfcheck_lbl_init_rinse, LV_ALIGN_CENTER, 280, 0);
	lv_obj_set_style_text_opa(g_selfcheck_lbl_init_rinse, LV_OPA_TRANSP, LV_PART_MAIN);

	g_selfcheck_panel_steps = lv_obj_create(root);
	lv_obj_set_size(g_selfcheck_panel_steps, LV_PCT(100), LV_PCT(100));
	lv_obj_set_pos(g_selfcheck_panel_steps, 0, 0);
	lv_obj_set_style_bg_opa(g_selfcheck_panel_steps, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(g_selfcheck_panel_steps, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(g_selfcheck_panel_steps, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(g_selfcheck_panel_steps, LV_LAYOUT_NONE, LV_PART_MAIN);
	lv_obj_remove_flag(g_selfcheck_panel_steps, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(g_selfcheck_panel_steps, LV_OBJ_FLAG_HIDDEN);

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

	g_selfcheck_row_indicator = lv_obj_create(g_selfcheck_panel_steps);
	lv_obj_set_size(g_selfcheck_row_indicator, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_align(g_selfcheck_row_indicator, LV_ALIGN_BOTTOM_MID, 0, -100);
	lv_obj_set_style_bg_opa(g_selfcheck_row_indicator, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(g_selfcheck_row_indicator, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(g_selfcheck_row_indicator, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_column(g_selfcheck_row_indicator, 32, LV_PART_MAIN);
	lv_obj_set_flex_flow(g_selfcheck_row_indicator, LV_FLEX_FLOW_ROW);
	lv_obj_set_flex_align(g_selfcheck_row_indicator, LV_FLEX_ALIGN_CENTER,
		LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
	lv_obj_remove_flag(g_selfcheck_row_indicator, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(g_selfcheck_row_indicator, LV_OBJ_FLAG_HIDDEN);

	g_selfcheck_lbl_phase = lv_label_create(g_selfcheck_row_indicator);
	lv_label_set_text(g_selfcheck_lbl_phase, "wash");
	lv_obj_set_style_text_color(g_selfcheck_lbl_phase, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(g_selfcheck_lbl_phase, s_font_sc_35);
	lv_obj_add_flag(g_selfcheck_lbl_phase, LV_OBJ_FLAG_HIDDEN);

	{
		const lv_coord_t step_side_margin = (lv_coord_t)((UI_FIXED_W * 4) / 100);
		const lv_coord_t step_gap = (lv_coord_t)((UI_FIXED_W * 3) / 100);
		const lv_coord_t step_row_w = (lv_coord_t)UI_FIXED_W - step_side_margin * 2;
		const lv_coord_t step_btn_w =
			(step_row_w - step_gap * (SELFCHECK_STEP_COUNT - 1)) / SELFCHECK_STEP_COUNT;
		const lv_coord_t step_btn_h = 88;
		static const char * step_labels[SELFCHECK_STEP_COUNT] = {
			"S-1", "S-2", "S-3", "S-4", "S-5"
		};

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
			g_selfcheck_step_btns[i] = make_admin_menu_btn(steps_row, step_labels[i]);
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

	LV_IMAGE_DECLARE(end);
	lv_obj_t * img_done = lv_image_create(center);
	lv_image_set_src(img_done, &end);

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

/* ========== 循环程序（商用洗：程序 Tab + 完成次数） ========== */

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

static void cycle_kb_close(void)
{
	(void)0;
}

static void cycle_ui_reset(void)
{
	g_cycle_active = false;
	g_cycle_ui_state = CYCLE_UI_SETUP;
	g_cycle_fault_on_running = false;
	lv_memzero(&g_cycle_session, sizeof(g_cycle_session));
	cycle_fault_blink_stop();
	cycle_kb_close();
}

static void cycle_session_apply_to_running(void)
{
	g_wheel_sel = g_cycle_session.prog_idx;
	g_prog_cfg[g_wheel_sel] = g_cycle_session.cfg;
	program_admin_apply_one(g_wheel_sel);
}

static void cycle_session_snapshot_from_ui(void)
{
	g_cycle_session.prog_idx = g_cycle_prog_sel;
	g_cycle_session.cfg = g_cycle_cfg[g_cycle_prog_sel];
}

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

void ui_cycle_mode_enter(void)
{
	/* TODO: MCU — 进入寿命试验，下发锁定程序参数 */
}

void ui_cycle_mode_exit(void)
{
	/* TODO: MCU — 退出寿命试验 */
}

static void cycle_start_run(void);

static void cycle_on_run_finished(void)
{
	if(g_cycle_session.fault_code != 0u) {
		cycle_enter_fault_display();
		return;
	}
	cycle_start_run();
}

static void cycle_start_run(void)
{
	if(!g_cycle_active) return;

	if(g_cycle_ui_state != CYCLE_UI_RUNNING) {
		cycle_session_snapshot_from_ui();
	}
	cycle_session_apply_to_running();
	g_cycle_ui_state = CYCLE_UI_RUNNING;
	ui_cycle_mode_enter();
	fsm_state_change(FSM_RUNNING);
	ui_screen_load(g_scr_running);
}

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

static void cycle_finish_current_run(void)
{
	if(g_cycle_ui_state != CYCLE_UI_RUNNING) return;
	running_countdown_reset_all();
	g_cycle_session.run_count++;
	cycle_run_count_label_sync();
	cycle_on_run_finished();
}

static void cycle_exit_to_vendor_menu(void)
{
	cycle_kb_close();
	cycle_ui_reset();
	ui_cycle_mode_exit();
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
	cycle_start_run();
}

static void cycle_wash_sync_prog_pick_ui(void)
{
	admin_prog_btn_style_init();
	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		lv_obj_t * b = g_cycle_prog_btns[i];
		if(b == NULL) continue;
		if(i == g_cycle_prog_sel) {
			lv_obj_remove_style(b, &s_admin_prog_btn_style, LV_PART_MAIN);
			lv_obj_add_style(b, &s_admin_prog_btn_sel_style, LV_PART_MAIN);
		} else {
			lv_obj_remove_style(b, &s_admin_prog_btn_sel_style, LV_PART_MAIN);
			lv_obj_add_style(b, &s_admin_prog_btn_style, LV_PART_MAIN);
		}
	}
}

static void cb_cycle_prog_pick(lv_event_t * e)
{
	int32_t idx = (int32_t)(intptr_t)lv_event_get_user_data(e);
	if(idx < 0 || idx >= TOTAL_PROGRAMS) return;
	g_cycle_prog_sel = idx;
	cycle_wash_sync_prog_pick_ui();
}

static void cycle_wash_build_program_panel(lv_obj_t * panel, lv_coord_t panel_h)
{
	lv_obj_set_size(panel, LV_PCT(100), panel_h);
	lv_obj_set_style_bg_opa(panel, LV_OPA_TRANSP, LV_PART_MAIN);
	lv_obj_set_style_border_width(panel, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(panel, 0, LV_PART_MAIN);
	lv_obj_set_style_layout(panel, LV_LAYOUT_NONE, LV_PART_MAIN);

	const lv_coord_t prog_btn_w = 140;
	const lv_coord_t prog_btn_h = 48;
	const lv_coord_t prog_gap = 12;
	const lv_coord_t prog_row_w = TOTAL_PROGRAMS * prog_btn_w + (TOTAL_PROGRAMS - 1) * prog_gap;
	const lv_coord_t prog_row_x0 = (lv_coord_t)((UI_FIXED_W - prog_row_w) / 2);
	const lv_coord_t prog_row_y = 66;

	const lv_coord_t count_box_w = 200;
	const lv_coord_t count_box_x = prog_row_x0 + prog_row_w + 12;
	lv_obj_t * count_box = program_admin_make_value_box(panel, count_box_w, prog_btn_h);
	lv_obj_set_pos(count_box, count_box_x, prog_row_y);
	g_cycle_lbl_run_count = lv_label_create(count_box);
	lv_obj_set_width(g_cycle_lbl_run_count, LV_PCT(100));
	lv_label_set_long_mode(g_cycle_lbl_run_count, LV_LABEL_LONG_CLIP);
	lv_obj_set_style_text_align(g_cycle_lbl_run_count, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	ui_set_obj_font(g_cycle_lbl_run_count, s_font_sc_30);
	lv_obj_set_style_text_color(g_cycle_lbl_run_count, lv_color_hex(0x333333), LV_PART_MAIN);
	cycle_run_count_label_sync();

	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		g_cycle_prog_btns[i] = make_admin_prog_btn(panel, ui_program_name_get(i));
		admin_prog_btn_bind_i18n(g_cycle_prog_btns[i], g_mode_name_ids[i]);
		lv_obj_set_size(g_cycle_prog_btns[i], prog_btn_w, prog_btn_h);
		lv_obj_set_pos(g_cycle_prog_btns[i], prog_row_x0 + i * (prog_btn_w + prog_gap), prog_row_y);
		lv_obj_add_event_cb(g_cycle_prog_btns[i], cb_cycle_prog_pick, LV_EVENT_CLICKED, (void *)(intptr_t)i);
	}
	g_cycle_prog_sel = 0;
	cycle_wash_sync_prog_pick_ui();
}

static void cycle_encoder_group_build(void)
{
	if(g_group_cycle == NULL) return;
	lv_group_remove_all_objs(g_group_cycle);
	if(g_cycle_btn_back != NULL) ui_encoder_group_add(g_group_cycle, g_cycle_btn_back);
	if(g_cycle_btn_runpause != NULL) ui_encoder_group_add(g_group_cycle, g_cycle_btn_runpause);
	if(g_cycle_btn_power != NULL) ui_encoder_group_add(g_group_cycle, g_cycle_btn_power);
	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		if(g_cycle_prog_btns[i] != NULL) ui_encoder_group_add(g_group_cycle, g_cycle_prog_btns[i]);
	}
	lv_group_set_wrap(g_group_cycle, false);
	if(g_cycle_btn_back != NULL) lv_group_focus_obj(g_cycle_btn_back);
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

	lv_obj_t * top = create_top_bar(root, &g_lbl_clock_cycle, NULL, NULL, NULL);

	LV_IMAGE_DECLARE(back);
	g_cycle_btn_back = lv_imgbtn_create(top);
	lv_imgbtn_set_src(g_cycle_btn_back, LV_IMGBTN_STATE_RELEASED, NULL, &back, NULL);
	lv_obj_align(g_cycle_btn_back, LV_ALIGN_LEFT_MID, 20, 0);
	lv_obj_remove_flag(g_cycle_btn_back, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_add_event_cb(g_cycle_btn_back, cb_cycle_back, LV_EVENT_CLICKED, NULL);

	g_cycle_btn_runpause = add_encoder_top_btn(top, "启停", 100, g_group_cycle);
	lv_obj_add_event_cb(g_cycle_btn_runpause, cb_cycle_runpause, LV_EVENT_CLICKED, NULL);
	g_cycle_btn_power = add_encoder_top_btn(top, "电源", 180, g_group_cycle);
	lv_obj_add_event_cb(g_cycle_btn_power, cb_power_long, LV_EVENT_LONG_PRESSED, NULL);

	lv_obj_t * lbl_title = lv_label_create(root);
	ui_lang_bind_label(lbl_title, STR_CYCLE_TITLE);
	lv_obj_set_style_text_color(lbl_title, lv_color_hex(COL_TEXT), LV_PART_MAIN);
	ui_set_obj_font(lbl_title, s_font_sc_30);
	lv_obj_align(lbl_title, LV_ALIGN_TOP_MID, 0, body_y + 4);

	g_cycle_panel_program = lv_obj_create(root);
	lv_obj_set_size(g_cycle_panel_program, LV_PCT(100), panel_h);
	lv_obj_align(g_cycle_panel_program, LV_ALIGN_TOP_MID, 0, body_y);
	cycle_wash_build_program_panel(g_cycle_panel_program, panel_h);

	cycle_fault_overlay_create(root, &g_cycle_panel_fault, &g_cycle_lbl_fault_alt);
	cycle_encoder_group_build();
}

void ui_cycle_run_complete(void)
{
	if(!g_cycle_active) return;
	cycle_finish_current_run();
}

uint32_t ui_cycle_run_count_get(void)
{
	return g_cycle_session.run_count;
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
		lv_obj_t * top = create_top_bar(row, &g_lbl_clock_off, NULL, g_group_off, NULL);

		LV_IMAGE_DECLARE(back);
		lv_obj_t * btn_back = lv_imgbtn_create(top);
		lv_imgbtn_set_src(btn_back, LV_IMGBTN_STATE_RELEASED, NULL, &back, NULL);
		lv_obj_align(btn_back, LV_ALIGN_LEFT_MID, 20, 0);
		lv_obj_add_event_cb(btn_back, cb_off_wake, LV_EVENT_CLICKED, NULL);
		lv_obj_remove_flag(btn_back, LV_OBJ_FLAG_SCROLLABLE);
		ui_encoder_group_add(g_group_off, btn_back);

		lv_obj_t * btn_runpause = add_encoder_top_btn(top, "启停", 100, g_group_off);
		lv_obj_add_event_cb(btn_runpause, cb_off_wake, LV_EVENT_CLICKED, NULL);
		g_off_btn_power = add_encoder_top_btn(top, "电源", 180, g_group_off);
		lv_obj_add_event_cb(g_off_btn_power, cb_off_wake, LV_EVENT_CLICKED, NULL);
		lv_obj_add_event_cb(g_off_btn_power, cb_power_long, LV_EVENT_LONG_PRESSED, NULL);

		/* 屏幕居中：系统时间 HH:MM，字号与运行页倒计时相同（125） */
		g_lbl_clock_off_center = lv_label_create(row);
		lv_label_set_text(g_lbl_clock_off_center, "10:08:00");
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

		g_group_off = lv_group_create();                   /* 各页面独立 focus 组 */
		g_group_home = lv_group_create();
		g_group_running = lv_group_create();
		g_group_end = lv_group_create();
		g_group_pay = lv_group_create();
		g_group_pay_done = lv_group_create();
		g_group_alarm = lv_group_create();
		g_group_admin = lv_group_create();
		g_group_selfcheck = lv_group_create();
		g_group_cycle = lv_group_create();
		lv_group_set_wrap(g_group_off, false);
		lv_group_set_wrap(g_group_home, false);
		lv_group_set_wrap(g_group_running, false);
		lv_group_set_wrap(g_group_end, false);
		lv_group_set_wrap(g_group_pay, false);
		lv_group_set_wrap(g_group_pay_done, false);
		lv_group_set_wrap(g_group_alarm, false);
		lv_group_set_wrap(g_group_admin, false);
		lv_group_set_wrap(g_group_selfcheck, false);
		lv_group_set_wrap(g_group_cycle, false);
		lv_group_set_edge_cb(g_group_admin, admin_group_edge_cb);
		lv_group_set_focus_cb(g_group_admin, admin_group_focus_cb);

		build_off();                                       /* 待机页（无业务 label） */
		build_home();                                      /* 主页：含全部底部文字 label */
		build_running();                                   /* 运行页：程序名/状态/阶段 label */
		build_end();                                       /* 结束页：标题/提示 label */
		build_alarm_overlay();                             /* lv_layer_top 报警弹层 */
		build_admin();                                     /* 管理员页 */
		build_selfcheck();                                 /* 工厂自检页 */
		build_cycle();                                     /* 循环程序页 */
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

/* PC 仿真：循环程序 API */
bool ui_cycle_is_active(void)
{
	return g_cycle_active;
}

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
