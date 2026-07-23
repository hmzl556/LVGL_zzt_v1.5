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
		lv_obj_t * top = create_top_bar(root, &g_lbl_clock_running, g_scr_home, g_group_running,
			&g_running_btn_back);
		g_running_btn_runpause = add_encoder_top_btn(top, "启停", 100, g_group_running);
		lv_obj_add_event_cb(g_running_btn_runpause, cb_running_runpause, LV_EVENT_CLICKED, NULL);
		lv_obj_add_event_cb(g_running_btn_runpause, cb_runpause, LV_EVENT_CLICKED, NULL);
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

		/* 倒计时 label：数字 M:SS，由 running_countdown_update_label 更新，不参与语言表 */
		lv_obj_t * time_txt = lv_label_create(mid);
		g_running_time_label = time_txt;
		lv_label_set_text(time_txt, "0:24");                 /* 占位初值，进入运行页后按程序时长重设 */
		lv_obj_set_style_text_color(time_txt, lv_color_hex(COL_TEXT), LV_PART_MAIN);
		ui_set_obj_font(time_txt, s_font_sc_125);

		/* 童锁圆形按钮 + 内部文字「童锁」（固定中文，不翻译） */
		lv_obj_t * child_lock = lv_button_create(root);
		g_running_child_lock_btn = child_lock;
		lv_obj_set_size(child_lock, 52, 52);
		lv_obj_set_style_radius(child_lock, LV_RADIUS_CIRCLE, LV_PART_MAIN);
		lv_obj_set_style_pad_all(child_lock, 0, LV_PART_MAIN);
		lv_obj_set_style_shadow_width(child_lock, 0, LV_PART_MAIN);
		lv_obj_set_style_bg_opa(child_lock, LV_OPA_COVER, LV_PART_MAIN);
		lv_obj_set_style_bg_color(child_lock, lv_color_hex(COL_CHILD_LOCK_WHITE), LV_PART_MAIN);
		lv_obj_set_style_border_width(child_lock, 2, LV_PART_MAIN);
		lv_obj_set_style_border_color(child_lock, lv_color_hex(0xBBBBBB), LV_PART_MAIN);
		lv_obj_remove_flag(child_lock, LV_OBJ_FLAG_SCROLLABLE);
		lv_obj_add_event_cb(child_lock, cb_running_child_lock_long, LV_EVENT_LONG_PRESSED, NULL);
		lv_obj_add_event_cb(child_lock, cb_running_child_lock_released, LV_EVENT_RELEASED, NULL);

		lv_obj_t * lock_lbl = lv_label_create(child_lock);
		g_running_child_lock_lbl = lock_lbl;
		lv_label_set_text(lock_lbl, "童锁");                 /* 固定中文，不参与 ui_translation */
		lv_label_set_long_mode(lock_lbl, LV_LABEL_LONG_CLIP);
		lv_obj_set_style_text_color(lock_lbl, lv_color_hex(0x222222), LV_PART_MAIN);
		lv_obj_set_style_text_align(lock_lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
		ui_set_obj_font(lock_lbl, s_font_sc_20);
		lv_obj_center(lock_lbl);
		ui_encoder_group_add(g_group_running, child_lock);

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
		lv_obj_set_style_bg_opa(g_running_lock_blocker, LV_OPA_50, LV_PART_MAIN);
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
	/* wash: no add_time */
}

// 将快照 cfg 写入运行会话（跳过支付/追加页）
static void cycle_session_apply_to_running(void)
{
	g_wheel_sel = g_cycle_session.prog_idx;
	g_prog_cfg[g_wheel_sel] = g_cycle_session.cfg;
	program_admin_apply_one(g_wheel_sel);
}

// 锁定当前 UI 参数到 session 快照（后续每轮复用）
static void cycle_session_snapshot_from_ui(void)
{
	g_cycle_session.prog_idx = g_cycle_prog_sel;
	g_cycle_session.cfg = g_cycle_cfg[g_cycle_prog_sel];
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

	/* 故障叠层：全屏半透明 + 居中大字 Ex↔次数（设定页；运行中故障用运行页叠层） */
	cycle_fault_overlay_create(root, &g_cycle_panel_fault, &g_cycle_lbl_fault_alt);

	cycle_encoder_group_build();
}

