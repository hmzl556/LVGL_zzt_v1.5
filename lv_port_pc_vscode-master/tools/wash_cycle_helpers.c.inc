/* ========== 循环程序（商用洗适配：复用程序设置字段，无金额） ========== */

static void cycle_run_count_label_sync(void);  /* 定义见下方 extracted 块 */

static ui_program_admin_t g_cycle_cfg[TOTAL_PROGRAMS];
static cycle_session_t g_cycle_session;
static bool g_cycle_active;
static cycle_ui_state_t g_cycle_ui_state;
static lv_timer_t * g_cycle_fault_blink_timer;
static cycle_fault_phase_t g_cycle_fault_phase;
static bool g_cycle_fault_on_running;
static bool g_cycle_prog_ui_loading;
static int32_t g_cycle_prog_sel;
static lv_obj_t * g_cycle_prog_btns[TOTAL_PROGRAMS];
static lv_obj_t * g_cycle_lbl_run_count;

static void cycle_wash_sync_prog_pick_ui(void)
{
	for(int i = 0; i < TOTAL_PROGRAMS; i++) {
		if(g_cycle_prog_btns[i] == NULL) continue;
		if(i == g_cycle_prog_sel) {
			lv_obj_add_state(g_cycle_prog_btns[i], LV_STATE_CHECKED);
		} else {
			lv_obj_remove_state(g_cycle_prog_btns[i], LV_STATE_CHECKED);
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

static void cycle_kb_close(void) { /* 商用洗循环页无独立键盘 */ }

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
