/**
 * PC 仿真：键盘浏览 E1–E14 故障页（不进入 ui-商用洗.c 的按键逻辑）
 */
#include "alarm_fault_sim.h"

#include <SDL.h>
#include <stdbool.h>
#include <stdio.h>

#define PC_SIM_FAULT_COUNT  15u

/* 循环程序（5.2.1）对外 API；量产 MCU 亦调用 ui_cycle_report_fault */
extern bool ui_cycle_is_active(void);
extern void ui_cycle_report_fault(uint8_t fault_code_1_based);

static uint8_t s_cycle_idx;                       /* [ ] 轮播当前下标 0=E1 */
static Uint8 s_key_prev[SDL_NUM_SCANCODES];       /* 边沿检测 */

static bool key_edge(SDL_Scancode sc, const Uint8 * st)
{
	if(sc >= SDL_NUM_SCANCODES) return false;
	bool down = st[sc] != 0;
	bool edge = down && (s_key_prev[sc] == 0);
	s_key_prev[sc] = down ? 1 : 0;
	return edge;
}

static void show_fault_idx(uint8_t idx)
{
	if(idx >= PC_SIM_FAULT_COUNT) return;
	s_cycle_idx = idx;
	/* 寿命试验激活：走循环程序故障叠层 Ex↔次数，不进全局报警页 */
	if(ui_cycle_is_active()) {
		ui_cycle_report_fault((uint8_t)(idx + 1u));
		return;
	}
	ui_pc_sim_fault_show_one(idx);
}

void pc_sim_alarm_fault_print_help(void)
{
	fprintf(stderr,
		"[PC alarm sim] F1-F12=E1-E12 | 0=E10 | -=E11 |=E12 | ,=E13 .=E14 /=E15\n"
		"               [ / ]=prev/next | Esc=clear all faults\n"
		"               (cycle prog. active: same keys -> cycle fault Ex<->count UI)\n");
}

void pc_sim_alarm_fault_poll_keys(void)
{
	SDL_PumpEvents();
	const Uint8 * st = SDL_GetKeyboardState(NULL);
	if(st == NULL) return;

	if(key_edge(SDL_SCANCODE_ESCAPE, st)) {
		ui_pc_sim_fault_clear_all();
		return;
	}

	if(key_edge(SDL_SCANCODE_LEFTBRACKET, st)) {
		show_fault_idx((uint8_t)((s_cycle_idx + PC_SIM_FAULT_COUNT - 1u) % PC_SIM_FAULT_COUNT));
		return;
	}
	if(key_edge(SDL_SCANCODE_RIGHTBRACKET, st)) {
		show_fault_idx((uint8_t)((s_cycle_idx + 1u) % PC_SIM_FAULT_COUNT));
		return;
	}

	static const SDL_Scancode digit_to_fault[] = {
		SDL_SCANCODE_1, SDL_SCANCODE_2, SDL_SCANCODE_3, SDL_SCANCODE_4,
		SDL_SCANCODE_5, SDL_SCANCODE_6, SDL_SCANCODE_7, SDL_SCANCODE_8,
		SDL_SCANCODE_9,
	};
	for(uint8_t i = 0; i < 9u; i++) {
		if(key_edge(digit_to_fault[i], st)) {
			show_fault_idx(i);
			return;
		}
	}

	if(key_edge(SDL_SCANCODE_0, st)) { show_fault_idx(9u); return; }
	if(key_edge(SDL_SCANCODE_MINUS, st)) { show_fault_idx(10u); return; }
	if(key_edge(SDL_SCANCODE_EQUALS, st)) { show_fault_idx(11u); return; }
	if(key_edge(SDL_SCANCODE_COMMA, st)) { show_fault_idx(12u); return; }
	if(key_edge(SDL_SCANCODE_PERIOD, st)) { show_fault_idx(13u); return; }
	if(key_edge(SDL_SCANCODE_SLASH, st)) { show_fault_idx(14u); return; }

	static const SDL_Scancode fkey_to_fault[] = {
		SDL_SCANCODE_F1, SDL_SCANCODE_F2, SDL_SCANCODE_F3, SDL_SCANCODE_F4,
		SDL_SCANCODE_F5, SDL_SCANCODE_F6, SDL_SCANCODE_F7, SDL_SCANCODE_F8,
		SDL_SCANCODE_F9, SDL_SCANCODE_F10, SDL_SCANCODE_F11, SDL_SCANCODE_F12,
	};
	for(uint8_t i = 0; i < 12u; i++) {
		if(key_edge(fkey_to_fault[i], st)) {
			show_fault_idx(i);
			return;
		}
	}
}
