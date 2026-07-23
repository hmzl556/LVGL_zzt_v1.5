/**
 * @file main.c
 *
 */

#ifndef _DEFAULT_SOURCE
  #define _DEFAULT_SOURCE /* needed for usleep() */
#endif
#include <stdlib.h>
#include <stdio.h>
#ifdef _MSC_VER
  #include <Windows.h>
#else
  #include <unistd.h>
  #include <pthread.h>
#endif
#include "lvgl/lvgl.h"
#include <SDL.h>
#include "hal/hal.h"
#include "ui/ui.h"
#if USE_UI_HW
#include "alarm_fault_sim.h"
#endif

extern uint32_t a = 0;

//ZZT的代码

// 如果不使用FreeRTOS操作系统,则使用下面的main函数
#if LV_USE_OS != LV_OS_FREERTOS

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  int win_w = 1600;
  int win_h = 600;


  // 初始化lvgl库
  lv_init();

  // 错误检查:在初始化SDL窗口时进行,若失败则给出提示并终止程序
  if(sdl_hal_init(win_w, win_h) == NULL) {
    fprintf(stderr, "sdl_hal_init failed (no SDL window). Check SDL2.dll next to main.exe (same arch as exe).\n");
    return 1;
  }

  // 初始化ui界面
  ui_init();

#if USE_UI_HW
  pc_sim_alarm_fault_print_help();
#endif

  // 主循环
  while(1) {
    uint32_t sleep_time_ms = lv_timer_handler();
    ui_tick();
#if USE_UI_HW
    pc_sim_alarm_fault_poll_keys();
#endif
    if(sleep_time_ms == LV_NO_TIMER_READY){
	  sleep_time_ms =  LV_DEF_REFR_PERIOD;
    }
#ifdef _MSC_VER
    Sleep(sleep_time_ms);
#else
    usleep(sleep_time_ms * 1000);
#endif
  }

  return 0;
}

#endif
