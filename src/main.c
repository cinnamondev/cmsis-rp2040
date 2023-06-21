#include <stdio.h>
#include "RP2040.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "lvgl.h"
#include "ili9341.h"
#include "pico/time.h"
#include "perf_counter.h"
#include "src/core/lv_disp.h"
#include "src/core/lv_obj.h"
#include "src/core/lv_obj_pos.h"
#include "src/font/lv_symbol_def.h"
#include "src/hal/lv_hal_disp.h"
#include "src/lv_api_map.h"
#include "src/misc/lv_timer.h"

void SysTick_Handler(void)
{

}

static void system_init(void)
{
    extern void SystemCoreClockUpdate();

    SystemCoreClockUpdate();
    /*! \note if you do want to use SysTick in your application, please use 
     *!       init_cycle_counter(true); 
     *!       instead of 
     *!       init_cycle_counter(false); 
     */
    init_cycle_counter(false);
    gpio_set_dir(25, GPIO_OUT);
}


void lv_hello_lvgl(void)
{
    // Create simple label centered on screen
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, LV_SYMBOL_OK "hello lvgl!");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

int main() {
    system_init();
    lv_init();
    
    lv_ili9341_init();
    lv_hello_lvgl();

    while(1) {
        lv_timer_handler();
    }

    return 0;
}

