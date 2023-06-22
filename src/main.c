/**
 * @file main.c
 * @brief LVGL Demo ILI9341.
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include "ili9341.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "perf_counter.h"
#include "pico/stdio.h"
#include "demos/lv_demos.h"


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
#ifdef DEBUG
    stdio_init_all();
#endif
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
#ifdef DEBUG 
    printf("hello world!");
#endif
    lv_init();
    
    lv_ili9341_init();
    //lv_hello_lvgl();
    lv_demo_widgets();
    while(1) {
        lv_timer_handler();
    }

    return 0;
}

