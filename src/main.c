/**
 * @file main.c
 * @brief LVGL Demo ILI9341.
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/**
 * @file {File Name}
 * @brief If required.
 * 
 * Detailed info if required.
 * @copyright Copyright (c) 2023
 * 
 */

// NOTE: Remember to add your source to your project file!

/* INCLUDES *******************************************************************/

#include "boards/pico.h"
#include "display/ili9341.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "perf_counter.h"
#include "pico/stdio.h"
#include "demos/lv_demos.h"
#include "display/tpcal.h"
#include "pico/time.h"

/* DEFINES ********************************************************************/

/* TYPEDEF/STRUCTURES *********************************************************/

/* STATIC PROTOTYPES **********************************************************/

static void system_init(void);

/* STATIC VARIABLES  **********************************************************/

/* MACROS *********************************************************************/

/* FUNCTIONS ******************************************************************/

void SysTick_Handler(void)
{

}

/**
 * @brief 
 * 
 */
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
    bus_init();
/*
    while(1) {
        xpt2046_ispressed();
        uint16_t x,y;
        xpt2046_xyz(&x, &y);
        sleep_ms(1);
    }

    return 0;
    */
    lv_disp_init();
    ili9341_cmd_p(ILI9341_W_BRIGHT, 255);
    //tpcal_create((tpcal_cb_t)xpt_tpcal, lv_demo_widgets);
    //lv_hello_lvgl();
    lv_demo_widgets();
    while(1) {
        lv_timer_handler();
    }

    return 0;
}

/* STATIC FUNCTIONS ***********************************************************/

/** Sets up cortex SysTick. Required for using perf_counter to port LVGL. */
static void system_init(void)
{
    // SysTick.
    extern void SystemCoreClockUpdate();
    SystemCoreClockUpdate();
    init_cycle_counter(false);

#ifdef PICO_DEFAULT_LED_PIN
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT); // LED pin pico board.
    // redefine if alternative board with alternative pin (or undefine if none)
#endif
#ifdef DEBUG
    stdio_init_all();       // Debug on UART or USB. Not relevant to SWD debug.
#endif
}