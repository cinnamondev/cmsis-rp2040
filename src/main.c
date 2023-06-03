#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "lvgl.h"
#include "ili9341.h"
#include "pico/time.h"
#include "perf_counter.h"
#include "src/misc/lv_timer.h"

static lv_disp_drv_t disp_drv;
//https://github.com/lvgl/lvgl/blob/release/v8.3/examples/widgets/label/lv_example_label_1.c

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

#if defined(RTE_Compiler_EventRecorder) && defined(USE_EVR_FOR_STDOUR)
    EventRecorderInitialize(0, 1);
#endif

}


void lv_example_label_1(void)
{

    lv_obj_t * label1 = lv_label_create(lv_scr_act());
    lv_label_set_long_mode(label1, LV_LABEL_LONG_WRAP);     //Break the long lines
    lv_label_set_recolor(label1, true);                      //Enable re-coloring by commands in the text
    lv_label_set_text(label1, "#0000ff Re-color# #ff00ff words# #ff0000 of a# label, align the lines to the center "
                              "and wrap long text automatically.");
    lv_obj_set_width(label1, 150);  //Set smaller width to make the lines wrap
    lv_obj_set_style_text_align(label1, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label1, LV_ALIGN_CENTER, 0, -40);

    lv_obj_t * label2 = lv_label_create(lv_scr_act());
    lv_label_set_long_mode(label2, LV_LABEL_LONG_SCROLL_CIRCULAR);     //Circular scroll
    lv_obj_set_width(label2, 150);
    lv_label_set_text(label2, "It is a circularly scrolling text. ");
    lv_obj_align(label2, LV_ALIGN_CENTER, 0, 40);
}

int main() {
    //stdio_init_all();
    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    int i = 0;
    while (i<10) {
        //printf("oi oi lass");
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
        i++;

    }

    system_init();
    lv_init();
    lv_ili9341_init(&disp_drv);

    lv_disp_t* display;
    display = lv_disp_drv_register(&disp_drv);
    
    lv_example_label_1();

    while(1) {
        lv_timer_handler_run_in_period(5);
    }
    
    return 0;
}

