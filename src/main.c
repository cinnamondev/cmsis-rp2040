#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "lvgl.h"
#include "lv_drv_ili9341.h"
#include "pico/time.h"
#include "perf_counter.h"
#include "src/core/lv_disp.h"
#include "src/core/lv_obj.h"
#include "src/core/lv_obj_pos.h"
#include "src/lv_api_map.h"
#include "src/misc/lv_timer.h"

static lv_disp_drv_t LV_DRIVER;
static lv_disp_draw_buf_t LV_BUFFER;
static lv_color_t lv_buf_1[320*240];
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
    init_cycle_counter(true);

#if defined(RTE_Compiler_EventRecorder) && defined(USE_EVR_FOR_STDOUR)
    EventRecorderInitialize(0, 1);
#endif

}


void lv_example_label_1(void)
{
    // Create simple label centered on screen
    lv_obj_t *parent= lv_obj_create(lv_scr_act());
    lv_obj_set_size(parent, 320, 240);
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, "Hello LVGL!");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
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

    ili9341_init();

    lv_disp_draw_buf_init(&LV_BUFFER, 
        &lv_buf_1,
        NULL,
        320*240
    );
    lv_disp_drv_init(&LV_DRIVER);
    LV_DRIVER.hor_res = 320;
    LV_DRIVER.ver_res = 240;
    LV_DRIVER.flush_cb = ili9341_flush;
    LV_DRIVER.draw_buf = &LV_BUFFER;

    lv_disp_t* display;
    display = lv_disp_drv_register(&LV_DRIVER);
    if (display == NULL) {
        int i = 0;
        while (i<10) {
            //printf("oi oi lass");
            gpio_put(LED_PIN, 1);
            sleep_ms(250);
            gpio_put(LED_PIN, 0);
            sleep_ms(250);
            i++;
        }
    }
    lv_example_label_1();

    while(1) {
        lv_task_handler();
        sleep_ms(5);

    }
    
    return 0;
}

