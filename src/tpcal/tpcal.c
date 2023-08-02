/**
 * @file tpcal.c
 * @brief Back logic for the TPCAL calibration driver.
 * 
 * Accepts calibration parameters from a variety of sources.
 *
 * TODO: NVS (Non-Volatile Storage) the program "reserves" a sector of flash
 * that can have the parameters written to it.
 *  IDEAS: this should be through some mechanism that can "share" a sector, as
 *  we have to essentially take a 4 kb sector to do this effectively.
 *
 * Front logic is implemented via tpcal/screen and can be used to calibrate
 * the screen at any point. 
 *
 * REFERENCES:
 * https://www.analog.com/media/en/technical-documentation/tech-articles/an-easytounderstand-explanation-of-calibration-in-touchscreen-systems.pdf
 * https://www.ti.com/lit/an/slyt277/slyt277.pdf
 * 
 */

// NOTE: Remember to add your source to your project file!

/* INCLUDES *******************************************************************/
#include "tpcal.h"

#include <stdio.h>
#include "pico/platform.h"
#include "screen.h"
#include "lvgl.h"
/* DEFINES ********************************************************************/

// LVGL position of each calibration point 

#ifndef TPCAL_POS_X1
#define TPCAL_POS_X1 1
#endif
#ifndef TPCAL_POS_Y1
#define TPCAL_POS_Y1 1
#endif
#ifndef TPCAL_POS_X2
#define TPCAL_POS_X2 1
#endif
#ifndef TPCAL_POS_Y2
#define TPCAL_POS_Y2 1
#endif
#ifndef TPCAL_POS_X3
#define TPCAL_POS_X3 1
#endif
#ifndef TPCAL_POS_Y3
#define TPCAL_POS_Y3 1
#endif

#define TPCAL_PNT_XY1 (lv_point_t) {.x = TPCAL_POS_X1, .y = TPCAL_POS_Y1}
#define TPCAL_PNT_XY2 (lv_point_t) {.x = TPCAL_POS_X2, .y = TPCAL_POS_Y2}
#define TPCAL_PNT_XY3 (lv_point_t) {.x = TPCAL_POS_X3, .y = TPCAL_POS_Y3}


/* TYPEDEF/STRUCTURES *********************************************************/


/** internal coefficients for translation */
struct _tpcal_trans_cfg_t {
    int32_t ax;
    int32_t bx;
    int32_t dx;
    int32_t ay;
    int32_t by;
    int32_t dy;
    bool calibrated;
};

/** internal state tracking - tpcal calib should be only looked for in
  * a calibration state
  */
enum _tpcal_state_t {
    TPCAL_CALIB,
    TPCAL_READY,
    TPCAL_INIT,
};

/* STATIC PROTOTYPES **********************************************************/
static void lv_tpcal_read_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
static void trans_coef_calc(lv_point_t p_touch[]);
static void tpcal_roll_avg(int16_t* x, int16_t* y, uint8_t* n);
static void calib_rst(void);
static void anim_unblock(lv_anim_t* a);
/* STATIC VARIABLES  **********************************************************/
static enum _tpcal_state_t state = TPCAL_INIT;
static struct _tpcal_trans_cfg_t coef = {.calibrated = false};
static void(*cb_onready)(void) = NULL;

static uint8_t c_n_p = 0;
static lv_indev_state_t c_prev_state;
/* MACROS *********************************************************************/

/* FUNCTIONS ******************************************************************/

/**
 * @brief sets up callback for tpcal use and register driver
 * 
 * @param indev driver to register
 */
void tpcal_register(lv_indev_drv_t *indev) {
    state = TPCAL_INIT;
    indev->user_data = indev->read_cb;
    indev->read_cb = lv_tpcal_read_cb;
}


void tpcal_nvs_save(void (*cb)(void)) {
    panic("tpcal_nvs_save not impl.");
}
void tpcal_nvs_load(void (*cb)(void)) {
    panic("tpcal_nvs_load not impl.!");
}

void tpcal_calib(void (*cb)(void)) {
    state = TPCAL_CALIB;
    cb_onready = cb;
    tpcal_scr_create(&TPCAL_PNT_XY1, cb_onready, calib_rst, anim_unblock);
}


/**
 * @brief Applies translation co-efficients
 * 
 * @param x Uncorrected display response to fix (in established x dir)
 * @param y Uncorrected display response to fix (in established y dir)
 */
void tpcal_translate(int16_t *x, int16_t *y) {
    if(state != TPCAL_READY) return;
    int16_t cx,cy;
    cx = *x; cy = *y;

    *x = (coef.ax*cx) + (coef.bx*cy) + coef.dx;
    *y = (coef.ay*cx) + (coef.by*cy) + coef.dy;
}
/* STATIC FUNCTIONS ***********************************************************/

static void calib_rst(void) {
    state = TPCAL_CALIB;
    c_n_p = 0;                              // ensure we are not in a state that
    c_prev_state = LV_INDEV_STATE_RELEASED; // can trigger a "press" register
    tpcal_scr_destroy();
    tpcal_scr_create(&TPCAL_PNT_XY1, cb_onready, calib_rst, anim_unblock);
}
static volatile bool allow_input = false;
static void anim_unblock(lv_anim_t* a) {
    allow_input = true;
}

/**
 * @brief "Entry point" for tpcal into lvgl. all touch reads will go through
 * here. 
 * 
 * @param indev 
 * @param data 
 */
static void lv_tpcal_read_cb(lv_indev_drv_t* indev, lv_indev_data_t* data) {
#ifdef DEBUG
    if (!indev->user_data) printf("read cb not set :(");
#endif
    if (!allow_input) return;

    // Execute driver callback internally. This is not "seen" by lvgl but makes
    // integration easier.
    ((void (*)(lv_indev_drv_t*, lv_indev_data_t*))indev->user_data)(indev,
                                                                    data);

    // containers for touch and display points
    static lv_point_t p_touch[3];
    static uint8_t n = 0;
    static lv_point_t p_disp[] = {TPCAL_PNT_XY1, TPCAL_PNT_XY2, TPCAL_PNT_XY3};

    switch (state) {
      case TPCAL_CALIB:;
        while (data->state == LV_INDEV_STATE_PRESSED) {
          // Collect another display sample.
          ((void (*)(lv_indev_drv_t*, lv_indev_data_t*))indev->user_data)(indev,
                                                                          data);
          tpcal_roll_avg(&(data->point.x), &(data->point.y), &n);
        }
        n = 0;
        if (c_prev_state == LV_INDEV_STATE_PRESSED) {
          // Submit on falling edge
          p_touch[c_n_p] = (lv_point_t){.x = data->point.x, .y = data->point.y};
          tpcal_set_point(p_disp[c_n_p]);
          c_n_p++;
          if (c_n_p == 2) {  // all 3 points recorded.
            c_n_p = 0;
            trans_coef_calc(p_touch);
            tpcal_scr_state_complete();  // display ui elements  tele
            state = TPCAL_READY;
            allow_input = true;
          }
        }
        c_prev_state = data->state;

        break;
      case TPCAL_INIT:
        printf("tpcal is in unsafe state! (tpcal_init)");
        break;
      case TPCAL_READY:
        // normal processing
        if (data->state == LV_INDEV_STATE_PRESSED) {
          tpcal_translate(&(data->point.x), &(data->point.y));
          tpcal_roll_avg(&(data->point.x), &(data->point.y), &n);
        }
        n = 0;
        break;
      default:
        printf("TPCAL Driver Unknown State");
        break;
    }
}

static void tpcal_roll_avg(int16_t* x, int16_t* y, uint8_t* n) {
    static int16_t buf_x[10], buf_y[10];

    for (int i = 9; i > 0; i--) {  // Shift results forward one
        buf_x[i] = buf_x[i - 1];
        buf_y[i] = buf_y[i - 1];
    }
    // rolling average is of the last n points up to 10 points.
    if ((*n) < 10) (*n)++;

    buf_x[0] = *x;
    buf_y[0] = *y;

    uint32_t s_x, s_y;
    for (int i = 0; i <= (*n); i++) {
        s_x += buf_x[i];
        s_y += buf_y[i];
    }

    // note that cases like n=0 should not be directly caused by this function
    *x = s_x / (*n);
    *y = s_y / (*n);
}

// TODO: remove 2nd param.

static void trans_coef_calc(lv_point_t p_touch[]) {
    uint32_t determinant =  // det(A) A is 3x3 inptu matrix (z=1 all)
        (p_touch[0].x - p_touch[2].x) * (p_touch[1].y - p_touch[2].y) -
        (p_touch[1].x - p_touch[2].x) * (p_touch[0].y - p_touch[2].y);
    // Refer to "Definitions for Equation 8". The algorithm has support for
    // 3 point calibration.

    // a,b,d refer to their similar symbols (alpha, beta, delta).
    
    // Translation co-efficients correct for rotation, scaling and offset.
    // The co-efficients can also be loaded from NVS (TODO.)
 
    coef = (struct _tpcal_trans_cfg_t){
        .ax = ((TPCAL_POS_X1 - TPCAL_POS_X3) * (p_touch[1].y - p_touch[2].y) -
               (TPCAL_POS_X1 - TPCAL_POS_X3) * (p_touch[0].y - p_touch[2].y)) /
              determinant,

        .bx = ((p_touch[0].x - p_touch[2].x) * (TPCAL_POS_X1 - TPCAL_POS_X3) -
               (p_touch[1].x - p_touch[2].x) * (TPCAL_POS_X1 - TPCAL_POS_X3)) /
              determinant,

        .dx = ((TPCAL_POS_X1) * ((p_touch[1].x * p_touch[2].y) -
                                 (p_touch[2].x * p_touch[1].y)) -
               (TPCAL_POS_X1) * ((p_touch[0].x * p_touch[2].y) -
                                 (p_touch[2].x * p_touch[0].y)) +
               (TPCAL_POS_X3) * ((p_touch[0].x * p_touch[1].y) -
                                 (p_touch[1].x * p_touch[0].y))) /
              determinant,

        .ay = ((TPCAL_POS_Y1 - TPCAL_POS_Y3) * (p_touch[1].y - p_touch[2].y) -
               (TPCAL_POS_Y2 - TPCAL_POS_Y3) * (p_touch[0].y - p_touch[2].y)) /
              determinant,

        .by = ((p_touch[0].x - p_touch[2].x) * (TPCAL_POS_Y2 - TPCAL_POS_Y3) -
               (p_touch[1].x - p_touch[2].x) * (TPCAL_POS_Y1 - TPCAL_POS_Y3)) /
              determinant,

        .dy = ((TPCAL_POS_Y1) * ((p_touch[1].x * p_touch[2].y) -
                                 (p_touch[2].x * p_touch[1].y)) -
               (TPCAL_POS_Y2) * ((p_touch[0].x * p_touch[2].y) -
                                 (p_touch[2].x * p_touch[0].y)) +
               (TPCAL_POS_Y3) * ((p_touch[0].x * p_touch[1].y) -
                                 (p_touch[1].x * p_touch[0].y))) /
              determinant,

        .calibrated = true
    };
}