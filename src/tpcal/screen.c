/**
 * @file tpcal/screen.c
 * @brief Front logic for the TPCAL calibration driver.
 *
 * The back logic (tpcal/tpcal) implements responding to uncalibrated touches,
 * and moving the point to the new point. When all points are inputted, the
 * front logic, via tpcal_scr_state_complete can display menu options and the
 * user can test the calibrated screen.
 * 
 * Relevant methods for implementing your own calib. mitm driver (tpcal/tpcal).
 * Resets need to be handled by your tpcal.
 * 
 */

/* INCLUDES *******************************************************************/

#include <stdio.h>
#include "screen.h"
#include "lvgl.h"
/* DEFINES ********************************************************************/


/* TYPEDEF/STRUCTURES *********************************************************/

/* STATIC PROTOTYPES **********************************************************/
static void btn_reset_onclick(lv_event_t* e);
static void btn_exit_onclick(lv_event_t* e);
/* STATIC VARIABLES  **********************************************************/

static void(*onready_cb)(void) = NULL;
static void(*onreset_cb)(void) = NULL;

static lv_obj_t* prev_scr;
static lv_obj_t* scr;

static lv_obj_t* lbl_instruct;
static lv_obj_t* btn_reset;
static lv_obj_t* btn_accept;

static lv_obj_t* pnt_calib;
static lv_anim_t anim_calx;
static lv_anim_t anim_caly;

static lv_indev_t* indev;

static lv_point_t _prev_p;
static lv_point_t initial_p;
/* MACROS *********************************************************************/

/* FUNCTIONS ******************************************************************/

// View
void tpcal_scr_create(lv_point_t* initial_pos, void(*onready)(void), void (*onreset)(void), void(*onanim)(lv_anim_t*)) {
    indev = lv_indev_get_act();
    initial_p = *initial_pos;
    onready_cb = onready;
    onreset_cb = onreset;
#ifdef DEBUG
    if (indev == NULL) printf("no registered tpcal driver? dafuq?");
    if (onready_cb == NULL) printf("no ready callback is available.");
    if (onreset_cb == NULL) printf("no reset callback available.");
#endif
    prev_scr = lv_scr_act();
    scr = lv_obj_create(NULL);
 
    static lv_obj_t* container_cnt;
    container_cnt = lv_obj_create(scr);
    lv_obj_set_align(container_cnt, LV_ALIGN_CENTER);
    lv_obj_remove_style_all(container_cnt);
    lv_obj_set_layout(container_cnt, LV_LAYOUT_FLEX);
    lv_obj_set_flex_align(container_cnt, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_flex_flow(container_cnt, LV_FLEX_FLOW_COLUMN);

    static lv_obj_t* container_btns;
    container_btns = lv_obj_create(container_cnt);
    lv_obj_remove_style_all(container_cnt);
    lv_obj_set_layout(container_cnt, LV_LAYOUT_FLEX);
    lv_obj_set_flex_align(container_cnt, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_flex_flow(container_cnt, LV_FLEX_FLOW_ROW);


    btn_reset = lv_btn_create(container_btns);
    lv_obj_add_event_cb(btn_reset, btn_reset_onclick, LV_EVENT_CLICKED, NULL);
    static lv_obj_t* lbl_btn_reset;
    lbl_btn_reset = lv_label_create(btn_reset);
    lv_label_set_text(lbl_btn_reset, TPCAL_LABEL_RESET);

    btn_accept = lv_btn_create(container_btns);
    static lv_obj_t* lbl_btn_accept;
    lbl_btn_accept = lv_label_create(btn_accept);
    lv_label_set_text(lbl_btn_accept, TPCAL_LABEL_ACCEPT);

    pnt_calib = lv_obj_create(scr);
    lv_obj_set_size(pnt_calib, TPCAL_CIRC_SIZE, TPCAL_CIRC_SIZE);
    lv_obj_set_style_radius(pnt_calib, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_pos(pnt_calib, initial_pos->x, initial_pos->y);
    lv_obj_clear_flag(pnt_calib, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(pnt_calib, LV_OBJ_FLAG_IGNORE_LAYOUT); // Don't get moved by parent!



    lbl_instruct = lv_label_create(scr);
    lv_label_set_text(lbl_instruct, TPCAL_LABEL_TAP);


    // Movement animation(s). Calls unblock_cb on animation finish, making it
    // possible to input new points.

    // Do not believe that the animation in x & y could be handled in one go.
    lv_anim_init(&anim_calx);
    lv_anim_init(&anim_caly);
    lv_anim_set_var(&anim_caly, pnt_calib);
    lv_anim_set_var(&anim_calx, pnt_calib);
    lv_anim_set_time(&anim_calx, 500);
    lv_anim_set_time(&anim_calx, 500);
    //lv_anim_set_ready_cb(&anim_calx, unblock_cb);
    lv_anim_set_exec_cb(&anim_calx, (lv_anim_exec_xcb_t)lv_obj_set_x);
    lv_anim_set_exec_cb(&anim_caly, (lv_anim_exec_xcb_t)lv_obj_set_y);
    lv_anim_set_ready_cb(&anim_calx, onanim);
}

void tpcal_scr_destroy(void) {
    lv_indev_set_cursor(indev, NULL);
    lv_obj_del(scr);
}

void tpcal_scr_state_complete(void) {
    lv_indev_set_cursor(indev, pnt_calib);
    lv_label_set_text(lbl_instruct, TPCAL_LABEL_FIN);
}

// Model Logic link

/**
 * @brief Animate calibration point to new point `p`.
 * 
 * @param p Coordinates
 */
void tpcal_set_point(lv_point_t p) {
    lv_anim_set_values(&anim_calx, _prev_p.x, p.x);
    lv_anim_set_values(&anim_caly, _prev_p.y, p.y);
    _prev_p = p;
    
    lv_anim_start(&anim_calx);
    lv_anim_start(&anim_caly);
}


/* STATIC FUNCTIONS ***********************************************************/

static void btn_reset_onclick(lv_event_t* e) {
    onreset_cb();       // Up to reset callback to destroy and reinstate.
}
static void btn_exit_onclick(lv_event_t* e) {
    tpcal_scr_destroy();
    onready_cb();
}