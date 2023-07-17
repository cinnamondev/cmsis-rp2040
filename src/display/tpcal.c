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
#include "tpcal.h"
#include "lvgl.h"
#include "src/core/lv_obj.h"
#include "src/core/lv_obj_pos.h"
#include "src/misc/lv_area.h"
#include "src/widgets/lv_label.h"
#include "stdio.h"

/* DEFINES ********************************************************************/
#define CIRCLE_SIZE      20
#define TP_MAX_VALUE     5000
/* TYPEDEF/STRUCTURES *********************************************************/

enum tpcal_state_t {
    TPCAL_WAIT_X1,
    TPCAL_WAIT_X2,
    TPCAL_WAIT_X3,
    TPCAL_INIT,
    TPCAL_DONE,
    TPCAL_DONE_WAIT
};

/* STATIC PROTOTYPES **********************************************************/

static inline void unblock_clicks(void);
static inline void block_clicks(void);
static void unblock_cb(lv_anim_t* a_e);
static void btn_click_action(lv_event_t* e);
static void update_pointer_pos(lv_coord_t x, lv_coord_t y);
static void rst_btn_onclick(lv_event_t* e);
static void exit_btn_onclick(lv_event_t* e);
/* STATIC VARIABLES  **********************************************************/

//static lv_point_t p[3]; /*Calibration points: p[0]: top-left; p[1]: top-right, p[2]: bottom-right, p[3]: bottom-left */
static enum tpcal_state_t state;
static lv_obj_t *prev_scr;
static lv_obj_t* scr_inner;
static lv_obj_t *scr_btn;
static lv_obj_t *label_dir;
static lv_obj_t *cal_point;

static tpcal_cb_t tpcal_cb = NULL;
static void (*fin_cb)(void) = NULL;

static lv_anim_t anim_calx;
static lv_anim_t anim_caly;

static lv_obj_t *scr;

/* MACROS *********************************************************************/

/* FUNCTIONS ******************************************************************/

/**
 * Create a touch pad calibration screen
 */
void tpcal_create(tpcal_cb_t recorder_cb, void (*finish_cb)(void)) {
    fin_cb = finish_cb;
    tpcal_cb = recorder_cb;
    lv_indev_t* touch = lv_indev_get_act(); // access to active touch
    state = TPCAL_INIT;

    // Transparent button (capture any input)
    static lv_style_t styl_scr_btn;
    lv_style_init(&styl_scr_btn);
    lv_style_set_bg_opa(&styl_scr_btn, LV_OPA_MAX);
    lv_style_set_opa(&styl_scr_btn, LV_OPA_MAX);    

    lv_obj_t* prev_scr = lv_scr_act();

    scr = lv_obj_create(prev_scr);
    lv_obj_set_size(scr, TP_MAX_VALUE, TP_MAX_VALUE);
    lv_scr_load(scr);

    scr_inner = lv_obj_create(scr);
    lv_obj_set_size(scr_inner, 320, 240);
    lv_obj_set_align(scr_inner, LV_ALIGN_CENTER);

    label_dir = lv_label_create(scr_inner);
    lv_label_set_text(label_dir, "Click the O precisely in the centre!\n");
    lv_obj_set_align(label_dir, LV_ALIGN_CENTER);

    cal_point = lv_obj_create(scr_inner);
    lv_obj_set_size(cal_point, CIRCLE_SIZE, CIRCLE_SIZE);
    lv_obj_set_style_radius(cal_point, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_pos(cal_point, TPCAL_POS_X1, TPCAL_POS_Y1);

    // Screen sized button
    scr_btn = lv_btn_create(scr);
    lv_obj_set_size(scr_btn, TP_MAX_VALUE, TP_MAX_VALUE);
    lv_obj_remove_style_all(scr_btn);
    lv_obj_add_style(scr_btn, &styl_scr_btn, 0);
    lv_obj_add_style(scr_btn, &styl_scr_btn, LV_STATE_PRESSED);
    lv_obj_add_event_cb(scr_btn, btn_click_action, LV_EVENT_CLICKED, NULL);

    // Movement animation(s). Calls unblock_cb on animation finish, making it
    // possible to input new points.
    lv_anim_init(&anim_calx);
    lv_anim_init(&anim_caly);
    lv_anim_set_var(&anim_caly, cal_point);
    lv_anim_set_var(&anim_calx, cal_point);
    lv_anim_set_time(&anim_calx, 200);
    lv_anim_set_time(&anim_calx, 200);
    lv_anim_set_ready_cb(&anim_calx, unblock_cb);
    lv_anim_set_exec_cb(&anim_calx, (lv_anim_exec_xcb_t)lv_obj_set_x);
    lv_anim_set_exec_cb(&anim_caly, (lv_anim_exec_xcb_t)lv_obj_set_y);

    state = TPCAL_WAIT_X1;
}


void tpcal_destroy(void) {
    lv_obj_del(scr);    // destroy screen & children (tpcal)
    lv_indev_set_cursor(lv_indev_get_act(), NULL); // ensure no cursor.
}


/* STATIC FUNCTIONS ***********************************************************/

static volatile bool accept_click = false;

static inline void unblock_clicks(void) {
    accept_click = true;
}

static inline void block_clicks(void) {
    accept_click = false;
}

static void unblock_cb(lv_anim_t* a_e) {
    accept_click = false;
}
static void btn_click_action(lv_event_t* e) {
    if (!accept_click) return;  // Do not modify - do not update state.

    block_clicks();     // must be released by animation cb first.
    static struct point_t touch[3];
    static struct point_t display[3];
#ifdef DEBUG
    if(tpcal_cb == NULL) printf("tpcal cb improperly set-prepare for unforseen consequences");
#endif
    lv_indev_t* indev = lv_indev_get_act();
    switch (state) {
        TPCAL_WAIT_X1:
#ifdef DEBUG
            printf("\n btnclick state x1 \n");
#endif
            // type punning is ok -equivalentish members.
            lv_indev_get_point(indev, (lv_point_t*)&touch[0]);
            state = TPCAL_WAIT_X2;
            update_pointer_pos(TPCAL_POS_X2, TPCAL_POS_Y2);
            break;
        TPCAL_WAIT_X2:
#ifdef DEBUG
            printf("\n btnclick state x2 \n");
#endif
            // record x2, get to next state
            lv_indev_get_point(indev, (lv_point_t*)&touch[1]);
            state = TPCAL_WAIT_X3;
            update_pointer_pos(TPCAL_POS_X3, TPCAL_POS_Y3);
            break;

        TPCAL_WAIT_X3:
#ifdef DEBUG
            printf("\n btnclick state x3 \n");
#endif
            lv_indev_get_point(indev, (lv_point_t*)&touch[2]);
            lv_obj_clear_flag(cal_point, LV_OBJ_FLAG_CLICKABLE);
            lv_indev_set_cursor(indev, cal_point); // drag demo
            lv_obj_del(scr_btn); // Calibration is finished!

            lv_label_set_text(label_dir, "Calibration complete!");
            lv_obj_set_align(label_dir, LV_ALIGN_TOP_MID);
            
            lv_obj_t* container = lv_obj_create(scr_inner);
            lv_obj_center(container);
            lv_obj_set_align(container, LV_ALIGN_CENTER);
            lv_obj_set_flex_flow(container, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

            lv_obj_t* exit_btn = lv_btn_create(container);
            lv_obj_t* label_exit = lv_label_create(exit_btn);
            lv_label_set_text(label_exit, "Exit TPCAL");
            lv_obj_add_event_cb(exit_btn, exit_btn_onclick, LV_EVENT_CLICKED, NULL);

            lv_obj_t* rst_btn = lv_btn_create(container);
            lv_obj_t* label_rst = lv_label_create(rst_btn);
            lv_label_set_text(label_rst, "Recalibrate?");
            lv_obj_add_event_cb(rst_btn, rst_btn_onclick, LV_EVENT_CLICKED, NULL);
            state = TPCAL_DONE;
            tpcal_cb(touch,display);
            break;
        default:
#ifdef DEBUG
            printf("\n btnclick took no action \n");
#endif
            break;
    
    }
}

static void rst_btn_onclick(lv_event_t* e) {
    tpcal_destroy();
    tpcal_create(tpcal_cb, fin_cb);
}

static void exit_btn_onclick(lv_event_t* e) {    
    tpcal_destroy();
    if (fin_cb != NULL) fin_cb();
}

static void update_pointer_pos(lv_coord_t x, lv_coord_t y)  {
    static lv_coord_t prev_x = TPCAL_POS_X1;
    static lv_coord_t prev_y = TPCAL_POS_Y1;
    lv_anim_set_values(&anim_calx, prev_x, x);
    lv_anim_set_values(&anim_caly, prev_y, y);
    prev_x = x;
    prev_y = y;
}