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

#include "lvgl.h"
#include "screen.h"
#include "src/core/lv_disp.h"
#include "src/core/lv_event.h"
#include "src/core/lv_indev.h"
#include "src/core/lv_obj.h"
#include "src/core/lv_obj_pos.h"
#include "src/misc/lv_anim.h"
#include "src/misc/lv_area.h"
/* DEFINES ********************************************************************/

/* TYPEDEF/STRUCTURES *********************************************************/

/* STATIC PROTOTYPES **********************************************************/

static void ondestroy(lv_event_t *e);
static void btn_exit_click(lv_event_t *e);
static void btn_reset_click(lv_event_t *e);


/* STATIC VARIABLES  **********************************************************/

static void (*cb_destroy)(void) = NULL;
static void (*cb_reset)(void) = NULL;

static lv_obj_t* scr;
static lv_obj_t* label_instruct;
static lv_obj_t* container_panel;
static lv_obj_t* btn_exit;
static lv_obj_t* btn_reset;
static lv_obj_t* label_btn_exit;
static lv_obj_t* label_btn_reset;
static lv_obj_t* obj_point;

static lv_anim_t anim_px;
static lv_anim_t anim_py;

/* MACROS *********************************************************************/

/* FUNCTIONS ******************************************************************/

/**
 * @brief Create the calibration screen.
 * 
 * @param cb_ondestroy On screen destroy callback. This can be used for a
 * startup application.
 * @param cb_onreset On reset callback. Screen will reset itself- this is for
 * the tpcal driver to reset itself.
 * @param cb_onanim On position change (following calibration point animation).
 * Use to unblock input following animation.
 * @param initial_point Initial point to place point at.
 */
void tpcal_scr_init(void (*cb_ondestroy)(void),
                    void (*cb_onreset)(void),
                    void (*cb_onposition)(lv_anim_t*),
                    const lv_point_t* initial_point) {
  cb_destroy = cb_ondestroy;  // wrapper by `onexit` function. Not executed on NULL.
  scr = lv_obj_create(NULL);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);  /// Flags
  lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN_WRAP);
  lv_obj_set_flex_align(scr, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_add_event_cb(scr, ondestroy, LV_EVENT_SCREEN_UNLOADED, &scr);

  label_instruct = lv_label_create(scr);
  lv_obj_set_width(label_instruct, 256);
  lv_obj_set_height(label_instruct, LV_SIZE_CONTENT);
  lv_obj_set_align(label_instruct, LV_ALIGN_CENTER);
  lv_label_set_text(label_instruct,
                    TPCAL_LABEL_TAP);
  lv_obj_set_style_text_align(label_instruct, LV_TEXT_ALIGN_CENTER,
                              LV_PART_MAIN | LV_STATE_DEFAULT);

  container_panel = lv_obj_create(scr);
  lv_obj_set_width(container_panel, LV_SIZE_CONTENT);   /// 244
  lv_obj_set_height(container_panel, LV_SIZE_CONTENT);  /// 79
  lv_obj_set_align(container_panel, LV_ALIGN_CENTER);
  lv_obj_set_flex_flow(container_panel, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(container_panel, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
  lv_obj_clear_flag(container_panel, LV_OBJ_FLAG_SCROLLABLE);  /// Flags
  lv_obj_add_flag(container_panel,      // Hidden until tpcal_scr_show_nav
                  LV_OBJ_FLAG_HIDDEN);  // or flag otherwise cleared.

  btn_exit = lv_btn_create(container_panel);
  lv_obj_set_width(btn_exit, 100);
  lv_obj_set_height(btn_exit, 50);
  lv_obj_set_align(btn_exit, LV_ALIGN_CENTER);
  lv_obj_add_flag(btn_exit, LV_OBJ_FLAG_SCROLL_ON_FOCUS);  /// Flags
  lv_obj_clear_flag(btn_exit, LV_OBJ_FLAG_SCROLLABLE);     /// Flags
  lv_obj_clear_flag(btn_exit, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(btn_exit, btn_exit_click, LV_EVENT_CLICKED, NULL);

  label_btn_exit = lv_label_create(btn_exit);
  lv_obj_set_width(label_btn_exit, LV_SIZE_CONTENT);   /// 1
  lv_obj_set_height(label_btn_exit, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_align(label_btn_exit, LV_ALIGN_CENTER);
  lv_label_set_text(label_btn_exit, "Exit TPCAL");

  btn_reset = lv_btn_create(container_panel);
  lv_obj_set_width(btn_reset, 100);
  lv_obj_set_height(btn_reset, 50);
  lv_obj_set_align(btn_reset, LV_ALIGN_CENTER);
  lv_obj_add_flag(btn_reset, LV_OBJ_FLAG_SCROLL_ON_FOCUS);  /// Flags
  lv_obj_clear_flag(btn_reset, LV_OBJ_FLAG_SCROLLABLE);     /// Flags
  lv_obj_clear_flag(btn_reset, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(btn_reset, btn_reset_click, LV_EVENT_CLICKED, NULL);

  label_btn_reset = lv_label_create(btn_reset);
  lv_obj_set_width(label_btn_reset, LV_SIZE_CONTENT);   /// 1
  lv_obj_set_height(label_btn_reset, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_align(label_btn_reset, LV_ALIGN_CENTER);
  lv_label_set_text(label_btn_reset, "Try again");

  obj_point = lv_obj_create(scr);
  lv_obj_set_width(obj_point, 10);
  lv_obj_set_height(obj_point, 10);
  lv_obj_set_x(obj_point, initial_point->x);
  lv_obj_set_y(obj_point, initial_point->y);
  lv_obj_set_align(obj_point, LV_ALIGN_CENTER);
  lv_obj_add_flag(obj_point, LV_OBJ_FLAG_FLOATING);      /// Flags
  lv_obj_clear_flag(obj_point, LV_OBJ_FLAG_SCROLLABLE);  /// Flags

    // Each axis needs its own animation
  lv_anim_init(&anim_px);
  lv_anim_set_var(&anim_px, obj_point);
  lv_anim_set_time(&anim_px, 1000);
  // lv_anim_set_user_data(&anim_px, anim_px_user_data);
  // lv_anim_set_custom_exec_cb(&anim_px, _ui_anim_callback_set_x);
  lv_anim_set_values(&anim_px, lv_obj_get_x(obj_point), initial_point->x);
  lv_anim_set_path_cb(&anim_px, lv_anim_path_linear);
  lv_anim_set_delay(&anim_px, 0);
  // lv_anim_set_deleted_cb(&anim_px, _ui_anim_callback_free_user_data);
  lv_anim_set_playback_time(&anim_px, 0);
  lv_anim_set_playback_delay(&anim_px, 0);
  lv_anim_set_repeat_count(&anim_px, 0);
  lv_anim_set_repeat_delay(&anim_px, 0);
  lv_anim_set_early_apply(&anim_px, false);
  lv_anim_start(&anim_px);

  lv_anim_init(&anim_py);
  lv_anim_set_var(&anim_py, obj_point);
  lv_anim_set_time(&anim_py, 1000);
  lv_anim_set_values(&anim_py, lv_obj_get_y(obj_point), initial_point->y);
  lv_anim_set_path_cb(&anim_py, lv_anim_path_linear);
  lv_anim_set_delay(&anim_py, 0);
  lv_anim_set_playback_time(&anim_py, 0);
  lv_anim_set_playback_delay(&anim_py, 0);
  lv_anim_set_repeat_count(&anim_py, 0);
  lv_anim_set_repeat_delay(&anim_py, 0);
  lv_anim_set_early_apply(&anim_py, false);
  lv_anim_start(&anim_py);

}

/**
 * @brief Move the target to a new position. Plays an animation of ~1s.
 * 
 * If callback cb_anim set on init is not NULL, it will be called on completion.
 * This can be used to prevent misclicks between calibration points. 
 *
 * @param target Object to move
 * @param p New point for the object
 */
void tpcal_scr_set_pos(const lv_point_t* p) {
  lv_anim_set_values(&anim_px, lv_obj_get_x(obj_point), p->x);
  lv_anim_set_values(&anim_py, lv_obj_get_y(obj_point), p->y);
  lv_anim_start(&anim_px);
  lv_anim_start(&anim_py);
}

/** Show navigation panel (exit, reset)*/
void tpcal_scr_show_nav(void) {
  lv_obj_add_flag(btn_reset, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_flag(btn_exit, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(container_panel, LV_OBJ_FLAG_HIDDEN);
}

/** Hide navigation panel (default state) (exit, reset)*/
void tpcal_scr_hide_nav(void) {
  lv_obj_clear_flag(btn_reset, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(btn_exit, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_flag(container_panel, LV_OBJ_FLAG_HIDDEN);
}

/** Set instructional label to new instruction. */
void set_instruction(const char* text) { lv_label_set_text(label_instruct, text); }

/* STATIC FUNCTIONS ***********************************************************/

/** wrapper for executing startup application on screen destroy*/
static void ondestroy(lv_event_t* e) {
  lv_indev_set_cursor(lv_indev_get_act(), NULL);
  if (cb_destroy != NULL) { cb_destroy(); }
#ifdef DEBUG
  else { printf("\nNo startup application was set. (tpcal)\n"); }
#endif
}

static void btn_exit_click(lv_event_t *e) {
  lv_obj_del(scr);    // LVGL will call ondestroy callback, startup app.
}

static void btn_reset_click(lv_event_t *e) {
  set_instruction(TPCAL_LABEL_TAP);
  tpcal_scr_hide_nav();

  if (cb_reset != NULL) { cb_reset(); }
#ifdef DEBUG
  else { printf("\nNo reset callback was set. (tpcal)\n"); }
#endif
}