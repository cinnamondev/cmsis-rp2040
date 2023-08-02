#ifdef __cplusplus
extern "C" {
#endif

/* INCLUDES *******************************************************************/
#include "lvgl.h"
/* DEFINES ********************************************************************/
#ifndef TPCAL_CIRC_SIZE
#define TPCAL_CIRC_SIZE 10
#endif

#ifndef TPCAL_LABEL_RESET
#define TPCAL_LABEL_RESET "Recalibrate"
#endif

#ifndef TPCAL_LABEL_ACCEPT
#define TPCAL_LABEL_ACCEPT "Exit TPCAL"
#endif

#ifndef TPCAL_LABEL_TAP
#define TPCAL_LABEL_TAP "Precisely press the centre of the point displayed on screen."
#endif

#ifndef TPCAL_LABEL_FIN
#define TPCAL_LABEL_FIN "Calibration complete! Drag the cursor around to test the functionality."
#endif
/* TYPEDEF/STRUCTURES *********************************************************/

/* MACROS *********************************************************************/

/* FUNCTIONS ******************************************************************/
void tpcal_scr_create(lv_point_t* initial_pos, void(*onready)(void), void (*onreset)(void), void(*onanim)(lv_anim_t*));
void tpcal_scr_destroy(void);
void tpcal_scr_state_complete(void);
void tpcal_set_point(lv_point_t p);

#ifdef __cplusplus
} /* extern "C"*/
#endif