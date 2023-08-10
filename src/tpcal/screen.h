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
#define TPCAL_LABEL_TAP "Press the centre of the point shown on the screen."
#endif

#ifndef TPCAL_LABEL_FIN
#define TPCAL_LABEL_FIN "Calibration complete."
#endif
/* TYPEDEF/STRUCTURES *********************************************************/

/* MACROS *********************************************************************/

/* FUNCTIONS ******************************************************************/

void tpcal_scr_init(void (*cb_ondestroy)(void), void (*cb_onreset)(void),
                    void (*cb_onposition)(lv_anim_t*),
                    const lv_point_t* initial_point);
void tpcal_scr_set_pos(const lv_point_t* p);
void tpcal_scr_show_nav(void);
void tpcal_scr_hide_nav(void);
void set_instruction(const char* text);

#ifdef __cplusplus
} /* extern "C"*/
#endif