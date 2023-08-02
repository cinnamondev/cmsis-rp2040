#ifdef __cplusplus
extern "C" {
#endif

/* INCLUDES *******************************************************************/
#include "lvgl.h"
/* DEFINES ********************************************************************/

/* TYPEDEF/STRUCTURES *********************************************************/

/* MACROS *********************************************************************/

/* FUNCTIONS ******************************************************************/

void tpcal_register(lv_indev_drv_t *indev);
void tpcal_nvs_save(void (*cb)(void));
void tpcal_nvs_load(void (*cb)(void));
void tpcal_calib(void (*cb)(void));
void tpcal_translate(int16_t *x, int16_t *y) ;

#ifdef __cplusplus
} /* extern "C" */
#endif