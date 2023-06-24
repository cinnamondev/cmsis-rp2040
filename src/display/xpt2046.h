
#include "hardware/spi.h"
#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
struct xpt2046_cfg_t {
  uint8_t rx;
  uint8_t tx;
  uint8_t sck;
  uint8_t cs;
  uint8_t irq;
  spi_inst_t *iface;
};
/**********************
 * GLOBAL PROTOTYPES
 **********************/
void lv_port_indev_init(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif
