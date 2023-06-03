/**
 * @file ILI9341.h
 *
 */


#include "hardware/spi.h"
#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>
/*
#ifndef LV_DRV_NO_CONF
#ifdef LV_CONF_INCLUDE_SIMPLE
#include "lv_drv_conf.h"
#else
#include "../../lv_drv_conf.h"
#endif
#endif
*/

#include "lvgl.h"


#if LV_COLOR_DEPTH != 16
#error "ILI9341 currently supports 'LV_COLOR_DEPTH == 16'. Set it in lv_conf.h"
#endif

#if LV_COLOR_16_SWAP != 1
#error "ILI9341 SPI requires LV_COLOR_16_SWAP == 1. Set it in lv_conf.h"
#endif

/*********************
 *      DEFINES
 *********************/
#define ILI9341_BGR true
#define ILI9341_RGB false

/**********************
 *      TYPEDEFS
 **********************/

struct ili9341_cfg_t {
    uint8_t rx;
    uint8_t tx;
    uint8_t sck;
    uint8_t cs;
    uint8_t dc;
    uint8_t resx;
    spi_inst_t* iface;
};



/**********************
 * GLOBAL PROTOTYPES
 **********************/
void config_ili9341(struct ili9341_cfg_t cfg);
lv_disp_drv_t* ili9341_driver(void);

void ili9341_init(void);
void ili9341_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p);
void ili9341_rotate(int degrees, bool bgr);
/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif
