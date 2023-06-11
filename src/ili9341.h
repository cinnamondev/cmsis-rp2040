#ifdef __cplusplus
    extern "C" {
#endif

#ifdef __cplusplus
    }
#endif    



/**
 * @file ili9314.h
 * 
 * Header for Pico ili9314 driver.
 */
#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include "hardware/spi.h"

/*********************
 *      DEFINES
 *********************/

// MADCTL parameters (36h)
#define MADCTL_MY           0x80
#define MADCTL_MX           0x40
#define MADCTL_MV           0x20
#define MADCTL_ML           0x10
#define MADCTL_BGR          0x08
#define MADCTL_RGB          0x00
#define MADCTL_MH           0x04

/**********************
 *      TYPEDEFS
 **********************/

 
enum rotation_t {
    R0    = 0,
    R90   = MADCTL_MV|MADCTL_MX,
    R180  = MADCTL_MX|MADCTL_MY,
    R270  = MADCTL_MV|MADCTL_MY,
    R0F   = MADCTL_MX,
    R90F  = MADCTL_MV|MADCTL_MX|MADCTL_MY,
    R180F = MADCTL_MY,
    R270F = MADCTL_MV|MADCTL_MX,
};


struct ili9341_cfg_t {
    uint8_t rx;
    uint8_t tx;
    uint8_t sck;
    uint8_t cs;
    uint8_t dc;
    uint8_t resx;
    enum rotation_t crot;
    spi_inst_t* iface;
};


/**********************
 * GLOBAL PROTOTYPES
 **********************/

void ili9341_init(void);
/* Initialize low level display driver */
void lv_ili9341_init(void);

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void);

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void);

// 
void ili9341_init();
void ili9341_write();
void ili9341_write_blk();
void ili9341_read();

void ili9341_init(void);

void ili9341_rotate(enum rotation_t r); // See ROTATE_ symbols.

// SPI methods

void ili9341_cmd(uint8_t cmd);
void ili9341_param(uint8_t param);
void ili9341_cmd_p(uint8_t cmd, uint8_t param);
void ili9341_cmd_params(uint8_t cmd, int n,...);

void configure_ili9341(struct ili9341_cfg_t cfg);
void ili9341_rotate(enum rotation_t r);

void ili9341_sw_res();
void ili9341_hw_res();

void ili9341_pixel(int x, int y, uint16_t color);
void ili9341_bmp(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t *bitmap);

void ili9341_cmd_mparam(uint8_t cmd, int len, uint8_t* params);
/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif