/**
 * @file ili9314.h
 *
 * Header for Pico ili9314 driver.
 */
#ifdef __cplusplus
extern "C" {
#endif

/* INCLUDES *******************************************************************/

#include "lvgl.h"
#include "hardware/spi.h"

/* DEFINES ********************************************************************/

// ILI9341 command set
// Standard command set (Level1  commands)
#define ILI9341_NOP 0x00         // no operation
#define ILI9341_SWRESET 0x01     // software reset
#define ILI9341_R_ID 0x04        // display id information
#define ILI9341_R_STATUS 0x09    // display status
#define ILI9341_R_DPWRM 0x0A     // display power mode
#define ILI9341_R_MADCTL 0x0B    // read matctl
#define ILI9341_R_PIXELF 0x0C    // read pixel format
#define ILI9341_R_IMGF 0x0D      // read image format
#define ILI9341_R_SIGNALM 0x0E   // read signal mode
#define ILI9341_R_DDIAGRES 0x0F  // self diagnostic result
#define ILI9341_SLEEP 0x10       // sleep mode
#define ILI9341_SLEEP_OUT 0x11   // wake from sleep
#define ILI9341_PARTIAL_ON 0x12  // partial mode on
#define ILI9341_NORMDISP_ON 0x13 // normal display mode on
#define ILI9341_INVERT_OFF 0x20  // inversion off
#define ILI9341_INVERT_ON 0x21   // inversion on
#define ILI9341_GAMMA 0x26       // gamma set
#define ILI9341_DISP_OFF 0x28    // display off
#define ILI9341_DISP_ON 0x29     // display on
#define ILI9341_COL_F_S 0x2A     // column address set
#define ILI9341_PGE_F_S 0x2B     // page address set
#define ILI9341_W_MEM 0x2C       // memory write
#define ILI9341_CLR_SET 0x2D     // color set
#define ILI9341_R_MEM 0x2E       // memory read
#define ILI9341_PAREA 0x30       // partial area
#define ILI9341_VSCR_DEF 0x33    // vertical scrolling definiton
#define ILI9341_TEAR_OFF 0x34    // tearing effect line off
#define ILI9341_TEAR_ON 0x35     // tearing effect line on
#define ILI9341_MAC 0x36         // memory address control
#define ILI9341_VSCR_S_F 0x37    // vertical scrolling start address
#define ILI9341_IDLE_OFF 0x38    // idle mode off
#define ILI9341_IDLE_ON 0x39     // idle mode on
#define ILI9341_PIXEL_FMT 0x3A   // set pixel format
#define ILI9341_W_MEM_C 0x3C     // write memory continue
#define ILI9341_R_MEM_C 0x3E     // read memory continue
#define ILI9341_TEAR_SC_S 0x44   // set tear scanline
#define ILI9341_SCAN_GET 0x45    // get scanline
#define ILI9341_W_BRIGHT 0x51    // write display brightness
#define ILI9341_R_BRIGHT 0x52    // read display brightness
#define ILI9341_W_CTRLD 0x53     // write ctrl display
#define ILI9341_R_CTRLD 0x54     // read ctrl display
#define ILI9341_W_CABC 0x55      // write content adaptive brightness control
#define ILI9341_R_CABC 0x56      // read content adaptive brightness control
#define ILI9341_W_CABC_MBR 0x5E  // write cabc min brightness
#define ILI9341_R_CABC_MBDR 0x5F // read cabc min brightness
#define ILI9341_ID1 0xDA         // read id1
#define ILI9341_ID2 0xDB         // read id2
#define ILI9341_ID3 0xDC         // read id3
// EXTENDED COMMAND SET (Level 2 commands)
#define ILI9341_RGB_IFRSCT 0xB0
#define ILI9341_FRAMEC_NORM 0xB1
#define ILI9341_FRAMEC_IDLE 0xB2
#define ILI9341_FRAMEC_PART 0xB3
#define ILI9341_INV_CONT 0xB4
#define ILI9341_BPRCH_CONT 0xB5
#define ILI9341_DISP_FNCC 0xB6
#define ILI9341_ENT_MODE_S 0xB7
#define ILI9341_BL1 0xB8
#define ILI9341_BL2 0xB9
#define ILI9341_BL3 0xBA
#define ILI9341_BL4 0xBB
#define ILI9341_BL5 0xBC
#define ILI9341_BL7 0xBE
#define ILI9341_BL8 0xBF
#define ILI9341_PWC1 0xC0
#define ILI9341_PWC2 0xC1
#define ILI9341_VCOMC1 0xC5
#define ILI9341_VCOMC2 0xC7
#define ILI9341_NV_MW 0xD0
#define ILI9341_NV_MPK 0xD1
#define ILI9341_NV_MSR 0xD2
#define ILI9341_ID4 0xD3
#define ILI9341_PGMMA_COR 0xE0
#define ILI9341_NGMMA_COR 0xE1
#define ILI9341_DGC1 0xE2
#define ILI9341_DGC2 0xE3
#define ILI9341_IFC 0xF6
// extended register cmd set
#define ILI9341_PWRA 0xCB
#define ILI9341_PWRB 0xCF
#define ILI9341_DTCA 0xE8
#define ILI9341_DTCAEXT 0xF9
#define ILI9341_DTCB 0xEA
#define ILI9341_POSEQC 0xED
#define ILI9341_E3G 0xF2
#define ILI9341_PUMPRATIO 0xF7

// Module consts.
#define ILI9341_HOR 320
#define ILI9341_VER 240

// MADCTL parameters (36h)
#define MADCTL_MY 0x80
#define MADCTL_MX 0x40
#define MADCTL_MV 0x20
#define MADCTL_ML 0x10
#define MADCTL_BGR 0x08
#define MADCTL_RGB 0x00
#define MADCTL_MH 0x04

// XPT related. See Table 4/5 for A2,A1,A0 parameters)
// XPT measurement parameters (A2,A1,A0)
#define XPT_MUX_X 0x50  /** X Position 0101 0000  1101*/
#define XPT_MUX_Y 0x10  /** Y Position 0001 0000*/
#define XPT_MUX_Z1 0x30 /** Cross panel (See figure 11) */
#define XPT_MUX_Z2 0x40 /** Cross panel (See figure 11) */
// Extended measurement parameters. XPT2046 has measurement modes that are not
// normally implemented.

#define XPT_MUX_TEMP0 0x00 /** Temperature naive measurement. (Page 18) */
#define XPT_MUX_VBAT 0x20  /* VBAT pin measurement  (Page 19) */
#define XPT_MUX_AUX 0x60   /** IN pin measurement (Page ?)*/
#define XPT_MUX_TEMP1 0x70 /** Temperature 2C measurement. (Page 18) */

#define XPT_MODE_8B 0x04
#define XPT_SET_PB1 0x02
#define XPT_SET_PB0 0x01

/* TYPEDEF/STRUCTURES *********************************************************/

enum rotation_t {
  R0 = 0,
  R90 = MADCTL_MV | MADCTL_MX,
  R180 = MADCTL_MX | MADCTL_MY,
  R270 = MADCTL_MV | MADCTL_MY,
  R0F = MADCTL_MX,
  R90F = MADCTL_MV | MADCTL_MX | MADCTL_MY,   // normal format
  R180F = MADCTL_MY,
  R270F = MADCTL_MV | MADCTL_MX,
};

/**
 * @brief SPI bus configuration parameters. Pins should be set to valid pins
 * on pico.
 */
struct ili_cfg_t {
  uint8_t rx;
  uint8_t tx;
  uint8_t sck;
  uint8_t ili_cs;
  uint8_t ili_dc;
  uint8_t ili_resx;
  uint8_t xpt_irq;
  uint8_t xpt_cs;
  spi_inst_t *iface;
};

struct drv_module_t {
  lv_disp_drv_t* display;
  lv_indev_drv_t* touch;
};

struct xpt_trans_t {
  int32_t ax;
  int32_t bx;
  int32_t dx;
  int32_t ay;
  int32_t by;
  int32_t dy;
  bool calibrated;
};


/* STATIC PROTOTYPES **********************************************************/

/* STATIC VARIABLES  **********************************************************/

/* MACROS *********************************************************************/

/* FUNCTIONS ******************************************************************/

// shared
void bus_config(struct ili_cfg_t cfg);
void bus_init(void);
void disp_module_init(void);
struct drv_module_t lv_disp_init(void);

// ili9341
void ili9341_cmd(uint8_t cmd);
void ili9341_param(uint8_t param);
void ili9341_cmd_p(uint8_t cmd, uint8_t param);
void ili9341_cmd_mparam(uint8_t cmd, int len, uint8_t *params);
void ili9341_init(void);
void ili_bounds(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void dma_tx_isr(void);
void ili9341_bmp(uint16_t _x1, uint16_t _y1, uint16_t _x2, uint16_t _y2,
                 uint16_t *bitmap);
void ili9341_bmp_block(uint16_t _x1, uint16_t _y1, uint16_t _x2, uint16_t _y2,
                     uint16_t *bitmap);
void ili9341_pixel(int x, int y, uint16_t color);
void ili9341_hw_res();
void ili9341_rotate(enum rotation_t r);
lv_disp_drv_t* lv_ili9341_init(void);
void lv_ili_rotate(enum rotation_t r);

//xpt2046
lv_indev_drv_t* lv_xpt2046_init(void);
void xpt2046_init(void);
void xpt2046_deaf_cmd(uint8_t cmd);
void xpt2046_cmd(uint8_t cmd, uint16_t* output);
bool xpt2046_touch();
bool xpt2046_ispressed();
void xpt2046_xyz(uint16_t *x, uint16_t *y);

#ifdef __cplusplus
} /*extern "C"*/
#endif

