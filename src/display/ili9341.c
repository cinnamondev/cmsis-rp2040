/**
 * @file ili9341.c
 * @brief ILI9341 and XPT2040 Driver with LVGL suppport.
 * @copyright Copyright (c) 2023
 *
 * References: 
 * - https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf
 * - https://grobotronics.com/images/datasheets/xpt2046-datasheet.pdf
 * - https://github.com/lvgl/lv_drivers
 * - https://docs.lvgl.io/8.3/porting/display.html
 * - https://docs.lvgl.io/8.3/porting/indev.html
 * - https://github.com/lvgl/lv_port_raspberry_pi_pico_mdk 
 *   (porting LVGL via GorgonMeducer's `perf_counter` library)
 */


/* INCLUDES *******************************************************************/

#include "tpcal.h"
#include "ili9341.h"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "lvgl.h"
#include "pico/time.h"

#include <stdio.h>
/* DEFINES ********************************************************************/

#define LV_ILI_DRV_BLOCKING

#define BUFFER_SIZE (int)(ILI9341_HOR * ILI9341_VER * 0.1)
#define SPEED_ILI   62.5e6
#define SPEED_XPT   2e6


/* TYPEDEF/STRUCTURES *********************************************************/

/* STATIC PROTOTYPES **********************************************************/

/* ILI9341 */

static inline void _ili_write_bytes(uint8_t *payload, int n);
static inline void _ili_write_byte(uint8_t payload);
static inline void _ili_write_hword(uint16_t *payload);
static inline void _ili_write_hwords(uint16_t *payload, int n);
static void _lv_ili_flush_irq(void);
static void _lv_ili_flush(lv_disp_drv_t *disp, const lv_area_t *area,
                         lv_color_t *px_map);

/* XPT2046 */

/** Updates last coordinates if possible.   */
static void lv_xpt_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
static void xpt_roll_avg(uint16_t *x, uint16_t *y, uint16_t* n);
static void xpt_correct(uint16_t *x, uint16_t *y);
static void save_coef_ifemp(void);
static bool try_load_coef(void);

/* STATIC VARIABLES  **********************************************************/
 
// DMA channel for background writing frames
static uint DMA_TX;
static dma_channel_config DMA_TX_CFG;

// LVGL
static lv_disp_drv_t LV_ILI_DRV;
static lv_indev_drv_t LV_XPT_INDEV_DRV;
// LVGL Display
static lv_disp_draw_buf_t lv_draw_buf_dsc;
static lv_color_t lv_buf_1[BUFFER_SIZE];
static lv_color_t lv_buf_2[BUFFER_SIZE];

// transformation co-efficients for XPT
static struct xpt_trans_t xpt_coef = {
  .calibrated = false,
};

static struct ili_cfg_t spi_cfg = {
    // Default assignments:
    .iface = spi1, // Use SPI1 interface
    .rx = 12,      // SPI_RX / MISO
    .tx = 11,      // SPI_TX / MOSI
    .sck = 10,     // SPI_SCK / SPI CLOCK

    .ili_dc = 15,      // DC (data select): low for dat high reg.
    .ili_cs = 13,      // Chip select, active low.
    .ili_resx = 14,    // Chip reset, active low.

    .xpt_cs = 8,
    .xpt_irq = 7 
};                 // change prior to lv_ili9341_init
                   // to use different assignments.
// pin definitions

/* MACROS *********************************************************************/

/** SPI might be busy due to DMA at any time, so you should check before.*/
#define SPI_WAIT_FREE()                \
  while (spi_is_busy(spi_cfg.iface)) { \
    tight_loop_contents();             \
  }

// NOPs are used to account for rise/fall time. Delay only in order of ns.

#define ILI_SEL()              \
  gpio_put(spi_cfg.ili_cs, 0); \
  asm volatile("nop\nnop\nnop\nnop\nnop")

#define ILI_DESEL()                        \
  asm volatile("nop\nnop\nnop\nnop\nnop"); \
  gpio_put(spi_cfg.ili_cs, 1)

#define XPT_SEL()              \
  gpio_put(spi_cfg.xpt_cs, 0); \
  asm volatile("nop\nnop\nnop\nnop\nnop")

#define XPT_DESEL()                        \
  asm volatile("nop\nnop\nnop\nnop\nnop"); \
  gpio_put(spi_cfg.xpt_cs, 1)

/* FUNCTIONS ******************************************************************/

/**** SHARED *****/

/**
 * @brief configure pin out of module
 * 
 * @param cfg ili_cfg_t configuration struct  
 */
void bus_config(struct ili_cfg_t cfg) { spi_cfg = cfg; }

/**
 * @brief Sets up the pins and interface of the shared SPI bus.
 */
void bus_init(void) {
  gpio_set_function(spi_cfg.rx, GPIO_FUNC_SPI);
  gpio_set_function(spi_cfg.tx, GPIO_FUNC_SPI);
  gpio_set_function(spi_cfg.sck, GPIO_FUNC_SPI);

  uint speed = spi_init(spi_cfg.iface, SPEED_ILI); // 16 bit spi
  // The ILI9341 can run crazy fast!
#ifdef DEBUG
  printf("Running SPI at actual speed: %u", speed); // Report actual speed
#endif
  spi_set_format(spi_cfg.iface, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
}

/**
 * @brief (no lvgl) initialize entire module (ili & xpt)
 * 
 */
void disp_module_init(void) {
  bus_init();
  ili9341_init();
  xpt2046_init();
}

/**
 * @brief initialize chips and lvgl drivers for ili touch module.
 * 
 * @return struct drv_module_t 
 */
struct drv_module_t lv_disp_init(void) {
  bus_init();
  return (struct drv_module_t) {
    .display = lv_ili9341_init(),
    .touch = lv_xpt2046_init(),
  };
}

/**** ILI9341 / disp*****/
/**** SPI *****/

/**
 * @brief Sends a command to the ILI9341.
 *
 * @param cmd Command to send
 */
void ili9341_cmd(uint8_t cmd) {
  SPI_WAIT_FREE();
  gpio_put(spi_cfg.ili_dc, 0);  // cmd mode
  _ili_write_byte(cmd);
  gpio_put(spi_cfg.ili_dc, 1);  // data mode (initial)
}

/**
 * @brief Sends a "parameter" to the chip.
 *
 * @param param parameter to send.
 */
void ili9341_param(uint8_t param) {
  SPI_WAIT_FREE();
  _ili_write_byte(param);
}

/**
 * @brief send a command and its parameter
 *
 * @param cmd command to send
 * @param param parameter to send
 */
void ili9341_cmd_p(uint8_t cmd, uint8_t param) {
  ili9341_cmd(cmd);
  ili9341_param(param);
}

/**
 * @brief Send many data/parameters following a command.
 *
 * @param cmd Command to send
 * @param len Number of parameters (bytes)
 * @param params Parameters to send
 */
void ili9341_cmd_mparam(uint8_t cmd, int len, uint8_t *params) {
  ili9341_cmd(cmd);
  ILI_SEL();
  spi_write_blocking(spi_cfg.iface, (uint8_t *)params, len);
  ILI_DESEL();
}

/**** COMMANDS *****/

/**
 * @brief Initialize the ILI9341 chip, according to spi_cfg.
 *
 * Sets up all pins connecting the RP2040 to the ILI9341, and sets up Direct
 * Memory Access between the chip and pico. IRQ1 must be available as an
 * exclusive interrupt.
 * @warning Shared SPI bus should be initialized via `ili_bus_init`
 */
void ili9341_init(void) {
  // config CS (active low)
  gpio_init(spi_cfg.ili_cs);
  gpio_set_dir(spi_cfg.ili_cs, GPIO_OUT);
  gpio_put(spi_cfg.ili_cs, 1);

  // config DC (data lo)
  gpio_init(spi_cfg.ili_dc);
  gpio_set_dir(spi_cfg.ili_dc, GPIO_OUT);
  gpio_put(spi_cfg.ili_dc, 1);
  // config resx (active low)
  gpio_init(spi_cfg.ili_resx);
  gpio_set_dir(spi_cfg.ili_resx, GPIO_OUT);
  gpio_put(spi_cfg.ili_resx, 1);

  // DMA: 1byte/transfer over SPI. 
  DMA_TX = dma_claim_unused_channel(true);
  DMA_TX_CFG = dma_channel_get_default_config(DMA_TX);
  channel_config_set_transfer_data_size(&DMA_TX_CFG, DMA_SIZE_8);
  channel_config_set_dreq(&DMA_TX_CFG, spi_get_dreq(spi_cfg.iface, true));
  dma_channel_configure(DMA_TX, &DMA_TX_CFG, &spi_get_hw(spi_cfg.iface)->dr,
                        NULL, 0, 0);
  
  // Hard & soft reset ILI.
  ili9341_hw_res();
  ili9341_cmd(ILI9341_SWRESET);
  sleep_ms(130); // wait another 5ms following sw reset.
  ili9341_cmd(ILI9341_DISP_OFF);

  // power control
  ili9341_cmd_p(ILI9341_PWC1, 0x23);

  ili9341_cmd_p(ILI9341_PWC2, 0x10);
  // vcom control
  ili9341_cmd_mparam(ILI9341_VCOMC1, 2, (uint8_t[2]){0x3e, 0x28});
  ili9341_cmd_p(ILI9341_VCOMC2, 0x86);

  // mac / madctl
  ili9341_rotate(R90F);

  ili9341_cmd_p(ILI9341_PIXEL_FMT, 0x55); // 16b/pixel (rgb565)

  ili9341_cmd_mparam(ILI9341_FRAMEC_NORM, 2,
                     (uint8_t[2]){0x00, // 70 hz
                                  0x1B});

  ili9341_cmd_p(0xf2, 0x00);

  ili9341_cmd_p(ILI9341_GAMMA, 0x01);

  // positive gamma
#ifdef ILI_CONF_GAMMA
  ili9341_cmd_mparam(ILI9341_PGMMA_COR, 15,
                     (uint8_t[15]){
                         0x0f,
                         0x31,
                         0x2b,
                         0x0c,
                         0x0e,
                         0x08,
                         0x4e,
                         0xf1,
                         0x37,
                         0x07,
                         0x10,
                         0x03,
                         0x0e,
                         0x09,
                         0x00,
                     });
  // negative gamma
  ili9341_cmd_mparam(ILI9341_NGMMA_COR, 15,
                     (uint8_t[15]){
                         0x00,
                         0x0e,
                         0x14,
                         0x03,
                         0x11,
                         0x07,
                         0x31,
                         0xc1,
                         0x48,
                         0x08,
                         0x0f,
                         0x0c,
                         0x31,
                         0x36,
                         0x0f,
                     });
#endif
  ili9341_cmd_mparam(ILI9341_DISP_FNCC, 3, (uint8_t[3]){0x08, 0x82, 0x27});

  ili9341_cmd(ILI9341_W_MEM);
  sleep_ms(150);

  ili9341_cmd(ILI9341_SLEEP_OUT);
  sleep_ms(150);
  ili9341_cmd(ILI9341_DISP_ON);
  sleep_ms(150);
  gpio_put(spi_cfg.ili_dc, 1); // ensure data mode for img writ.
}

/**
 * @brief Set the bounds of the image frame memory to write to.
 *
 * @param x0 Column address start
 * @param y0 Column end
 * @param x1 Page address start
 * @param y1 Page end
 */
void ili_bounds(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  ili9341_cmd_mparam(
      ILI9341_COL_F_S, 4,
      (uint8_t[4]){(x0 >> 8), (x0 & 0xff), (x1 >> 8), (x1 & 0xff)});
  // ili_write_hwords((uint16_t[]) {x0,x1}, 2);// hword params
  ili9341_cmd_mparam(
      ILI9341_PGE_F_S, 4,
      (uint8_t[4]){(y0 >> 8), (y0 & 0xff), (y1 >> 8), (y1 & 0xff)});
  // ili_write_hwords((uint16_t[]) {y0,y1}, 2);// hword params
  ili9341_cmd(ILI9341_W_MEM);
}
/**** DRAWING     *****/

/**
 * @brief Bare mininum DMA call for async. dma.
 *
 * Additional logic should be done via your own isr. 
 * 
 */
void dma_tx_isr(void) {
  ILI_DESEL();
}

/**
 * @brief Write image data to specifed area in frame memory.
 *
 * @param _x1 Column Start
 * @param _y1 Column End
 * @param _x2 Row Start
 * @param _y2 Row End
 * @param bitmap color pixel map (rgb565 format)
 *
 * @warning Handler must be set before calling (ie: dma_tx_isr, lv_dma_tx_isr).
 */
void ili9341_bmp(uint16_t _x1, uint16_t _y1, uint16_t _x2, uint16_t _y2,
                 uint16_t *bitmap) {
  // Truncuate anything OOB.
  uint16_t x1 = _x1 < 0 ? 0 : _x1;
  uint16_t y1 = _y1 < 0 ? 0 : _y1;
  uint16_t x2 = _x2 > ILI9341_HOR - 1 ? ILI9341_HOR - 1 : _x2;
  uint16_t y2 = _y2 > ILI9341_VER - 1 ? ILI9341_VER - 1 : _y2;
  dma_channel_set_trans_count(DMA_TX, (x2-x1+1)*(y2-y1+1)*2, false);
  SPI_WAIT_FREE();
  ili_bounds(x1, y1, x2, y2);
  ILI_SEL();
  dma_channel_set_read_addr(DMA_TX, bitmap, true);
}

/**
 * @brief Write image data to specifed area in frame memory.
 * 
 * @param _x1 Column start
 * @param _y1 Column end
 * @param _x2 Row start
 * @param _y2 Row end
 * @param bitmap color pixel map (rgb565)
 */
void ili9341_bmp_block(uint16_t _x1, uint16_t _y1, uint16_t _x2, uint16_t _y2,
                     uint16_t *bitmap) {
  ili9341_bmp(_x1, _y1, _x2, _y2, bitmap);
  dma_channel_wait_for_finish_blocking(DMA_TX);
  ILI_DESEL();
}

/**
 * @brief Draw pixel on display (untested)
 * 
 * @param x x coordinate (0<=x<320)
 * @param y y coordinate (0<=y<240)
 * @param color rgb565 formatted color
 */
void ili9341_pixel(int x, int y, uint16_t color) {
  ili_bounds(x, y, x + 1, y + 1);
  _ili_write_hword(&color);
}

/**
 * @brief Hardware reset ILI9341 chip.
 *
 * @warning Blocks execution for ~20ms to get reset to be accepted and the chip
 * in a configurable state (not writeable state, consult datasheet.) 
 */
void ili9341_hw_res() {
  sleep_ms(10);
  gpio_put(spi_cfg.ili_resx, 0);
  sleep_us(20);
  gpio_put(spi_cfg.ili_resx, 1);
  sleep_ms(10);
}

/**
 * @brief Rotate the display to a specific orientation
 *
 * @param r Orientation to change to. `F` indicated the display is flipped
 * along the new axis.
 *
 * @warning only the frame memory interpretation changes.
 */
void ili9341_rotate(enum rotation_t r) {
  uint8_t madctr = MADCTL_BGR | r;
  ili9341_cmd_p(ILI9341_MAC, madctr);
}

/**** LVGL *****/

/**
 * @brief Initialize the ILI9341 and register it as a display.
 * 
 * @return lv_disp_drv_t* Pointer to the registered display driver.
 */
lv_disp_t* lv_ili9341_init(void) {
  //lv_disp_drv_t LV_ILI_DRV; // Driver *can* be local but 
  ili9341_init();

  // Partial render buffer (flush async via DMA.)
  lv_disp_draw_buf_init(&lv_draw_buf_dsc, lv_buf_1,
                        lv_buf_2,
                        BUFFER_SIZE
  );
  // Initialize driver and set it's members.
  lv_disp_drv_init(&LV_ILI_DRV);
  LV_ILI_DRV.hor_res = ILI9341_HOR;
  LV_ILI_DRV.ver_res = ILI9341_VER;
  LV_ILI_DRV.draw_buf = &lv_draw_buf_dsc;
  LV_ILI_DRV.flush_cb = _lv_ili_flush;

#ifndef LV_ILI_DRV_BLOCKING
  // lv_ili_flush -> DMA_TX -> IRQ1 -> lv_ili_flush_irq -> lv_disp_flush_ready
  dma_channel_set_irq1_enabled(DMA_TX, true);
  irq_set_exclusive_handler(DMA_IRQ_1, _lv_ili_flush_irq);
  irq_set_enabled(DMA_IRQ_1, true);
#endif

  return   lv_disp_drv_register(&LV_ILI_DRV);
}

/**
 * @brief Interrupt call - tells lvgl the flush is done when DMA has sent all
 * it's data.
 */
static void _lv_ili_flush_irq(void) {
  // prevent being stuck in IRQ
  dma_channel_acknowledge_irq1(DMA_TX);
  // sleep_us(10);
  ILI_DESEL();
  lv_disp_flush_ready(&LV_ILI_DRV);
}

/** LVGL flush_cb function. Define LV_ILI_DRV_BLOCKING to perform sync. flush.*/
static void _lv_ili_flush(lv_disp_drv_t *disp, const lv_area_t *area,
                         lv_color16_t *px_map) {
  if (area->x1 > (ILI9341_HOR - 1) || area->y1 > (ILI9341_VER - 1) ||
      area->x2 < 0 || area->y2 < 0) {
#ifdef DEBUG
    printf("Malformed lv_ili_flush input. Got x1 %u x2 %u y1 %u y2 %u.",
           area->x1, area->x2, area->y1, area->y2);
#endif
    lv_disp_flush_ready(disp);  // Prevent malformed input.
    return;
  }
#ifndef LV_ILI_DRV_BLOCKING
  ili9341_bmp(area->x1, area->y1, area->x2, area->y2, (uint16_t *)px_map);
  // `lv_disp_flush_ready` will be called via handler `lv_ili_dma_handle`.
#else
  ili9341_bmp_block(area->x1, area->y1, area->x2, area->y2, (uint16_t *)px_map);
  lv_disp_flush_ready(disp);
#endif
}

/**
 * @brief lvgl rotate display in run (todo, untested.)
 * 
 * @param r 
 */
void lv_ili_rotate(enum rotation_t r) {
  if (r & MADCTL_MV | r == 0) { // off-axis?
    LV_ILI_DRV.hor_res = ILI9341_HOR;  // parr. to initial
    LV_ILI_DRV.ver_res = ILI9341_VER;
  } else {
    LV_ILI_DRV.hor_res = ILI9341_VER; // norm. to initial
    LV_ILI_DRV.ver_res = ILI9341_HOR;
  }
  ili9341_rotate(r);
}

/* STATIC FUNCTIONS ***********************************************************/



/** Write `n` bytes over SPI. Does not modify DC. */
static inline void _ili_write_bytes(uint8_t *payload, int n) {
  ILI_SEL();
  spi_write_blocking(spi_cfg.iface, payload, n);
  ILI_DESEL();
}

/** Write a byte over SPI. Does not modify DC. */
static inline void _ili_write_byte(uint8_t payload) {
  _ili_write_bytes(&payload, 1);
}
/** Write a hword (16 bit/2 bytes) over SPI. Does not modify DC. */
static inline void _ili_write_hword(uint16_t *payload) {
  _ili_write_bytes((uint8_t*) payload, 2);
}

/** Write `n` hwords (16 bit/2 bytes) over SPI. Does not modify DC. */
static inline void _ili_write_hwords(uint16_t *payload, int n) {
  _ili_write_bytes((uint8_t*) payload, 2*n);
}

lv_indev_t* indev_touchpad;
lv_indev_t* lv_xpt2046_init(void) {
  /**
   * Here you will find example implementation of input devices supported by
   * LittelvGL:
   *  - Touchpad
   *  - Mouse (with cursor support)
   *  - Keypad (supports GUI usage only with key)
   *  - Encoder (supports GUI usage only with: left, right, push)
   *  - Button (external buttons to press points on the screen)
   *
   *  The `..._read()` function are only examples.
   *  You should shape them according to your hardware
   */
  xpt2046_init();
  /*Register a touchpad input device*/
  lv_indev_drv_init(&LV_XPT_INDEV_DRV);
  LV_XPT_INDEV_DRV.type = LV_INDEV_TYPE_POINTER;
  LV_XPT_INDEV_DRV.read_cb = lv_xpt_read;
  return lv_indev_drv_register(&LV_XPT_INDEV_DRV);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*------------------
 * Touchpad
 * -----------------*/


/**
 * @brief Initialize XPT2046 chip / IO.
 * 
 * Any XPT_CONFIG defines should occur before this is called.
 */
void xpt2046_init(void) { 
  // Only unique pins to the XPT are CS and \overline{PENIRQ}
  gpio_init(spi_cfg.xpt_cs);
  gpio_set_dir(spi_cfg.xpt_cs,GPIO_OUT);
#ifdef XPT_CONFIG_USE_PENIRQ
  gpio_init(8);                
  gpio_set_dir(8, GPIO_IN);
#endif
  }

/**
 * @brief Send an 8 bit command to the XPT2046.
 * 
 * Use this function when you don't care about the result (i.e: dummy calls.).
 * Otherwise, call `xpt2046_cmd` instead.
 * @param cmd command to write
 */
void xpt2046_deaf_cmd(uint8_t cmd) {
  cmd |= 0x8; // ensure control bit is high
  SPI_WAIT_FREE();
  spi_set_baudrate(spi_cfg.iface, SPEED_XPT);
  XPT_SEL();
  spi_write_blocking(spi_cfg.iface, &cmd, 1);
  XPT_DESEL();
  spi_set_baudrate(spi_cfg.iface, SPEED_ILI);
}

/**
 * @brief Sends an 8 bit command and returns a 16 bit (12 bit conversion)
 *
 * The result is bitshifted to make up for dead-space, depending on the state of
 * the MODE bit flag.
 *
 * @warning The return type is always a hword- irregardless of MODE.
 * @param cmd Command to write
 * @param out Conversion result (Shifted according to MODE.)
 */
void xpt2046_cmd(uint8_t cmd, uint16_t* output) {
  SPI_WAIT_FREE();
  spi_set_baudrate(spi_cfg.iface, SPEED_XPT);
  XPT_SEL();
  spi_write_read_blocking(spi_cfg.iface, &cmd, (uint8_t*)output, 2);
  XPT_DESEL();
  spi_set_baudrate(spi_cfg.iface, SPEED_ILI);
  //if (cmd & 0X08) { // is MODE high?
  //  return (output >> 8); // 0b00000000XXXXXXXX (8 bit result)
  //} else {
  //return (output); // 0b0000XXXXXXXXXXXX (12 bit result, default.)
  //}
}

/**
 * @brief Is display touched (SPI mechanism)
 * 
 * @return true Display touched (beyond z threshold)
 * @return false Display x  not touched.
 */
bool xpt2046_touch() {
  uint16_t z1,z2;
  xpt2046_cmd(XPT_MUX_Z1 | XPT_SET_PB0 | XPT_SET_PB1, &z1);
  xpt2046_cmd(XPT_MUX_Z2 | XPT_SET_PB0 | XPT_SET_PB1, &z2);
#ifdef DEBUG
  printf("touched? z1 %d z2 %d expr. %d t %d \n", z1, z2, (z1 - z2), (z1-z2)<4000);
#endif
  //return true;
  return (z1-z2)<4000;
}

/**
 * @brief Checks if the display is pressed
 * 
 * XPT_CONFIG_USE_PENIRQ should be defined if PENIRQ' is available to detect
 * pen input without an SPI write.
 * @return true 
 * @return false 
 */
bool xpt2046_ispressed() {
#ifdef XPT_CONFIG_USE_PENIRQ
  if (gpio_get(spi_cfg.xpt_irq)) return xpt2046_touch();
#else
  return xpt2046_touch();
#endif
}

/**
 * @brief Get coordinates from xpt2046.
 *
 * @param x X Coordinate (12 bit)
 * @param y Y Coordinate (12 bit)
 */
void xpt2046_xyz(uint16_t *x, uint16_t *y) {
  //xpt2046_cmd(XPT_MUX_X | XPT_SET_PB0 | XPT_SET_PB1, NULL);
  xpt2046_cmd(XPT_MUX_X | XPT_SET_PB0 | XPT_SET_PB1,x);
  xpt2046_cmd(XPT_MUX_Y | XPT_SET_PB0 | XPT_SET_PB1, y);
#ifdef DEBUG
  printf("xy %d %d", *x, *y);
#endif
}

/** Updates last coordinates if possible.   */
static void lv_xpt_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
  static uint16_t x = 0;   // Initial / Previous coords.
  static uint16_t y = 0;
  static uint16_t n = 0;
  /*Save the pressed coordinates and the state*/
  if (xpt2046_ispressed()) {
    xpt2046_xyz(&x,&y);
    xpt_roll_avg(&x,&y, &n);       // Update via rolling average
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    n=0;
    data->state = LV_INDEV_STATE_RELEASED;
  }
  // ensure update
  data->point.x = x;
  data->point.y = y;
}

/**
 * @brief Takes new coordinates and stores a rolling average 
 * 
 * @param x
 * @param y 
 * @param n 
 */
static void xpt_roll_avg(uint16_t *x, uint16_t *y, uint16_t* n) {
  static int16_t buf_x[10];
  static int16_t buf_y[10];

  for (int i = 9; i > 0; i--) {
    buf_x[i] = buf_x[i-1];  // Shift to make room for latest value.
    buf_y[i] = buf_y[i-1];
  }
  // new sample incoming! n is reset externally (on rel.)
  if ((*n) < 10) (*n)++;

  int32_t s_x= 0;
  int32_t s_y = 0;
  for(int i = 0; i < 10 ; i++) {
      s_x += buf_x[i];
      s_y += buf_y[i];
  }

  *x = s_x / *n;
  *y = s_y / *n;
}

/**
 * @brief Apply calibration parameters (if set)
 *
 *
 * @param x 
 * @param y 
 */
static void xpt_correct(uint16_t *x, uint16_t *y) {
  bool tmp_rotation_swap = false;
  bool tmp_rotation_invert_x = false;
  bool tmp_rotation_invert_y = false;

  if (tmp_rotation_swap) {    // X and Y swap? (Display rotation off-axis)
    int16_t tmp = *x;
    *x = *y;
    *y = tmp;
  }
  if (tmp_rotation_invert_x) {

  }
  // Set xpt_coef through calibration function or loader function to use
  // transformation matrix.
  if (xpt_coef.calibrated) {
    *x = xpt_coef.ax * (*x) + xpt_coef.bx * (*y) + xpt_coef.dx;
    *y = xpt_coef.ay * (*x) + xpt_coef.by * (*y) + xpt_coef.dy;
  }
}

/**
 * @brief Calibrate the display, by accepting `n` coordinates from the display
 * and touchscreen
 *
 * Both parameters must have a width of `3` points, as we are using a 3 point
 * calibration algorithm.
 * 
 * @param coords_disp 3 display coordinates
 * @param coords_touch 3 coordinates read from touchscreen
 */
void xpt_tpcal(struct point_t p_disp[], struct point_t p_touch[]) {
  //if (try_load_coef()) return;

  uint32_t determinant =  // det(A) A is 3x3 inptu matrix (z=1 all)
      (p_touch[0].x - p_touch[2].x) * (p_touch[1].y - p_touch[2].y) -
      (p_touch[1].x - p_touch[2].x) * (p_touch[0].y - p_touch[2].y);

  // I would refer to `Definitions for Equation 8` when looking at this in the
  // TI/Analog AN for this. (check docs for refs.). (a,b,d)(x,y) are co-effs.
  // of a transformation to account for any misallignment in both directions.
  // It also corrects for scale, so we either need to load calibration from
  // flash ASAP or, failing that, run the tpcal.
  xpt_coef.ax = ((p_disp[0].x - p_disp[2].x) * (p_touch[1].y - p_touch[2].y) -
                 (p_disp[1].x - p_disp[2].x) * (p_touch[0].y - p_touch[2].y)) /
                determinant;
  xpt_coef.bx = ((p_touch[0].x - p_touch[2].x) * (p_disp[1].x - p_disp[2].x) -
                 (p_touch[1].x - p_touch[2].x) * (p_disp[0].x - p_disp[2].x)) /
                determinant;
  xpt_coef.dx =
      ((p_disp[0].x) *
           ((p_touch[1].x * p_touch[2].y) - (p_touch[2].x * p_touch[1].y)) -
       (p_disp[1].x) *
           ((p_touch[0].x * p_touch[2].y) - (p_touch[2].x * p_touch[0].y)) +
       (p_disp[2].x) *
           ((p_touch[0].x * p_touch[1].y) - (p_touch[1].x * p_touch[0].y))) /
      determinant;
  xpt_coef.ay = ((p_disp[0].y - p_disp[2].y) * (p_touch[1].y - p_touch[2].y) -
                 (p_disp[1].y - p_disp[2].y) * (p_touch[0].y - p_touch[2].y)) /
                determinant;
  xpt_coef.by = ((p_touch[0].x - p_touch[2].x) * (p_disp[1].y - p_disp[2].y) -
                 (p_touch[1].x - p_touch[2].x) * (p_disp[0].y - p_disp[2].y)) /
                determinant;
  xpt_coef.dy =
      ((p_disp[0].y) *
           ((p_touch[1].x * p_touch[2].y) - (p_touch[2].x * p_touch[1].y)) -
       (p_disp[1].y) *
           ((p_touch[0].x * p_touch[2].y) - (p_touch[2].x * p_touch[0].y)) +
       (p_disp[2].y) *
           ((p_touch[0].x * p_touch[1].y) - (p_touch[1].x * p_touch[0].y))) /
      determinant;
  xpt_coef.calibrated = true;

  //save_coef_ifemp();
}


static void save_coef_ifemp(void) {
  // save xpt_coef to flash! we have a 4k block we can use for config.
  panic("not implemented");
}

static bool try_load_coef(void) {
  // load xpt_coef from flash! returns a bool if it succeeds or fails.
  // if it fails, we fallback to tpcal.
  panic("not implemented");
  return false;
}
