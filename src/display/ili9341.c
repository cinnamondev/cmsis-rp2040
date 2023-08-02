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

#include "hardware/spi.h"
#include "ili9341.h"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "lvgl.h"
#include "pico/time.h"

#include <stdint.h>
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
static void __not_in_flash_func(spi_generic_write8_read16_blk)(spi_inst_t *spi, const uint8_t *src, uint16_t *dst);

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
lv_disp_drv_t* lv_ili9341_init(void) {
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

  return &LV_ILI_DRV;
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
lv_indev_drv_t* lv_xpt2046_init(void) {
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
  return &LV_XPT_INDEV_DRV;
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
  cmd |= 0x80;
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
  // we have written ourselves into a pickle here. for each byte sent in 8 bit,
  // we read 1 byte... so theory says we can write to 16 bit with a cast- but then
  // we will advance out the bounds of the 16 bit value as the word size is 32 bit.
  // in order to do this with 8 bit, we need to store a RESLOW & RESHI and then
  // push that to output. for now... we enable 16 bit mode temporarily, as this is
  // polled relatively infrequently.

  uint16_t _cmd = cmd; // XXXX XXXX 0000 0000 (zeros are read portion)
  _cmd |= 0x80;
  SPI_WAIT_FREE();
  spi_set_baudrate(spi_cfg.iface, SPEED_XPT);
  spi_set_format(spi_cfg.iface, 16, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  XPT_SEL();
  //spi_generic_write8_read16_blk(spi_cfg.iface, &cmd, output);
  //spi_write_read_blocking(spi_inst_t *spi, , (uint8_t*)&output, 2)
  spi_write16_read16_blocking(spi_cfg.iface, &_cmd, output, 1);
  *output = *output >> 4;
  XPT_DESEL();  
  spi_set_baudrate(spi_cfg.iface, SPEED_ILI);
  spi_set_format(spi_cfg.iface, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
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
  xpt2046_deaf_cmd(XPT_MUX_Z1 | XPT_SET_PB0 | XPT_SET_PB1);
  xpt2046_cmd(XPT_MUX_Z1 | XPT_SET_PB0 | XPT_SET_PB1, &z1);
  xpt2046_cmd(XPT_MUX_Z2 | XPT_SET_PB0 | XPT_SET_PB1, &z2);
#ifdef DEBUG
  printf("touched? z1 %d z2 %d expr. %d t %d %d \n", z1, z2, (z1 - z2), (z1-z2)<400, (z1-z2)>600);
#endif
  //return true;
  return (z1-z2)<400 || (z1-z2)>600;
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
  //xpt2046_deaf_cmd(XPT_MUX_X | XPT_SET_PB0 | XPT_SET_PB1);
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
  /*Save the pressed coordinates and the state*/
  if (xpt2046_touch()) {
    xpt2046_xyz(&x,&y);
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
  // If released, the coordinates will remain at the last pressed point.
  data->point.x = x;
  data->point.y = y;
}
