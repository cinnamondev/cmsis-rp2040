

// https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf

/*********************
 *      INCLUDES
 *********************/
#include "ili9341.h"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/regs/intctrl.h"
#include "hardware/spi.h"
#include "lvgl.h"
#include "perf_counter.h"
#include "pico/platform.h"
#include "pico/time.h"
#include "src/hal/lv_hal_disp.h"
#include "src/misc/lv_color.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

/*********************
 *      DEFINES
 *********************/
#define BUFFER_SIZE (int)(ILI9341_HOR * ILI9341_VER * 0.1)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
// static void irq0_dma_flush_ready();

static void ili_flush(lv_disp_drv_t *disp, const lv_area_t *area,
                      lv_color_t *px_map);

static inline void ili_write_byte(uint8_t payload);

static inline void ili_write(void *payload, uint16_t len, bool dc);
static inline void ili_write_dat(void *payload, uint16_t len);

static void ili_addr_win(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

static inline void ili_write_hword(uint16_t *payload);

static inline void ili_write_hwords(uint16_t *payload, int len);

static void irq1_dma_flush_ready();

/**********************
 *  STATIC VARIABLES
 **********************/

// initial state
static uint16_t ILI_CUR_HOR = ILI9341_HOR;
static uint16_t ILI_CUR_VER = ILI9341_VER;

// DMA channel for background writing frames
static uint DMA_TX;
static dma_channel_config DMA_TX_CFG;

static lv_disp_drv_t driver;
// lvgl internal buffer
static lv_disp_draw_buf_t lv_draw_buf_dsc;
static lv_color_t lv_buf_1[BUFFER_SIZE];
static lv_color_t lv_buf_2[BUFFER_SIZE];

static struct ili9341_cfg_t ili_cfg = {
    // Default assignments:
    .iface = spi1, // Use SPI1 interface
    .rx = 12,      // SPI_RX / MISO
    .tx = 11,      // SPI_TX / MOSI
    .sck = 10,     // SPI_SCK / SPI CLOCK
    .dc = 15,      // DC (data select): low for dat high reg.
    .cs = 13,      // Chip select, active low.
    .resx = 14,    // Chip reset, active low.
};                 // change prior to lv_ili9341_init
                   // to use different assignments.
// pin definitions

/**********************
 *      MACROS
 **********************/

// CSH is 40ns. if you have a faster (up to 133mhz) clock, then this MIGHT
// be too little. If you are running at a low clock, then this might be too
// sloppy.

#define CS_SELECT()                                                            \
  asm volatile("nop\nnop\nnop\nnop\nnop");                                     \
  gpio_put(ili_cfg.cs, 0);                                                     \
  asm volatile("nop\nnop\nnop\nnop\nnop")

#define CS_DESELECT()                                                          \
  asm volatile("nop\nnop\nnop\nnop\nnop");                                     \
  gpio_put(ili_cfg.cs, 1);                                                     \
  asm volatile("nop\nnop\nnop\nnop\nnop")

#define SPI_WAIT_FREE()                                                        \
  while (spi_is_busy(ili_cfg.iface)) {                                         \
  }

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**** COMMAND/SPI *****/

/**
 * @brief Sends a command to the ILI9341.
 *
 * @param cmd Command to send
 */
void ili9341_cmd(uint8_t cmd) {
  SPI_WAIT_FREE();
  gpio_put(ili_cfg.dc, 0); // cmd mode
  ili_write_byte(cmd);
  gpio_put(ili_cfg.dc, 1); // data mode (initial)
}

/**
 * @brief Sends a "parameter" to the chip.
 *
 * @param param parameter to send.
 */
void ili9341_param(uint8_t param) {
  SPI_WAIT_FREE();
  ili_write_byte(param);
}

/**
 * @brief send a command and its parameter
 *
 * @param cmd command to send
 * @param param parameter to send
 */
inline void ili9341_cmd_p(uint8_t cmd, uint8_t param) {
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
  CS_SELECT();
  spi_write_blocking(ili_cfg.iface, (uint8_t *)params, len);
  CS_DESELECT();
}

/**** DRAWING     *****/

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

  // prevent OOB (negative or >=240/320 coords.)
  uint16_t x1 = _x1 < 0 ? 0 : _x1;
  uint16_t y1 = _y1 < 0 ? 0 : _y1;
  uint16_t x2 = _x2 > ILI9341_HOR - 1 ? ILI9341_HOR - 1 : _x2;
  uint16_t y2 = _y2 > ILI9341_VER - 1 ? ILI9341_VER - 1 : _y2;
  SPI_WAIT_FREE();
  ili_addr_win(x1, y1, x1, y1);
  CS_SELECT();
  // Set new write address and send data off
  dma_channel_configure(DMA_TX, &DMA_TX_CFG, 
                        &spi_get_hw(ili_cfg.iface)->dr,
                        bitmap, 
                        (x2-x1+1) * (y2-y1+1) * 2,
                        false); // start asap
  dma_channel_set_irq1_enabled(DMA_TX, true);
  dma_channel_start(DMA_TX);
}

void ili9341_pixel(int x, int y, uint16_t color) {
  ili_addr_win(x, y, x + 1, y + 1);
  ili_write_hword(&color);
}

void configure_ili9341(struct ili9341_cfg_t cfg) { ili_cfg = cfg; }

void ili9341_hw_res() {
  sleep_ms(10);
  gpio_put(ili_cfg.resx, 0);
  sleep_us(20);
  gpio_put(ili_cfg.resx, 1);
  sleep_ms(10);
}
/**
 * @brief Rotate the display to a specific orientation
 *
 * @param r Orientation to change to. `F` indicated the display is flipped
 * along the new axis.
 */
void ili9341_rotate(enum rotation_t r) {
  uint8_t madctr = MADCTL_BGR | r;
  if (r & MADCTL_MV | r == 0) { // off-axis?
    ILI_CUR_HOR = ILI9341_HOR;  // parr. to initial
    ILI_CUR_VER = ILI9341_VER;
  } else {
    ILI_CUR_HOR = ILI9341_VER; // norm. to initial
    ILI_CUR_VER = ILI9341_HOR;
  }
  ili9341_cmd_p(ILI9341_MAC, madctr);
}

/**
 * @brief Creates a lv_disp_drv_t
 *
 */
void lv_ili9341_init(void) {
  lv_disp_drv_init(&driver);
  DMA_TX = dma_claim_unused_channel(true);

  /*-------------------------
   * Initialize your display
   * -----------------------*/
  ili9341_init();

  /*------------------------------------
   * Create a display and set a flush_cb
   * -----------------------------------*/
  driver.hor_res = ILI9341_HOR;
  driver.ver_res = ILI9341_VER;

  // Buffer will be flushed to DMA here.
  driver.flush_cb = ili_flush;
  /* Example
   * Two buffers for partial rendering
   * In flush_cb DMA or similar hardware should be used to update the display in
   * the background.*/

  // Setup draw buffer with partial rendering
  lv_disp_draw_buf_init(&lv_draw_buf_dsc, lv_buf_1,
                        lv_buf_2, // lv_buf_2,
                        BUFFER_SIZE);
  driver.draw_buf = &lv_draw_buf_dsc;
  lv_disp_drv_register(&driver);
  // interrupts from dma will go to irq1_dma_flush_ready
  dma_channel_set_irq1_enabled(DMA_TX, true);
  irq_set_exclusive_handler(DMA_IRQ_1, irq1_dma_flush_ready);
  irq_set_enabled(DMA_IRQ_1, true);
}

/**
 * @brief Initialize the ILI9341 chip, according to ili_cfg.
 *
 * Sets up all pins connecting the RP2040 to the ILI9341, and sets up Direct
 * Memory Access between the chip and pico. IRQ1 must be available as an
 * exclusive interrupt.
 */
void ili9341_init(void) {
  // Initialize ports
  // printf("ffiewifgi")
  gpio_set_function(ili_cfg.rx, GPIO_FUNC_SPI);
  gpio_set_function(ili_cfg.tx, GPIO_FUNC_SPI);
  gpio_set_function(ili_cfg.sck, GPIO_FUNC_SPI);

  uint speed = spi_init(ili_cfg.iface, 62.5e6); // 16 bit spi
  // WE'RE RUNNING THIS BABY AT A MILLION MPH!
#ifdef DEBUG
  printf("spi speed %u", speed);
#endif
  spi_set_format(ili_cfg.iface, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

  // config CS (active low)
  gpio_init(ili_cfg.cs);
  gpio_set_dir(ili_cfg.cs, GPIO_OUT);
  gpio_put(ili_cfg.cs, 1);

  // config DC (data lo)
  gpio_init(ili_cfg.dc);
  gpio_set_dir(ili_cfg.dc, GPIO_OUT);
  gpio_put(ili_cfg.dc, 1);
  // config resx (active low)
  gpio_init(ili_cfg.resx);
  gpio_set_dir(ili_cfg.resx, GPIO_OUT);
  gpio_put(ili_cfg.resx, 1);

  // setup DMA for TX.
  DMA_TX = dma_claim_unused_channel(true);
  DMA_TX_CFG = dma_channel_get_default_config(DMA_TX);
  channel_config_set_transfer_data_size(&DMA_TX_CFG, DMA_SIZE_8);
  channel_config_set_dreq(&DMA_TX_CFG, spi_get_dreq(ili_cfg.iface, true));

  spi_get_hw(ili_cfg.iface);
  // Hard & soft reset ILI.
  ili9341_hw_res();
  ili9341_cmd(ILI9341_SWRESET);
  sleep_ms(130); // wait another 5ms following sw reset.
  ili9341_cmd(ILI9341_DISP_OFF);

  // power control
  ili9341_cmd_p(ILI9341_PWC1, 0x23);

  ili9341_cmd_p(ILI9341_PWC2, 0x10);
  // vcomc1 control
  ili9341_cmd_mparam(ILI9341_VCOMC1, 2, (uint8_t[2]){0x3e, 0x28});
  // vcomc2 c
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

  ili9341_cmd_mparam(ILI9341_DISP_FNCC, 3, (uint8_t[3]){0x08, 0x82, 0x27});

  ili9341_cmd(ILI9341_W_MEM);
  sleep_ms(150);

  ili9341_cmd(ILI9341_SLEEP_OUT);
  sleep_ms(150);
  ili9341_cmd(ILI9341_DISP_ON);
  sleep_ms(150);
  /*
  // page addr set
  ili9341_cmd_params(ILI9341_PGE_F_S,4,
      0x00,
      0x00,
      (ILI9341_VER >> 8),
      (ILI9341_VER & 0xff)    // end = width - 1
  );
  // column addr set
  ili9341_cmd_params(ILI9341_COL_F_S,4,
      0x00,
      0x00,
      0x00,
      ILI9341_HOR - 1 // end column = height -1
  );
  ili9341_cmd(ILI9341_W_MEM);
  */
  gpio_put(ili_cfg.dc, 1); // ensure data mode for img writ.
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * @brief Interrupt call - tells lvgl the flush is done when DMA has sent all
 * it's data.
 */
static void irq1_dma_flush_ready() {
  // prevent being stuck in IRQ
  dma_channel_acknowledge_irq1(DMA_TX);
  // sleep_us(10);
  CS_DESELECT();
  lv_disp_flush_ready(&driver);
}

/**
 * @brief Write a byte to the ILI9341.
 *
 * @param payload 8 bit sequence to write
 */
static inline void ili_write_byte(uint8_t payload) {
  CS_SELECT();
  spi_write_blocking(ili_cfg.iface, &payload, 1);
  CS_DESELECT();
}

static inline void ili_write_bytes(uint8_t *payload, int len) {
  CS_SELECT();
  spi_write_blocking(ili_cfg.iface, payload, len);
  CS_DESELECT();
}

static inline void ili_write_hword(uint16_t *payload) {
  CS_SELECT();
  spi_write_blocking(ili_cfg.iface, (uint8_t *)payload, 2);
  CS_DESELECT();
}

static inline void ili_write_hwords(uint16_t *payload, int len) {
  CS_SELECT();
  spi_write_blocking(ili_cfg.iface, (uint8_t *)payload, 2 * len);
  CS_DESELECT();
}

/**
 * @brief Set column address start and page address start to occupy
 * part of the display.
 *
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 */
static void ili_addr_win(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  ili9341_cmd_mparam(
      ILI9341_COL_F_S, 4,
      (uint8_t[4]){(x0 >> 8), (x0 & 0xff), (x1 >> 8), (x1 & 0xff)});
  ili9341_cmd_mparam(
      ILI9341_PGE_F_S, 4,
      (uint8_t[4]){(y0 >> 8), (y0 & 0xff), (y1 >> 8), (y1 & 0xff)});
  ili9341_cmd(ILI9341_W_MEM); // we can start sending image data
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called
 * by LVGL
 */
void ili9341_enable_update(void) { disp_flush_enabled = true; }

/* Disable updating the screen (the flushing process) when disp_flush() is
 * called by LVGL
 */
void ili9341_disable_update(void) { disp_flush_enabled = false; }

/**
 * @brief flush the incoming buffer `px_map` for `area` to the display via DMA.
 *
 * Flush finishes on interrupt
 *
 * @param disp
 * @param area
 * @param px_map
 */

static void ili_flush(lv_disp_drv_t *disp, const lv_area_t *area,
                      lv_color16_t *px_map) {
  if (!disp_flush_enabled || area->x2 < 0 || area->y2 < 0 ||
      area->x1 > (ILI9341_HOR - 1) || area->y1 > (ILI9341_VER - 1)) {
    lv_disp_flush_ready(disp); // prevent dodgy input
    return;
  }
  // prevent OOB (negative or >=240/320 coords.)
  uint16_t x1 = area->x1 < 0 ? 0 : area->x1;
  uint16_t y1 = area->y1 < 0 ? 0 : area->y1;
  uint16_t x2 = area->x2 > ILI9341_HOR - 1 ? ILI9341_HOR - 1 : area->x2;
  uint16_t y2 = area->y2 > ILI9341_VER - 1 ? ILI9341_VER - 1 : area->y2;

  SPI_WAIT_FREE();
  ili_addr_win(x1, y1, x2, y2);
  // todo: hardware rot ensure.
  CS_SELECT();
  dma_channel_configure(DMA_TX, &DMA_TX_CFG,
                        &spi_get_hw(ili_cfg.iface)->dr, // write address
                        px_map,                         // read address
                        (x2 - x1 + 1) * (y2 - y1 + 1) *
                            2,  //(x2-x1)*(y2-y1)*2,// element count (each
                                //element is of size transfer_data_size)
                        false); // start asap

  dma_channel_set_irq1_enabled(DMA_TX, true);
  dma_channel_start(DMA_TX);

  // This will be called by an interrupt when DMA has flushed.
  // lv_disp_flush_ready(disp);
}
