

// https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf

/*********************
 *      INCLUDES
 *********************/
#include "ili9341.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/regs/intctrl.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "lvgl.h"
#include "perf_counter.h"
#include "pico/time.h"
#include "src/hal/lv_hal_disp.h"
#include "src/misc/lv_color.h"

/*********************
 *      DEFINES
 *********************/
// commands [8.1]
// Standard command set (Level1  commands)
#define ILI9341_NOP         0x00    // no operation
#define ILI9341_SWRESET     0x01    // software reset
#define ILI9341_R_ID        0x04    // display id information
#define ILI9341_R_STATUS    0x09    // display status
#define ILI9341_R_DPWRM     0x0A    // display power mode
#define ILI9341_R_MADCTL    0x0B    // read matctl
#define ILI9341_R_PIXELF    0x0C    // read pixel format
#define ILI9341_R_IMGF      0x0D    // read image format
#define ILI9341_R_SIGNALM   0x0E    // read signal mode
#define ILI9341_R_DDIAGRES  0x0F    // self diagnostic result
#define ILI9341_SLEEP       0x10    // sleep mode
#define ILI9341_SLEEP_OUT   0x11    // wake from sleep
#define ILI9341_PARTIAL_ON  0x12    // partial mode on
#define ILI9341_NORMDISP_ON 0x13    // normal display mode on
#define ILI9341_INVERT_OFF  0x20    // inversion off
#define ILI9341_INVERT_ON   0x21    // inversion on
#define ILI9341_GAMMA       0x26    // gamma set
#define ILI9341_DISP_OFF    0x28    // display off
#define ILI9341_DISP_ON     0x29    // display on
#define ILI9341_COL_F_S     0x2A    // column address set
#define ILI9341_PGE_F_S     0x2B    // page address set
#define ILI9341_W_MEM       0x2C    // memory write
#define ILI9341_CLR_SET     0x2D    // color set
#define ILI9341_R_MEM       0x2E    // memory read
#define ILI9341_PAREA       0x30    // partial area
#define ILI9341_VSCR_DEF    0x33    // vertical scrolling definiton
#define ILI9341_TEAR_OFF    0x34    // tearing effect line off
#define ILI9341_TEAR_ON     0x35    // tearing effect line on
#define ILI9341_MAC         0x36    // memory address control
#define ILI9341_VSCR_S_F    0x37    // vertical scrolling start address
#define ILI9341_IDLE_OFF    0x38    // idle mode off
#define ILI9341_IDLE_ON     0x39    // idle mode on
#define ILI9341_PIXEL_FMT   0x3A    // set pixel format
#define ILI9341_W_MEM_C     0x3C    // write memory continue
#define ILI9341_R_MEM_C     0x3E    // read memory continue
#define ILI9341_TEAR_SC_S   0x44    // set tear scanline
#define ILI9341_SCAN_GET    0x45    // get scanline
#define ILI9341_W_BRIGHT    0x51    // write display brightness
#define ILI9341_R_BRIGHT    0x52    // read display brightness
#define ILI9341_W_CTRLD     0x53    // write ctrl display
#define ILI9341_R_CTRLD     0x54    // read ctrl display
#define ILI9341_W_CABC      0x55    // write content adaptive brightness control
#define ILI9341_R_CABC      0x56    // read content adaptive brightness control
#define ILI9341_W_CABC_MBR  0x5E    // write cabc min brightness
#define ILI9341_R_CABC_MBDR 0x5F    // read cabc min brightness
#define ILI9341_ID1         0xDA    // read id1
#define ILI9341_ID2         0xDB    // read id2
#define ILI9341_ID3         0xDC    // read id3
// EXTENDED COMMAND SET (Level 2 commands)
#define ILI9341_RGB_IFRSCT  0xB0
#define ILI9341_FRAMEC_NORM 0xB1
#define ILI9341_FRAMEC_IDLE 0xB2
#define ILI9341_FRAMEC_PART 0xB3
#define ILI9341_INV_CONT    0xB4
#define ILI9341_BPRCH_CONT  0xB5
#define ILI9341_DISP_FNCC   0xB6
#define ILI9341_ENT_MODE_S  0xB7
#define ILI9341_BL1         0xB8
#define ILI9341_BL2         0xB9
#define ILI9341_BL3         0xBA
#define ILI9341_BL4         0xBB
#define ILI9341_BL5         0xBC
#define ILI9341_BL7         0xBE
#define ILI9341_BL8         0xBF
#define ILI9341_PWC1        0xC0
#define ILI9341_PWC2        0xC1
#define ILI9341_VCOMC1      0xC5
#define ILI9341_VCOMC2      0xC7
#define ILI9341_NV_MW       0xD0
#define ILI9341_NV_MPK      0xD1
#define ILI9341_NV_MSR      0xD2
#define ILI9341_ID4         0xD3
#define ILI9341_PGMMA_COR   0xE0
#define ILI9341_NGMMA_COR   0xE1
#define ILI9341_DGC1        0xE2
#define ILI9341_DGC2        0xE3
#define ILI9341_IFC         0xF6

// extend register command
#define ILI9341_PWRA        0xCB
#define ILI9341_PWRB        0xCF
#define ILI9341_DTCA        0xE8
#define ILI9341_DTCAEXT     0xF9
#define ILI9341_DTCB        0xEA
#define ILI9341_POSEQC      0xED
#define ILI9341_E3G         0xF2
#define ILI9341_PUMPRATIO   0xF7


#define ILI9341_HOR 320
#define ILI9341_VER 240
#define BUFFER_SIZE (int)(ILI9341_HOR*ILI9341_VER*0.1)




/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
//static void irq0_dma_flush_ready();

static void ili_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * px_map);

static inline void ili_write_byte(uint8_t payload);

static inline void ili_write(void* payload, uint16_t len, bool dc);
static inline void ili_write_dat(void *payload, uint16_t len);

static void ili_addr_win(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);


static inline void ili_write_hword(uint16_t* payload);

static inline void ili_write_hwords(uint16_t* payload, int len);

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

static struct ili9341_cfg_t ili_cfg = {    // Default assignments:
    .iface = spi0,                  // Use SPI1 interface
    .rx  = 4,                       // SPI_RX / MISO
    .tx  = 3,                       // SPI_TX / MOSI
    .sck = 2,                       // SPI_SCK / SPI CLOCK
    .dc  = 7,                       // DC (data select): low for dat high reg.
    .cs  = 5,                       // Chip select, active low.
    .resx= 6,                       // Chip reset, active low.
    .crot= R90F,
};                                  // change prior to lv_ili9341_init
                                    // to use different assignments.

// pin definitions


/**********************
 *      MACROS
 **********************/

#define CS_SELECT()     asm volatile("nop\nnop\nnop");\
                        gpio_put(ili_cfg.cs,0);\
                        asm volatile("nop\nnop\nnop")


#define CS_DESELECT()   asm volatile("nop\nnop\nnop");\
                        gpio_put(ili_cfg.cs,1);\
                        asm volatile("nop\nnop\nnop")

#define SPI_WAIT_FREE() while(spi_is_busy(ili_cfg.iface)) {}

#define SET_TRANSFER_HWORD() spi_set_format(ili_cfg.iface, 16, SPI_CPOL_1,SPI_CPHA_1,SPI_MSB_FIRST);
#define SET_TRANSFER_BYTE() spi_set_format(ili_cfg.iface, 8, SPI_CPOL_1,SPI_CPHA_1,SPI_MSB_FIRST);



/**********************
 *   GLOBAL FUNCTIONS
 **********************/

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
 * @brief Simultaneously sends a command and its subsequent parameters.
 * 
 * @param cmd   Command to execute
 * @param n     Number of parameters to send
 * @param ...   Parameters to send (sequential)
 */
void ili9341_cmd_params(uint8_t cmd, int n,...) {
    va_list args;
    va_start(args, n);
    SPI_WAIT_FREE();
    ili9341_cmd(cmd);

    CS_SELECT();
    for(int i=0;i<n;i++) {
        spi_write_blocking(ili_cfg.iface, va_arg(args, int), 1);
    }
    CS_DESELECT();
    va_end(args);
}

void ili9341_cmd_mparam(uint8_t cmd, int len, uint8_t* params) {
    ili9341_cmd(cmd);
    CS_SELECT();
    spi_write_blocking(ili_cfg.iface, (uint8_t*)params, len);
    CS_DESELECT();
}

void ili9341_bmp(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t *bitmap) {
    ili_addr_win(x0,y0,x1,y1);
    SET_TRANSFER_HWORD();
    gpio_put(ili_cfg.dc,1);
    CS_SELECT();
    spi_write16_blocking(ili_cfg.iface, bitmap, (x1-x0)*(y1-y0));
    CS_DESELECT();
    gpio_put(ili_cfg.dc,0);
    SET_TRANSFER_BYTE();
}

void ili9341_pixel(int x, int y, uint16_t color) {
    ili_addr_win(x, y,x+1,y+1);
    ili_write_hword(&color);
}


void configure_ili9341(struct ili9341_cfg_t cfg) {
    ili_cfg = cfg;
}

void ili9341_hw_res() { 
    sleep_ms(10);
    gpio_put(ili_cfg.resx,0);
    sleep_us(20);
    gpio_put(ili_cfg.resx,1);
    sleep_ms(10);
}
/**
 * @brief Rotate the display to a specific orientation
 * 
 * @param r Orientation to change to. `F` indicated the display is flipped
 * along the new axis.
 */
void ili9341_rotate(enum rotation_t r) {
    uint8_t madctr = MADCTL_RGB | r;
    if (r & MADCTL_MV | r==0) {            // off-axis?
        ILI_CUR_HOR=ILI9341_HOR;    // parr. to initial
        ILI_CUR_VER=ILI9341_VER;
    } else {
        ILI_CUR_HOR=ILI9341_VER;   // norm. to initial
        ILI_CUR_VER=ILI9341_HOR;
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
     * In flush_cb DMA or similar hardware should be used to update the display in the background.*/
    
    // Setup draw buffer with partial rendering
    lv_disp_draw_buf_init(&lv_draw_buf_dsc, 
        lv_buf_1, 
        lv_buf_2, //lv_buf_2, 
        BUFFER_SIZE
    );
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
    //printf("ffiewifgi")
    gpio_set_function(ili_cfg.rx, GPIO_FUNC_SPI);
    gpio_set_function(ili_cfg.tx, GPIO_FUNC_SPI);
    gpio_set_function(ili_cfg.sck, GPIO_FUNC_SPI);

    spi_init(ili_cfg.iface, 10e6);     // 16 bit spi
    SET_TRANSFER_BYTE();

    // config CS (active low)
    gpio_init(ili_cfg.cs);
    gpio_set_dir(ili_cfg.cs, GPIO_OUT);
    gpio_put(ili_cfg.cs,1);

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
    channel_config_set_dreq(&DMA_TX_CFG,
        spi_get_dreq(ili_cfg.iface, true)
    );  

    spi_get_hw(ili_cfg.iface);
    // Hard & soft reset ILI.
    ili9341_hw_res();
    ili9341_cmd(ILI9341_SWRESET);
    sleep_ms(130); // wait another 5ms following sw reset.
    ili9341_cmd(ILI9341_DISP_OFF);

    // power control
    ili9341_cmd_p(ILI9341_PWC1,0x23);
    
    ili9341_cmd_p(ILI9341_PWC2, 0x10);
    // vcomc1 control
    ili9341_cmd_mparam(ILI9341_VCOMC1, 2, (uint8_t[2]){
        0x3e,0x28
    });
    // vcomc2 c
    ili9341_cmd_p(ILI9341_VCOMC2, 0x86);

    // mac / madctl
    ili9341_rotate(ili_cfg.crot);

    ili9341_cmd_p(ILI9341_PIXEL_FMT, 0x55); // 16b/pixel (rgb565)

    ili9341_cmd_mparam(ILI9341_FRAMEC_NORM,2, (uint8_t[2]){ 
        0x00,       // 70 hz
        0x1B
    }); 

    ili9341_cmd_p(0xf2,0x00);

    ili9341_cmd_p(ILI9341_GAMMA, 0x01);

    // positive gamma
    ili9341_cmd_mparam(ILI9341_PGMMA_COR, 15, (uint8_t[15]){
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
    ili9341_cmd_mparam(ILI9341_NGMMA_COR, 15, (uint8_t[15]){
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
    
    ili9341_cmd_mparam(ILI9341_DISP_FNCC, 3, (uint8_t[3]){
        0x08, 0x82, 0x27
    });

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
    dma_channel_set_irq1_enabled(DMA_TX, false);
    gpio_put(25, 1);
    sleep_ms(200);
    gpio_put(25, 0);
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

static inline void ili_write_bytes(uint8_t* payload, int len) {
    CS_SELECT();
    spi_write_blocking(ili_cfg.iface, payload, len);
    CS_DESELECT();
}

static inline void ili_write_hword(uint16_t* payload) {
    CS_SELECT();
    spi_write_blocking(ili_cfg.iface, (uint8_t*)payload, 1);
    CS_DESELECT();
}

static inline void ili_write_hwords(uint16_t* payload, int len) {
    CS_SELECT();
    spi_write_blocking(ili_cfg.iface, (uint8_t*)payload, 2*len);
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
    ili9341_cmd_mparam(ILI9341_COL_F_S, 4, (uint8_t[4]){
        (x0>>8),
        (x0&0xff),
        (x1>>8),
        (x1&0xff)
    });
    ili9341_cmd_mparam(ILI9341_PGE_F_S, 4, (uint8_t[4]){
        (y0>>8),
        (y0&0xff),
        (y1>>8),
        (y1&0xff)
    });
    ili9341_cmd(ILI9341_W_MEM); // we can start sending image data

}



volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void ili9341_enable_update(void) {
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void ili9341_disable_update(void) {
    disp_flush_enabled = false;
}

/**
 * @brief flush the incoming buffer `px_map` for `area` to the display via DMA.
 * 
 * Flush finishes on interrupt
 * 
 * @param disp 
 * @param area 
 * @param px_map 
 */

static void ili_flush(lv_disp_drv_t *disp,const lv_area_t *area,lv_color16_t *px_map) {
    if(!disp_flush_enabled || area->x2 < 0 || area->y2 < 0 || area->x1 > (ILI9341_HOR - 1) || area->y1 > (ILI9341_VER - 1)) {
        lv_disp_flush_ready(disp);  // prevent dodgy input
        return;
    }
    // prevent OOB (negative or >=240/320 coords.)
    uint16_t x1 = area->x1 < 0 ? 0 : area->x1;
    uint16_t y1 = area->y1 < 0 ? 0 : area->y1;
    uint16_t x2 = area->x2 > ILI9341_HOR - 1 ? ILI9341_HOR - 1 : area->x2;
    uint16_t y2 = area->y2 > ILI9341_VER - 1 ? ILI9341_VER - 1 : area->y2;

    SPI_WAIT_FREE();
    ili_addr_win(x1,y1,x2,y2);
    // todo: hardware rot ensure.
    CS_SELECT();
    dma_channel_configure(DMA_TX, &DMA_TX_CFG,
        &spi_get_hw(ili_cfg.iface)->dr, // write address
		px_map,						// read address
		(x2-x1+1)*(y2-y1+1)*2,//(x2-x1)*(y2-y1)*2,// element count (each element is of size transfer_data_size)
		false
    );						// start asap

    dma_channel_set_irq1_enabled(DMA_TX, true);
    dma_channel_start(DMA_TX);
    
    // This will be called by an interrupt when DMA has flushed.
    //lv_disp_flush_ready(disp);
}

