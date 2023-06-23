#include "hardware/spi.h"


/**
 * @brief Describes an SPI interface.
 * 
 * ILI and XPT have the same bus but different speeds, so we need to have
 * seperate configs
 */
struct spi_device_t {
    uint8_t rx;
    uint8_t tx;
    uint8_t sck;
    uint8_t cs;
    uint8_t dc;
    uint8_t resx;
    spi_inst_t* iface;
    uint speed;
} SPI_DEFAULT = {
    .rx   = 12,
    .tx   = 11,
    .sck  = 10,
    .dc   = (uint8_t)NULL,
    .cs   = (uint8_t)NULL,
    .resx = (uint8_t)NULL,
    .iface=spi1,
    .speed=2.5e6
};