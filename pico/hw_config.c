#include "hw_config.h"

static spi_t spi = {
    .hw_inst = spi0,
    .sck_gpio = 2,
    .mosi_gpio = 3,
    .miso_gpio = 4,
    .baud_rate = 5 * 1000 * 1000
};

static sd_spi_if_t spi_if = {
    .spi = &spi,
    .ss_gpio = 5
};

static sd_card_t sd_card = {
    .type = SD_IF_SPI,
    .spi_if_p = &spi_if
};

size_t sd_get_num() {
    return 1;
}

sd_card_t *sd_get_by_num(size_t num) {
    if (num == 0) {
        return &sd_card;
    }

    return NULL;
}