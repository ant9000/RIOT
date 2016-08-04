/*
 * Copyright (C) 2016 Antonio Galea <antonio.galea@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ata8510
 * @{
 *
 * @file
 * @brief       Implementation of driver internal functions
 *
 * @author      Antonio Galea <antonio.galea@gmail.com>
 *
 * @}
 */

#include "periph/spi.h"
#include "periph/gpio.h"
#include "xtimer.h"
#include "ata8510_internal.h"

int ata8510_send_cmd(const ata8510_t *dev,
                          const uint8_t *tx_buffer,
                          const uint8_t *rx_buffer,
                          const size_t len)
{
    int count;
    spi_acquire(dev->params.spi);
    gpio_clear(dev->params.cs_pin);
    count = spi_transfer_bytes(
        dev->params.spi, (char *)tx_buffer, (char *)rx_buffer, len
    );
    gpio_set(dev->params.cs_pin);
    spi_release(dev->params.spi);
    return count;
}

void ata8510_power_on(const ata8510_t *dev){
    gpio_clear(dev->params.reset_pin);
    gpio_clear(dev->params.reset_pin);
    gpio_clear(dev->params.reset_pin);
    gpio_clear(dev->params.reset_pin);
    gpio_set(dev->params.reset_pin);
    gpio_set(dev->params.reset_pin);
    gpio_set(dev->params.reset_pin);
    gpio_set(dev->params.reset_pin);
    gpio_clear(dev->params.sleep_pin);
    // delay_ms(2);
}

uint8_t ata8510_get_status(const ata8510_t *dev)
{
    return 0;
}

void ata8510_assert_awake(ata8510_t *dev)
{
}

void ata8510_hardware_reset(ata8510_t *dev)
{
}

void ata8510_configure_phy(ata8510_t *dev)
{
}

void ata8510_force_trx_off(const ata8510_t *dev)
{
}
