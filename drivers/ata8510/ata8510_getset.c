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
 * @brief       Getter and setter functions for the ATA8510 driver
 *
 * @author      Antonio Galea <antonio.galea@gmail.com>
 *
 * @}
 */

#include "ata8510.h"
#include "ata8510_internal.h"
#include "ata8510_registers.h"
#include "periph/spi.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

uint16_t ata8510_get_addr_short(ata8510_t *dev)
{
    return (dev->netdev.short_addr[0] << 8) | dev->netdev.short_addr[1];
}

void ata8510_set_addr_short(ata8510_t *dev, uint16_t addr)
{
    dev->netdev.short_addr[0] = (uint8_t)(addr);
    dev->netdev.short_addr[1] = (uint8_t)(addr >> 8);
#ifdef MODULE_SIXLOWPAN
    /* https://tools.ietf.org/html/rfc4944#section-12 requires the first bit to
     * 0 for unicast addresses */
    dev->netdev.short_addr[0] &= 0x7F;
#endif
//    ata8510_reg_write(dev, ATA8510_REG__SHORT_ADDR_0,
//                        dev->netdev.short_addr[1]);
//    ata8510_reg_write(dev, ATA8510_REG__SHORT_ADDR_1,
//                        dev->netdev.short_addr[0]);
}

uint64_t ata8510_get_addr_long(ata8510_t *dev)
{
    uint64_t addr;
    uint8_t *ap = (uint8_t *)(&addr);
    for (int i = 0; i < 8; i++) {
        ap[i] = dev->netdev.long_addr[i];
    }
    return addr;
}

void ata8510_set_addr_long(ata8510_t *dev, uint64_t addr)
{
    for (int i = 0; i < 8; i++) {
        dev->netdev.long_addr[i] = (uint8_t)(addr >> (i * 8));
//        ata8510_reg_write(dev, (ATA8510_REG__IEEE_ADDR_0 + i),
//                            (addr >> ((7 - i) * 8)));
    }
}

uint8_t ata8510_get_chan(ata8510_t *dev)
{
    return 0;
}

void ata8510_set_chan(ata8510_t *dev, uint8_t channel)
{
}

uint8_t ata8510_get_page(ata8510_t *dev)
{
    return 0;
}

void ata8510_set_page(ata8510_t *dev, uint8_t page)
{
}

uint16_t ata8510_get_pan(ata8510_t *dev)
{
    return 0;
}

void ata8510_set_pan(ata8510_t *dev, uint16_t pan)
{
}

int16_t ata8510_get_txpower(ata8510_t *dev)
{
    return 0;
}

void ata8510_set_txpower(ata8510_t *dev, int16_t txpower)
{
}

uint8_t ata8510_get_max_retries(ata8510_t *dev)
{
    return 0;
}

void ata8510_set_max_retries(ata8510_t *dev, uint8_t max)
{
}

uint8_t ata8510_get_csma_max_retries(ata8510_t *dev)
{
    return 0;
}

void ata8510_set_csma_max_retries(ata8510_t *dev, int8_t retries)
{
}

void ata8510_set_csma_backoff_exp(ata8510_t *dev, uint8_t min, uint8_t max)
{
}

void ata8510_set_csma_seed(ata8510_t *dev, uint8_t entropy[2])
{
}

int8_t ata8510_get_cca_threshold(ata8510_t *dev)
{
    return 0;
}

void ata8510_set_cca_threshold(ata8510_t *dev, int8_t value)
{
}

void ata8510_set_option(ata8510_t *dev, uint16_t option, bool state)
{
}

void ata8510_set_state(ata8510_t *dev, uint8_t state)
{
}

void ata8510_reset_state_machine(ata8510_t *dev)
{
}
