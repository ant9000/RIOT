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
#include "ata8510_registers.h"

void ata8510_reg_write(const ata8510_t *dev,
                         const uint8_t addr,
                         const uint8_t value)
{
}

uint8_t ata8510_reg_read(const ata8510_t *dev, const uint8_t addr)
{
    return 0;
}

void ata8510_sram_read(const ata8510_t *dev,
                         const uint8_t offset,
                         uint8_t *data,
                         const size_t len)
{
}

void ata8510_sram_write(const ata8510_t *dev,
                          const uint8_t offset,
                          const uint8_t *data,
                          const size_t len)
{
}

void ata8510_fb_start(const ata8510_t *dev)
{
}

void ata8510_fb_read(const ata8510_t *dev,
                       uint8_t *data,
                       const size_t len)
{
}

void ata8510_fb_stop(const ata8510_t *dev)
{
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
