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
 * @brief       Implementation of public functions for ATA8510 radio module
 *
 * @author      Antonio Galea <antonio.galea@gmail.com>
 *
 * @}
 */

#include "periph/cpuid.h"
#include "byteorder.h"
#include "net/ieee802154.h"
#include "net/gnrc.h"
#include "ata8510_registers.h"
#include "ata8510_internal.h"
#include "ata8510_netdev.h"

#define ENABLE_DEBUG (0)
#include "debug.h"


void ata8510_setup(ata8510_t *dev, const ata8510_params_t *params)
{
}

void ata8510_reset(ata8510_t *dev)
{
}

bool ata8510_cca(ata8510_t *dev)
{
    return false;
}

size_t ata8510_send(ata8510_t *dev, uint8_t *data, size_t len)
{
    return 0;
}

void ata8510_tx_prepare(ata8510_t *dev)
{
}

size_t ata8510_tx_load(ata8510_t *dev, uint8_t *data,
                         size_t len, size_t offset)
{
    return 0;
}

void ata8510_tx_exec(ata8510_t *dev)
{
}
