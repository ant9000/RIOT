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
#include "ata8510_internal.h"
#include "ata8510_netdev.h"

#define ENABLE_DEBUG (0)
#include "debug.h"


void ata8510_setup(ata8510_t *dev, const ata8510_params_t *params)
{
    netdev2_t *netdev = (netdev2_t *)dev;

    netdev->driver = &ata8510_driver;
    /* initialize device descriptor */
    memcpy(&dev->params, params, sizeof(ata8510_params_t));
    dev->idle_state = ATA8510_STATE_IDLE;
    dev->state = ATA8510_STATE_IDLE;
    dev->pending_tx = 0;
    /* initialise SPI */
    spi_init_master(dev->params.spi, SPI_CONF_FIRST_RISING, params->spi_speed);
}

void ata8510_reset(ata8510_t *dev)
{
#if CPUID_LEN
/* make sure that the buffer is always big enough to store a 64bit value */
#   if CPUID_LEN < IEEE802154_LONG_ADDRESS_LEN
    uint8_t cpuid[IEEE802154_LONG_ADDRESS_LEN];
#   else
    uint8_t cpuid[CPUID_LEN];
#endif
    eui64_t addr_long;
#endif

    ata8510_hardware_reset(dev);

    /* Reset state machine to ensure a known state */
    ata8510_reset_state_machine(dev);

    /* reset options and sequence number */
    dev->netdev.seq = 0;
    dev->netdev.flags = 0;
    /* set short and long address */
#if CPUID_LEN
    /* in case CPUID_LEN < 8, fill missing bytes with zeros */
    memset(cpuid, 0, CPUID_LEN);

    cpuid_get(cpuid);

#if CPUID_LEN > IEEE802154_LONG_ADDRESS_LEN
    for (int i = IEEE802154_LONG_ADDRESS_LEN; i < CPUID_LEN; i++) {
        cpuid[i & 0x07] ^= cpuid[i];
    }
#endif

    /* make sure we mark the address as non-multicast and not globally unique */
    cpuid[0] &= ~(0x01);
    cpuid[0] |= 0x02;
    /* copy and set long address */
    memcpy(&addr_long, cpuid, IEEE802154_LONG_ADDRESS_LEN);
    ata8510_set_addr_long(dev, NTOHLL(addr_long.uint64.u64));
    ata8510_set_addr_short(dev, NTOHS(addr_long.uint16[0].u16));
#else
    ata8510_set_addr_long(dev, ATA8510_DEFAULT_ADDR_LONG);
    ata8510_set_addr_short(dev, ATA8510_DEFAULT_ADDR_SHORT);
#endif
//    /* set default PAN id */
//    ata8510_set_pan(dev, ATA8510_DEFAULT_PANID);
//    /* set default channel */
//    ata8510_set_chan(dev, ATA8510_DEFAULT_CHANNEL);
//    /* set default TX power */
//    ata8510_set_txpower(dev, ATA8510_DEFAULT_TXPOWER);
//    /* set default options */
//    ata8510_set_option(dev, NETDEV2_IEEE802154_PAN_COMP, true);
//    ata8510_set_option(dev, ATA8510_OPT_AUTOACK, true);
//    ata8510_set_option(dev, ATA8510_OPT_CSMA, true);
//    ata8510_set_option(dev, ATA8510_OPT_TELL_RX_START, false);
//    ata8510_set_option(dev, ATA8510_OPT_TELL_RX_END, true);
//#ifdef MODULE_NETSTATS_L2
//    ata8510_set_option(dev, ATA8510_OPT_TELL_TX_END, true);
//#endif
    /* set default protocol */
#ifdef MODULE_GNRC_SIXLOWPAN
    dev->netdev.proto = GNRC_NETTYPE_SIXLOWPAN;
#elif MODULE_GNRC
    dev->netdev.proto = GNRC_NETTYPE_UNDEF;
#endif
//    /* enable safe mode (protect RX FIFO until reading data starts) */
//    ata8510_reg_write(dev, ATA8510_REG__TRX_CTRL_2,
//                        ATA8510_TRX_CTRL_2_MASK__RX_SAFE_MODE);
//#ifdef MODULE_AT86RF212B
//    ata8510_set_page(dev, 0);
//#endif
//
//    /* don't populate masked interrupt flags to IRQ_STATUS register */
//    uint8_t tmp = ata8510_reg_read(dev, ATA8510_REG__TRX_CTRL_1);
//    tmp &= ~(ATA8510_TRX_CTRL_1_MASK__IRQ_MASK_MODE);
//    ata8510_reg_write(dev, ATA8510_REG__TRX_CTRL_1, tmp);
//
//    /* disable clock output to save power */
//    tmp = ata8510_reg_read(dev, ATA8510_REG__TRX_CTRL_0);
//    tmp &= ~(ATA8510_TRX_CTRL_0_MASK__CLKM_CTRL);
//    tmp &= ~(ATA8510_TRX_CTRL_0_MASK__CLKM_SHA_SEL);
//    tmp |= (ATA8510_TRX_CTRL_0_CLKM_CTRL__OFF);
//    ata8510_reg_write(dev, ATA8510_REG__TRX_CTRL_0, tmp);
//
//    /* enable interrupts */
//    ata8510_reg_write(dev, ATA8510_REG__IRQ_MASK,
//                        ATA8510_IRQ_STATUS_MASK__TRX_END);
//    /* clear interrupt flags */
//    ata8510_reg_read(dev, ATA8510_REG__IRQ_STATUS);
//
//    /* go into RX state */
//    ata8510_set_state(dev, ATA8510_STATE_RX_AACK_ON);

    DEBUG("ata8510_reset(): reset complete.\n");
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
