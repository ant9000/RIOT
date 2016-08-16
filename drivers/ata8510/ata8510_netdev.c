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
 * @brief       Netdev adaption for the ATA8510 driver
 *
 * @author      Antonio Galea <antonio.galea@gmail.com>
 * @author      Roberto Asquini <bobasquins@gmail.com>
 *
 * @}
 */

#include <string.h>
#include <assert.h>
#include <errno.h>

#include "net/eui64.h"
#include "net/ieee802154.h"
#include "net/netdev2.h"
#include "net/netdev2/ieee802154.h"

#include "ata8510.h"
#include "ata8510_netdev.h"
#include "ata8510_internal.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define _MAX_MHR_OVERHEAD   (25)

static int _send(netdev2_t *netdev, const struct iovec *vector, unsigned count);
static int _recv(netdev2_t *netdev, void *buf, size_t len, void *info);
static int _init(netdev2_t *netdev);
static void _isr(netdev2_t *netdev);
static int _get(netdev2_t *netdev, netopt_t opt, void *val, size_t max_len);
static int _set(netdev2_t *netdev, netopt_t opt, void *val, size_t len);

const netdev2_driver_t ata8510_driver = {
    .send = _send,
    .recv = _recv,
    .init = _init,
    .isr = _isr,
    .get = _get,
    .set = _set,
};


static void _irq_handler(void *arg)
{
    netdev2_t *dev = (netdev2_t *) arg;

    if (dev->event_callback) {
        dev->event_callback(dev, NETDEV2_EVENT_ISR);
    }
}

static int _init(netdev2_t *netdev)
{
    ata8510_t *dev = (ata8510_t *)netdev;

    /* initialise GPIOs */
    gpio_init(dev->params.cs_pin, GPIO_OUT);
    gpio_set(dev->params.cs_pin);
    gpio_init(dev->params.sleep_pin, GPIO_OUT);
    gpio_clear(dev->params.sleep_pin);
    gpio_init(dev->params.reset_pin, GPIO_OUT);
    gpio_set(dev->params.reset_pin);
    gpio_init_int(dev->params.int_pin, GPIO_IN, GPIO_FALLING, _irq_handler, dev);

    ata8510_power_on(dev);
    spi_poweron(dev->params.spi);

    /* test if the SPI is set up correctly and the device is responding */
    if (ata8510_get_device_signature(dev) != ATA8510_PARTNUM){
        DEBUG("[ata8510] error: unable to read correct part number\n");
        return -1;
    }

    DEBUG("[ata8510] init done\n");

#ifdef MODULE_NETSTATS_L2
    memset(&netdev->stats, 0, sizeof(netstats_t));
#endif
    /* reset device to default values and put it into RX state */
    ata8510_reset(dev);

    return 0;
}

static int _send(netdev2_t *netdev, const struct iovec *vector, unsigned count)
{
    return 0;
}

static int _recv(netdev2_t *netdev, void *buf, size_t len, void *info)
{
    return 0;
}

static int _get(netdev2_t *netdev, netopt_t opt, void *val, size_t max_len)
{
    int res = -ENOTSUP;

    if (((res = netdev2_ieee802154_get((netdev2_ieee802154_t *)netdev, opt, val,
                                       max_len)) >= 0) || (res != -ENOTSUP)) {
        return res;
    }

    return res;
}

static int _set(netdev2_t *netdev, netopt_t opt, void *val, size_t len)
{
    int res = -ENOTSUP;
    return res;
}

static void _isr(netdev2_t *netdev)
{
}
