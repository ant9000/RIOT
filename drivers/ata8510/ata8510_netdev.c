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
#include "xtimer.h"

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
    ringbuffer_init(&dev->rb, (char *)dev->mem, sizeof(dev->mem));

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
    ata8510_t *dev = (ata8510_t *)netdev;
    const struct iovec *ptr = vector;
    size_t len = 0;
    uint8_t data[ATA8510_DFIFO_TX_LENGTH];
    int n;

    if (dev->pending_tx) {
        DEBUG("_send: error: pending transmissiong\n");
        return -EOVERFLOW;
    }
    dev->pending_tx = 1;

    ata8510_SetIdleMode(dev);
    ata8510_tx_prepare(dev);

    /* load packet data into buffer */
    for (unsigned i = 0; i < count; i++, ptr++) {
        /* current packet data too long */
        if ((len + ptr->iov_len) > ATA8510_MAX_PKT_LENGTH) {
            DEBUG("_send: packet too large (%u byte) to be sent\n",
                  (unsigned)len);
            return -EOVERFLOW;
        }
        len = ata8510_tx_load(dev, ptr->iov_base, ptr->iov_len, len);
    }

    // activate tx mode 
    ata8510_tx_exec(dev);

    // send message
    while(!ringbuffer_empty(&dev->rb)) {
        n = ata8510_ReadFillLevelTxFIFO(dev);
        if (n < ATA8510_DFIFO_TX_LENGTH>>1) { // DFIFO_TX is at least half empty
            n = ATA8510_DFIFO_TX_LENGTH - n - 1; // free space
            if (n > ATA8510_DFIFO_TX_LENGTH>>1) { // never fill pipe more than half
                n = ATA8510_DFIFO_TX_LENGTH>>1;
            }
            n = ringbuffer_get(&dev->rb, (char *)data, n);
            if (n) {
                DEBUG("_send: batch %d bytes\n", n);
                ata8510_WriteTxFifo(dev, n, data);
            }
        }
    }
    DEBUG("_send: sent %d bytes\n", len);
#ifdef MODULE_NETSTATS_L2
    netdev->stats.tx_bytes += len;
#endif

    /* return the number of bytes that were sent */
    return (int)len;
}

static int _recv(netdev2_t *netdev, void *buf, size_t len, void *info)
{
    size_t pkt_len;
    uint8_t i;
	int rssicalc;

    ata8510_t *dev = (ata8510_t *)netdev;

    if(ringbuffer_empty(&dev->rb)){ return 0; }
    pkt_len = dev->rb.avail;

    /* just return length when buf == NULL */
    if (buf == NULL) {
        return pkt_len;
    }
#ifdef MODULE_NETSTATS_L2
    netdev->stats.rx_count++;
    netdev->stats.rx_bytes += pkt_len;
#endif
    /* not enough space in buf */
    if (pkt_len > len) {
        return -ENOBUFS;
    }

    /* copy payload */
    ringbuffer_get(&dev->rb, (char *)buf, pkt_len);

	DEBUG("_recv: pkt_len=%d\n", pkt_len);

    if (info != NULL) {
        netdev2_ieee802154_rx_info_t *radio_info = info;
        radio_info->lqi=0;
        radio_info->rssi=0;
        rssicalc=0;
        if (dev->RSSI_len > 0) {
            for (i=0; i<dev->RSSI_len; i++) { rssicalc+=dev->RSSI[i]; }
            radio_info->rssi = rssicalc/dev->RSSI_len;
            DEBUG("_recv: RSSI values = %d RSSI = %d \n", dev->RSSI_len, radio_info->rssi);
        } else {
            DEBUG("_recv: no RSSI values read\n");
        }
    }
    return pkt_len;
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

static void _isr(netdev2_t *netdev){
    uint8_t data[37];
    uint8_t status[4];
    uint8_t dataSFIFO[19];
    uint8_t mystate8510, mynextstate8510;
	int i, n;
    int sfifo_len=0, dfifo_rx_len=0;
    uint16_t errorcode;

    ata8510_t *dev = (ata8510_t *)netdev;
	dev->interrupts++;
	ata8510_GetEventBytes(dev, status);

	mystate8510 = ata8510_get_state(dev);
	mynextstate8510 = ata8510_get_state_after_tx(dev);

    // empty all buffers as soon as possible

    if (
        (status[ATA8510_SYSTEM] & ATA8510_SYSTEM_SFIFO)    || // SFIFO fill event
        (status[ATA8510_SYSTEM] & ATA8510_SYSTEM_DFIFO_RX) || // DFIFO_RX fill event
	    (status[ATA8510_EVENTS] & ATA8510_EVENTS_EOTA)        // EOTA event
    ) {
        switch (mystate8510) {
            case TX_ON:
                // TODO: in TX mode, SFIFO fills up when preamble is big
                break;
            default:
                sfifo_len = ata8510_ReadFillLevelRSSIFIFO(dev);
                if (sfifo_len>0) {
                    ata8510_ReadRSSIFIFO(dev, sfifo_len, dataSFIFO);
                    for (i=0; i<sfifo_len; i++) {
                        dev->RSSI[ (i + dev->RSSI_len) % sizeof(dev->RSSI) ] = dataSFIFO[i+3];
                    }
                    dev->RSSI_len += sfifo_len;
                    if(dev->RSSI_len > sizeof(dev->RSSI)){
                        dev->RSSI_len = sizeof(dev->RSSI);
                    }
                }
                break;
        }
    }

	if (
        (status[ATA8510_SYSTEM] & ATA8510_SYSTEM_DFIFO_RX) || // DFIFO_RX fill event
	    (status[ATA8510_EVENTS] & ATA8510_EVENTS_EOTA)        // EOTA event
    ) {
        dfifo_rx_len = ata8510_ReadFillLevelRxFIFO(dev);
        if (dfifo_rx_len>0) { // there is data to read
            ata8510_ReadRxFIFO(dev, dfifo_rx_len, data);
            n = ringbuffer_add(&dev->rb, (char *)data+3, dfifo_rx_len);
            if (n != dfifo_rx_len){ DEBUG("_isr: RX buffer overflow"); }
        }
	}

    // SYS_ERR event
    errorcode = 0;
	if (status[ATA8510_SYSTEM] & ATA8510_SYSTEM_SYS_ERR) {
		errorcode = ata8510_read_error_code(dev);
        switch (errorcode) {
            case 21: //DEBUG_ERROR_CODE_SFIFO_OVER_UNDER_FLOW
                break;
            default:
		        dev->sys_errors++;
                break;
        }
	}

    DEBUG("_isr: state: %d\n", mystate8510);
    DEBUG(
        "_isr: Get Event Bytes: %02x %02x %02x %02x\n",
        status[0], status[1], status[2], status[3]
    );
    DEBUG(
        "_isr: SYS_ERR=%d CMD_RDY=%d SYS_RDY=%d SFIFO=%d DFIFO_RX=%d DFIFO_TX=%x\n",
        (status[ATA8510_SYSTEM] & ATA8510_SYSTEM_SYS_ERR  ? 1 : 0),
        (status[ATA8510_SYSTEM] & ATA8510_SYSTEM_CMD_RDY  ? 1 : 0),
        (status[ATA8510_SYSTEM] & ATA8510_SYSTEM_SYS_RDY  ? 1 : 0),
        (status[ATA8510_SYSTEM] & ATA8510_SYSTEM_SFIFO    ? 1 : 0),
        (status[ATA8510_SYSTEM] & ATA8510_SYSTEM_DFIFO_RX ? 1 : 0),
        (status[ATA8510_SYSTEM] & ATA8510_SYSTEM_DFIFO_TX ? 1 : 0)
    );
    DEBUG(
        "_isr: SOTA=%d EOTA=%d\n",
        (status[ATA8510_EVENTS] & ATA8510_EVENTS_SOTA     ? 1 : 0),
        (status[ATA8510_EVENTS] & ATA8510_EVENTS_EOTA     ? 1 : 0)
    );
    if (errorcode) {
		DEBUG("_isr: SysErr=%d  SSM state= %d\n", errorcode>>8, errorcode&0xff);
    }
    if (sfifo_len) {
       DEBUG("_isr: SFIFO len=%d data=[ ", sfifo_len);
       for (i=0; i<sfifo_len; i++) DEBUG(" %d", dataSFIFO[i+3]);
       DEBUG("]\n");
       DEBUG("_isr: RSSI len=%d data=[ ", dev->RSSI_len);
       for (i=0; i<dev->RSSI_len; i++) DEBUG(" %d", dev->RSSI[i]);
       DEBUG("]\n");
    }
    if (dfifo_rx_len) {
       DEBUG("_isr: DFIFO_RX len=%d\n", dfifo_rx_len);
    }
    DEBUG("\n");

    // EOTA event
	if (status[ATA8510_EVENTS] & ATA8510_EVENTS_EOTA) {
	    switch (mystate8510) {
            case TX_ON:
                // flush TX ringbuffer
                if (!ringbuffer_empty(&dev->rb)) {
                    DEBUG(
                        "_isr: TX end, discarding %d stale bytes from buffer\n",
                        dev->rb.avail
                    );
                    ringbuffer_remove(&dev->rb, dev->rb.avail);
                }

                ata8510_SetIdleMode(dev);
                switch (mynextstate8510) {
                    case IDLE:
                        break;
                    case POLLING:
                         ata8510_SetPollingMode(dev);
                         break;
                    default:
                         DEBUG("_isr: Cannot handle state %d after TX\n", mynextstate8510);
                         break;
                }
                dev->pending_tx = 0;
                break;
		    case POLLING:
                dev->service = ATA8510_CONFIG_SERVICE(status[ATA8510_CONFIG]);
                dev->channel = ATA8510_CONFIG_CHANNEL(status[ATA8510_CONFIG]);

                ata8510_SetIdleMode(dev);
                ata8510_SetPollingMode(dev);

                netdev->event_callback(netdev, NETDEV2_EVENT_RX_COMPLETE);
                break;

		    default:
				dev->unknown_case++;
		        break;
        }
    }
}
