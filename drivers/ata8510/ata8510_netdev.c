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

static void _isr(netdev2_t *netdev){
    uint8_t data[37];
    uint8_t saved8510status[4];
    uint8_t dataSFIFO[19];
    uint8_t mystate8510, mynextstate8510;
	uint8_t rxlen, rssilen, sfifolen = 0;;
	uint8_t i;
    uint16_t errorcode;

    ata8510_t *dev = (ata8510_t *)netdev;
	dev->interrupts++;
	ata8510_GetEventBytes(dev, data);
	if (data[0] & 0x80) {
		// SYS_ERR happened
		errorcode = ata8510_read_error_code(dev);
		DEBUG("     _isr SYS_ERR! %02x %02x %02x %02x Errorcode = 0x%04x  SysErr=%d  SSM state= %d\n",
				data[0], data[1], data[2], data[3], errorcode, errorcode>>8, errorcode&0xff);
		dev->sys_errors++;
	}
	mystate8510 = ata8510_get_state(dev);
	mynextstate8510 = ata8510_get_state_after_tx(dev);
	saved8510status[0] = data[0];  // save it now since if we print we lost the message
	saved8510status[1] = data[1];
	saved8510status[2] = data[2];
	saved8510status[3] = data[3];

	switch (mystate8510) {
		case IDLE:
			DEBUG("_isr State IDLE!\n");
		break;
		case TX_ON:
			switch (data[1]&0x10) {
				case 0x10:
				// end of transmission
					DEBUG("End of Transmission!\n");
					ata8510_SetIdleMode(dev);
					xtimer_usleep(70);
					switch(mynextstate8510) {
						case IDLE:
						break;
						case POLLING:
							ata8510_SetPollingMode(dev);
							ata8510_set_state(dev, POLLING);
						break;
						default:
							printf("_isr Cannot handle state 8510 %d after TX \n", mynextstate8510);
					}
				break;
				default: {
					printf("_isr Unknown events.events %02x [%02x] %02x %02x \n",
							data[0], data[1], data[2], data[3]);
					dev->unknown_case++;
				}
			DEBUG("_isr State TX_ON!\n");
			}
		break;
		case RX_ON:
			DEBUG("_isr State RX_ON!\n");
		break;
		case POLLING:
			switch (data[0]) {
				case 0x00:
				case 0x02:
				case 0x22:
				case 0x20:
				case 0x80: // SYS_ERR: probably after a wrong reception. For now restart 8510
				case 0x82:
					// receiving! Even after an error..
					if (data[1] & 0x10) {  // EOT in Rx. Message complete. We can read
						dev->rx_service = (data[3] & 0x07);
						dev->rx_channel = (data[3] & 0x30)>>4;
						rxlen = ata8510_ReadFillLevelRxFIFO(dev);
						dev->rx_len = rxlen;
						ata8510_ReadRxFIFO(dev, rxlen, data);
						data[rxlen+3]=0x00; // close in any case the message received
						rssilen=ata8510_ReadFillLevelRSSIFIFO(dev);
						if (rssilen>4) {
							ata8510_ReadRSSIFIFO(dev, rssilen, dev->RSSI);
							dev->RSSI[2]=rssilen; // uses the dummy location to save the length of the RSSI buffer.
						} else {
							// if interrupt SFIFO has emptied the SFIFO get from dataSFIFO some RSSI values
							for (i=3; i< 10; i++)	dev->RSSI[i] = dataSFIFO[i];
							dev->RSSI[2] = 7;
							for (i=3; i< 10; i++) printf(" %d", dataSFIFO[i]);
							DEBUG("--\n");
						}
						ata8510_SetIdleMode(dev);
						xtimer_usleep(70);
						ata8510_SetPollingMode(dev);
						for (i=0; i<rxlen+1; i++) {
							dev->rx_buffer[i] = data[i+3];  // raw copy of received bytes in ata8510 receive buffer
						}
						dev->rx_available = 1;
						DEBUG("_isr RxLen %d  8510event %d Data Received: %s\n",
								rxlen, dev->interrupts,(char *)&data[3]);
					}
					break;
				case 0x04:
				case 0x24:
						// SFIFO event
						;
					break;
				default:
						printf("   _isr Unknown case %02x %02x %02x %02x interrupts %d\n",
								data[0], data[1], data[2], data[3], dev->interrupts);
						dev->blocked = 1;
						dev->unknown_case++;
//							ata8510_SetPollingMode(dev);
					break;
			DEBUG("_isr State POLLING!\n");
			}
		break;
		case RSSIMEAS:
			DEBUG("_isr State RSSI Measure!\n");
			;
		break;
		default:
			printf("_isr Unknown case: state 8510 = %d\n", mystate8510);
			dev->unknown_case++;
	}

	if (saved8510status[0]&0x04) {
		// SFIFO event. In any case read the SFIFO
		sfifolen=ata8510_ReadFillLevelRSSIFIFO(dev);
		DEBUG("_isr rssilen = %d\n",(int)sfifolen);
		if (sfifolen>0) {
			ata8510_ReadRSSIFIFO(dev, sfifolen, dataSFIFO);
			for (i=0; i< sfifolen; i++) DEBUG(" %d", dataSFIFO[i]);
		}
		DEBUG("\n");
	}
}
