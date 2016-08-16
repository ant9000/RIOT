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
#include "ata8510_params.h"
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
    }
}

void ata8510_get_version_flash(ata8510_t *dev, uint8_t *data){
    uint8_t command[6]={0x13,0x00,0x00,0x00,0x00,0x00};
    ata8510_send_cmd(dev, command, data, sizeof(command));
    DEBUG(
		"[ata8510] GetVersionFlash: [0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]\n",
		data[0], data[1], data[2], data[3], data[4], data[5]
    );
}

uint8_t ata8510_SetPollingMode(ata8510_t *dev)
{
    // set polling mode
	uint8_t command[3]={ ATA8510_CMD_SETSYSTEMMODE, ATA8510_CMD_SETPOLLINGMODE, 0x00};
	uint8_t data[3];
	ata8510_send_cmd(dev, command, data, sizeof(command));
	ata8510_set_state(dev, POLLING);
	return data[0];
}


uint8_t ata8510_SetIdleMode(ata8510_t *dev)
{
	// set idle mode
	uint8_t command[3]={ ATA8510_CMD_SETSYSTEMMODE, ATA8510_RF_IDLEMODE, ATA8510_RF_RXSERVICE};
	uint8_t data[3];
	ata8510_send_cmd(dev, command, data, sizeof(command));
	ata8510_set_state(dev, IDLE);
	return data[0];
}


uint8_t ata8510_ReadFillLevelRxFIFO(ata8510_t *dev){
	uint8_t command[3]={ATA8510_CMD_READRXFILLLEVEL,0x00,0x00};
	uint8_t data[3];
	ata8510_send_cmd(dev, command, data, sizeof(command));
	DEBUG(
		"[ata8510] Read Fill Level Rx FIFO: [0x%02x 0x%02x 0x%02x]\n",
		data[0], data[1], data[2]
	);
	return data[2];
}

uint8_t ata8510_ReadFillLevelTxFIFO(ata8510_t *dev){
	uint8_t command[3]={ATA8510_CMD_READTXFILLLEVEL,0x00,0x00};
	uint8_t data[3];
	ata8510_send_cmd(dev, command, data, sizeof(command));
	DEBUG(
		"[ata8510] Read Fill Level Tx FIFO: [0x%02x 0x%02x 0x%02x]\n",
		data[0], data[1], data[2]
	);
	return data[2];
}

uint8_t ata8510_ReadFillLevelRSSIFIFO(ata8510_t *dev){
	uint8_t command[3]={ATA8510_CMD_READRSSIFILLLEVEL,0x00,0x00};
	uint8_t data[3];
	ata8510_send_cmd(dev, command, data, sizeof(command));
	DEBUG(
		"[ata8510] Read Fill Level RSSI FIFO: [0x%02x 0x%02x 0x%02x]\n",
		data[0], data[1], data[2]
	);
	return data[2];
}

void ata8510_GetEventBytes(ata8510_t *dev, uint8_t *data){
	uint8_t command[4]={ATA8510_CMD_GETCONFIGEVENTBYTE,0x00,0x00,0x00};
	ata8510_send_cmd(dev, command, data, sizeof(command));
	DEBUG(
		"[ata8510] Get Event Bytes: [0x%02x 0x%02x 0x%02x 0x%02x]\n",
		data[0], data[1], data[2], data[3]
	);
}

void ata8510_ReadRSSIFIFO(ata8510_t *dev, uint8_t len, uint8_t *data){
	// FIFO has 16 bytes
	uint8_t command[19]={ATA8510_CMD_READRSSIFIFO,len,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
							0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	ata8510_send_cmd(dev, command, data, len+3);
	DEBUG(
		"[ata8510] Read RSSI FIFO: [0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x ]\n",
		data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], 
		data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18]
	);
}

void ata8510_ReadRxFIFO(ata8510_t *dev, uint8_t len, uint8_t *data){
	// FIFO has 16 bytes
	uint8_t command[19]={ATA8510_CMD_READRXFIFO,len,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	ata8510_send_cmd(dev, command, data, len+3);
	DEBUG(
		"[ata8510] Read Rx FIFO: [0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x ]\n",
		data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], 
		data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18]
	);
}



uint8_t ata8510_get_device_signature(ata8510_t *dev){
    uint8_t data[6]={0,0,0,0,0,0};
    ata8510_get_version_flash(dev,data);
    return data[2];
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

ATA8510STATES state8510;

void ata8510_set_state(ata8510_t *dev, uint8_t state)
{
	state8510 = state;
}

uint8_t ata8510_get_state(ata8510_t *dev)
{
	return state8510;
}

void ata8510_reset_state_machine(ata8510_t *dev)
{
	ata8510_SetIdleMode(dev);
	state8510 = IDLE;
}
