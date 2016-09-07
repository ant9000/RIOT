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
 * @author      Roberto Asquini <bobasquins@gmail.com>
 *
 * @}
 */

#include "periph/spi.h"
#include "periph/gpio.h"
#include "xtimer.h"
#include "ata8510_internal.h"
#include "ata8510_params.h"

#define ENABLE_DEBUG (0)
#include "debug.h"
#include "xtimer.h"

int ata8510_send_cmd(const ata8510_t *dev,
                          const uint8_t *tx_buffer,
                          const uint8_t *rx_buffer,
                          const size_t len)
{
    int count;
    spi_acquire(dev->params.spi);
    gpio_clear(dev->params.cs_pin);
	xtimer_usleep(1);
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
    xtimer_usleep(310);
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

uint8_t ata8510_SetPollingMode(ata8510_t *dev)
{
    // set polling mode
	uint8_t command[3]={ ATA8510_CMD_SETSYSTEMMODE, ATA8510_CMD_SETPOLLINGMODE, 0x00};
	uint8_t data[3];
	ata8510_send_cmd(dev, command, data, sizeof(command));
    //xtimer_usleep(70);
	return data[0];
}


uint8_t ata8510_SetIdleMode(ata8510_t *dev)
{
	// set idle mode
	uint8_t command[3]={ ATA8510_CMD_SETSYSTEMMODE, ATA8510_RF_IDLEMODE, ATA8510_RF_RXSERVICE};
	uint8_t data[3];
	ata8510_send_cmd(dev, command, data, sizeof(command));
    xtimer_usleep(70);
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
/*  // if enabled the following DEBUG print it will delay too much the reading of the DFIFO buffer. Use only in case of real need and disable immediately after 
	DEBUG(
		"[ata8510] Get Event Bytes: [0x%02x 0x%02x 0x%02x 0x%02x]\n",
		data[0], data[1], data[2], data[3]
	);
*/
}

void ata8510_ReadRSSIFIFO(ata8510_t *dev, uint8_t len, uint8_t *data){
    // FIFO has 16 bytes
    uint8_t command[19]={
        ATA8510_CMD_READRSSIFIFO,len,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    };
    ata8510_send_cmd(dev, command, data, len+3);
#if ENABLE_DEBUG
    DEBUG("[ata8510] Read RSSI FIFO: [");
    for(int i=0;i<len;i++){ DEBUG(" 0x%02x", data[i]); }
    DEBUG("]\n");
#endif
}

int ata8510_calc_dbm(uint8_t rssi) {
	return ( (rssi>>1) - 135 );
}

void ata8510_ReadRxFIFO(ata8510_t *dev, uint8_t len, uint8_t *data){
	// FIFO has 32 bytes
	uint8_t command[35]={
        ATA8510_CMD_READRXFIFO,len,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    };
	ata8510_send_cmd(dev, command, data, len+3);
#if ENABLE_DEBUG
	DEBUG("[ata8510] Read Rx FIFO: [");
    for(int i=0;i<len+3;i++){ DEBUG(" 0x%02x", data[i]); }
	DEBUG("]\n");
#endif
}

void ata8510_write_sram_register(ata8510_t *dev, uint16_t addr, uint8_t data){
	// writes n=len consecutive bytes (max 16) in the 8510 SRAM or registers starting from 16bits address addr
	uint8_t addrh = (addr&0xFF00)>>8;
	uint8_t addrl = addr&0x00FF;
	uint8_t command[5]={ATA8510_CMD_WRITESRAM,1,addrh,addrl,data};
	uint8_t dummy[5]={0,0,0,0,0};

	ata8510_send_cmd(dev, command, dummy, 5);
	DEBUG("[ata8510] Write SRAM / Register: addr = 0x%04x, data = 0x%02x\n", addr, command[4]);
}

uint8_t ata8510_read_sram_register(ata8510_t *dev, uint16_t addr){
	// reads 1 byte from the 8510 SRAM or register from 16bits address addr
	uint8_t addrh = (addr&0xFF00)>>8;
	uint8_t addrl = addr&0x00FF;
	uint8_t command[6]={ATA8510_CMD_READSRAM,1,addrh,addrl,0x00,0x00};
	uint8_t data[6]={0,0,0,0,0,0};
	ata8510_send_cmd(dev, command, data, 6);
	DEBUG("[ata8510] Read SRAM / Register: addr = 0x%04x, data = 0x%02x\n", addr, data[5]);
	return data[5];
}



void ata8510_StartRSSI_Measurement(ata8510_t *dev, uint8_t service, uint8_t channel){
	uint8_t command[2]={ATA8510_CMD_STARTRSSIMSRMNT,0x00};
	uint8_t dummy[2];
	uint8_t servchan;
	if (service <= 2) {
		servchan = service;
	} else {
		DEBUG("tx_exec: Service not permitted %d\n",service);
		return;
	}
	if (channel<=2) {
		servchan |= ( (channel<<4) + 0x40);
	} else {
		DEBUG("tx_exec: Channel not permitted %d\n",service);
		return;
	}
	command[1] = servchan;
	ata8510_send_cmd(dev, command, dummy, sizeof(command));
	DEBUG(
		"[ata8510] Start RSSI Measurement: [0x%02x 0x%02x]\n",
		command[0], command[1]
	);
}

void ata8510_GetRSSI_Value(ata8510_t *dev, uint8_t *data){
	uint8_t command[4]={ATA8510_CMD_GETRSSIMSRMNT,0x00,0x00,0x00};
	ata8510_send_cmd(dev, command, data, sizeof(command));
	DEBUG(
		"[ata8510] Get RSSI Value: [0x%02x 0x%02x 0x%02x 0x%02x]\n",
		data[0], data[1], data[2], data[3]
	);
}

/**
 * \brief Write TX preamble buffer of the RF transceiver
 *
 * Write data to the TX preamble buffer of the transceiver.
 */
int ata8510_WriteTxPreamble(ata8510_t *dev, uint8_t data_size, uint8_t *data)
{
	int ret;
	uint8_t command[19]={ATA8510_CMD_WRITETXPREAMBLE, data_size};
	uint8_t dummy[19];
	uint8_t index;

	if (data_size<=19) {
		for (index=0; index <data_size; index++)
		{
			command[2 + index]= data[index];
		}

		ret=ata8510_send_cmd(dev, command, dummy, data_size+2);
		return ret;

	} else
		return -100;  // ROB: PREAMBLE FIFO is 16 bytes!!

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

/**
 * \brief Write TX buffer of the RF transceiver
 *
 * Write data to the TX buffer of the transceiver.
 */
int ata8510_WriteTxFifo(ata8510_t *dev, uint8_t data_size, uint8_t *data)
{
	int ret;
	uint8_t command[34]={
        ATA8510_CMD_WRITETXFIFO,data_size,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    };
	uint8_t dummy[34];
	uint8_t i;

#if ENABLE_DEBUG
	DEBUG("ata8510_WriteTxFifo(\n\tdata_size=%d\n\tdata=[", data_size);
    for(int i=0;i<data_size;i++){ DEBUG(" 0x%02x", data[i]); }
    DEBUG(" ])\n");
#endif

	if(data_size>32) {
        DEBUG("ata8510_WriteTxFifo: too much data received (%d), truncating\n", data_size);
        data_size = 32;
    }

    for (i=0; i<data_size; i++) {
        command[i + 2]= data[i];
    }

    ret=ata8510_send_cmd(dev, command, dummy, data_size+2);
    return ret;
}


/**
 * \brief Set system mode of the RF transceiver
 *
 * Switch the transceiver to the wanted system mode.
 */
int ata8510_SetSystemMode(ata8510_t *dev, uint8_t mode_config, uint8_t service_channel)
{
	int ret;
	uint8_t command[3]={ATA8510_CMD_SETSYSTEMMODE, mode_config, service_channel};
	uint8_t dummy[3];

	ret=ata8510_send_cmd(dev, command, dummy, ATA8510_CMD_SETSYSTEMMODE_LEN);

	return ret;
}
