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

#define ENABLE_DEBUG (1)
#include "debug.h"

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
    xtimer_usleep(310);
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
