/*
 * Copyright (C) 2016 Antonio Galea <antonio.galea@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_ata8510
 * @{
 *
 * @file
 * @brief       Internal interfaces for ATA8510 driver
 *
 * @author      Antonio Galea <antonio.galea@gmail.com>
 * @author      Roberto Asquini <bobasquins@gmail.com>
 */

#ifndef ATA8510_INTERNAL_H_
#define ATA8510_INTERNAL_H_

#include <stdint.h>

#include "ata8510.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Constant part number of the ATA8510
 * @{
 */
#define ATA8510_PARTNUM       (0x21)

/**
 * @brief   Send a command to the given device
 *
 * @param[in]  dev        device to read from
 * @param[in]  tx_buffer  data to transmit
 * @param[out] rx_buffer  buffer to read data into
 * @param[in]  len        number of bytes to transmit
 */
int ata8510_send_cmd(const ata8510_t *dev,
                          const uint8_t *tx_buffer,
                          const uint8_t *rx_buffer,
                          const size_t len);

/**
 * @brief   Power on sequence 
 *
 * @param[in] dev       device to manipulate
 */
void ata8510_power_on(const ata8510_t *dev);

/**
 * @brief   Cancel ongoing transactions and switch to TRX_OFF state
 *
 * @param[in] dev       device to manipulate
 */
void ata8510_force_trx_off(const ata8510_t *dev);

/**
 * @brief   Convenience function for reading the status of the given device
 *
 * @param[in] dev       device to read the status from
 *
 * @return              internal status of the given device
 */
uint8_t ata8510_get_status(const ata8510_t *dev);

/**
 * @brief   Make sure that device is not sleeping
 *
 * @param[in] dev       device to eventually wake up
 */
void ata8510_assert_awake(ata8510_t *dev);

/**
 * @brief   Trigger a hardware reset
 *
 * @param[in] dev       device to reset
 */
void ata8510_hardware_reset(ata8510_t *dev);


/**
 * @brief   Set PHY parameters based on channel and page number
 *
 * @param[in] dev       device to configure
 */
void ata8510_configure_phy(ata8510_t *dev);


/**
 * @brief Write TX preamble buffer to RF transceiver
 *
 * @param[in] dev       device to configure
 * @param[in] data_size number of bytes of the preamble
 * @param[in] data      preamble to trasmit
 */
int ata8510_WriteTxPreamble(ata8510_t *dev, uint8_t data_size, uint8_t *data);

/**
 * @brief Write TX buffer of the RF transceiver
 *
 * @param[in] dev       device to configure
 * @param[in] data_size number of bytes of the payload
 * @param[in] data	payload to trasmit
 */
int ata8510_WriteTxFifo(ata8510_t *dev,  uint8_t data_size, uint8_t *data);

/**
 * @brief Set system mode of the RF receiver
 *
 * @param[in] dev          device to configure
 * @param[in] mode_config  number of bytes of the preamble
 * @param[in] service_channel  service and channel to be used to transmit
 */
int ata8510_SetSystemMode(ata8510_t *dev, uint8_t mode_config, uint8_t service_channel);


#ifdef __cplusplus
}
#endif

#endif /* ATA8510_INTERNAL_H_ */
/** @} */
