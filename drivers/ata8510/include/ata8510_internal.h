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
 */

#ifndef ATA8510_INTERNAL_H_
#define ATA8510_INTERNAL_H_

#include <stdint.h>

#include "ata8510.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Read from a register at address `addr` from device `dev`.
 *
 * @param[in] dev       device to read from
 * @param[in] addr      address of the register to read
 *
 * @return              the value of the specified register
 */
uint8_t ata8510_reg_read(const ata8510_t *dev, const uint8_t addr);

/**
 * @brief   Write to a register at address `addr` from device `dev`.
 *
 * @param[in] dev       device to write to
 * @param[in] addr      address of the register to write
 * @param[in] value     value to write to the given register
 */
void ata8510_reg_write(const ata8510_t *dev, const uint8_t addr,
                         const uint8_t value);

/**
 * @brief   Read a chunk of data from the SRAM of the given device
 *
 * @param[in]  dev      device to read from
 * @param[in]  offset   starting address to read from [valid 0x00-0x7f]
 * @param[out] data     buffer to read data into
 * @param[in]  len      number of bytes to read from SRAM
 */
void ata8510_sram_read(const ata8510_t *dev,
                         const uint8_t offset,
                         uint8_t *data,
                         const size_t len);

/**
 * @brief   Write a chunk of data into the SRAM of the given device
 *
 * @param[in] dev       device to write to
 * @param[in] offset    address in the SRAM to write to [valid 0x00-0x7f]
 * @param[in] data      data to copy into SRAM
 * @param[in] len       number of bytes to write to SRAM
 */
void ata8510_sram_write(const ata8510_t *dev,
                          const uint8_t offset,
                          const uint8_t *data,
                          const size_t len);

/**
 * @brief   Start a read transcation internal frame buffer of the given device
 *
 * Reading the frame buffer returns some extra bytes that are not accessible
 * through reading the RAM directly. This locks the used SPI.
 *
 * @param[in]  dev      device to start read
 */
void ata8510_fb_start(const ata8510_t *dev);

/**
 * @brief   Read the internal frame buffer of the given device
 *
 * Each read advances the position in the buffer by @p len.
 *
 * @param[in]  dev      device to read from
 * @param[out] data     buffer to copy the data to
 * @param[in]  len      number of bytes to read from the frame buffer
 */
void ata8510_fb_read(const ata8510_t *dev,
                       uint8_t *data, const size_t len);

/**
 * @brief   Stop a read transcation internal frame buffer of the given device
 *
 * Release the SPI device and unlock frame buffer protection.
 *
 * @param[in]  dev      device to stop read
 */
void ata8510_fb_stop(const ata8510_t *dev);

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


#ifdef __cplusplus
}
#endif

#endif /* ATA8510_INTERNAL_H_ */
/** @} */
