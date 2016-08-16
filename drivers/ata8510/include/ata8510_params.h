/*
 * Copyright (C) 2016 Antonio Galea <antonio.galea@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ata8510
 *
 * @{
 * @file
 * @brief       Default configuration for the ATA8510 driver
 *
 * @author      Antonio Galea <antonio.galea@gmail.com>
 */

#ifndef ATA8510_PARAMS_H
#define ATA8510_PARAMS_H

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Set default configuration parameters for the AT86RF2xx driver
 * @{
 */
#ifndef ATA8510_PARAM_SPI
#define ATA8510_PARAM_SPI         (SPI_0)
#endif
#ifndef ATA8510_PARAM_SPI_SPEED
#define ATA8510_PARAM_SPI_SPEED   (SPI_SPEED_5MHZ)
#endif
#ifndef ATA8510_PARAM_CS
#define ATA8510_PARAM_CS          (GPIO_PIN(0, 0))
#endif
#ifndef ATA8510_PARAM_INT
#define ATA8510_PARAM_INT         (GPIO_PIN(0, 1))
#endif
#ifndef ATA8510_PARAM_SLEEP
#define ATA8510_PARAM_SLEEP       (GPIO_PIN(0, 2))
#endif
#ifndef ATA8510_PARAM_RESET
#define ATA8510_PARAM_RESET       (GPIO_PIN(0, 3))
#endif

#define ATA8510_PARAMS_DEFAULT    {.spi = ATA8510_PARAM_SPI, \
                                     .spi_speed = ATA8510_PARAM_SPI_SPEED, \
                                     .cs_pin = ATA8510_PARAM_CS, \
                                     .int_pin = ATA8510_PARAM_INT, \
                                     .sleep_pin = ATA8510_PARAM_SLEEP, \
                                     .reset_pin = ATA8510_PARAM_RESET}
/**@}*/

/**
 * @brief   AT86RF231 configuration
 */
static const ata8510_params_t ata8510_params[] =
{
#ifdef ATA8510_PARAMS_BOARD
    ATA8510_PARAMS_BOARD,
#else
    ATA8510_PARAMS_DEFAULT,
#endif
};


#define ATA8510_CMD_SETPOLLINGMODE		(0x23)
#define ATA8510_CMD_SETIDLEMODE			(0x00)
#define ATA8510_CMD_READRXFILLLEVEL		(0x01)
#define ATA8510_CMD_READTXFILLLEVEL		(0x02)
#define ATA8510_CMD_READRSSIFILLLEVEL	(0x03)
#define ATA8510_CMD_GETCONFIGEVENTBYTE  (0x04)
#define ATA8510_CMD_GETEVENTBYTE		(0x04)
#define ATA8510_CMD_READRSSIFIFO		(0x05)
#define ATA8510_CMD_READRXFIFO			(0x06)

#define ATA8510_CMD_WRITESRAM			(0x07)
#define ATA8510_CMD_READSRAM			(0x08)
#define ATA8510_CMD_WRITEEEPROM			(0x09)
#define ATA8510_CMD_READEEPROM			(0x0A)
#define ATA8510_CMD_WRITETXFIFO			(0x0B)
#define ATA8510_CMD_WRITETXPREAMBLE		(0x0C)
#define ATA8510_CMD_SETSYSTEMMODE		(0x0D)
#define ATA8510_CMD_GETVERSIONFLASH		(0x13)
#define ATA8510_CMD_RESET               (0x15)
#define ATA8510_CMD_STARTRSSIMSRMNT		(0x1B)
#define ATA8510_CMD_GETRSSIMSRMNT		(0x1C)

#define ATA8510_CMD_READRXFILLLEVEL_LEN		(3)
#define ATA8510_CMD_READTXFILLLEVEL_LEN		(3)
#define ATA8510_CMD_READRSSIFILLLEVEL_LEN	(3)
#define ATA8510_CMD_GETCONFIGEVENTBYTE_LEN	(4)
#define ATA8510_CMD_GETEVENTBYTE_LEN		(4)
#define ATA8510_CMD_WRITEEEPROM_LEN         (4)
#define ATA8510_CMD_READEEPROM_LEN          (5)
#define ATA8510_CMD_SETSYSTEMMODE_LEN		(3)
#define ATA8510_CMD_GETVERSIONFLASH_LEN		(6)
#define ATA8510_CMD_RESET_LEN               (2)
#define ATA8510_CMD_STARTRSSIMSRMNT_LEN		(2)
#define ATA8510_CMD_GETRSSIMSRMNT_LEN       (4)
#define ATA8510_CMD_WRITESRAM_LEN           (4)
#define ATA8510_CMD_READSRAM_LEN            (5)
#define ATA8510_DATA_HEADER		('D')
#define ATA8510_DATA_HEADER_LEN			(1)
#define ATA8510_CHECKSUM_LEN			(1)
#define ATA8510_WriteTxPreambleBuffer_LEN	(9)

						// systemModeConfig:
#define ATA8510_RF_RXPOLLINGMODE	(0x23)	// bit5 VCO tuning before changing OPM enabled,
						// OPM[3] RXPollingMode

						// systemModeConfig:
#define ATA8510_RF_IDLEMODE		(0x20)	// bit5 VCO tuning before changing OPM enabled,
						// OPM[0] IDLEMode

						// systemModeConfig:
#define ATA8510_RF_TXMODE			(0x21)	// bit5 VCO tuning before changing OPM enabled,
						// OPM[1] TXMode

						// systemModeConfig:
#define ATA8510_RF_RXMODE			(0x22)  // bit5 VCO tuning before changing OPM enabled,
						// OPM[2] RXMode

						// serviceChannelConfig:
#define ATA8510_RF_TXSERVICE		(0x40)	// bit6 Enable PathA
						// ch[0] Channel 0
#define ATA8510_RF_RXSERVICE		(0x40)	// bit6 Enable PathA
						// ch[0] Channel 0



#ifdef __cplusplus
}
#endif

#endif /* ATA8510_PARAMS_H */
/** @} */
