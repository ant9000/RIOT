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

#ifdef __cplusplus
}
#endif

#endif /* ATA8510_PARAMS_H */
/** @} */
