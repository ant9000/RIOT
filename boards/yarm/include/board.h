/*
 * Copyright (C) 2016 Antonio Galea <antonio.galea@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_yarm Acmesystems YARM
 * @ingroup     boards
 * @brief       Support for the Acmesystems YARM board.
 * @{
 *
 * @file
 * @brief       Board specific definitions for the Acmesystems YARM board.
 *
 * @author      Antonio Galea <antonio.galea@gmail.com>
 */

#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name ATA8510 configuration
 *
 * {spi bus, spi speed, cs pin, int pin, reset pin, sleep pin}
 */
#define ATA8510_PARAMS_BOARD        {.spi = SPI_0, \
                                     .spi_speed = SPI_SPEED_100KHZ, \
                                     .cs_pin = GPIO_PIN(PA, 5), \
                                     .int_pin = GPIO_PIN(PA, 14), \
                                     .sleep_pin = GPIO_PIN(PA, 27), \
                                     .reset_pin = GPIO_PIN(PA, 15)}

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /** BOARD_H */
/** @} */
