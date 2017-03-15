/*
 * Copyright (C) 2017 Antonio Galea <antonio.galea@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief   Common header for ata8510 transmission example
 *
 * @author  Antonio Galea <antonio.galea@gmail.com>
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>

#include "ata8510.h"
#include "ata8510_params.h"
#include "net/netdev2.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ATA8510_NUM   (sizeof(ata8510_params) / sizeof(ata8510_params[0]))

extern ata8510_t devs[ATA8510_NUM];

void recv(netdev2_t *dev);
int ifconfig(int argc, char **argv);
int txtsnd(int argc, char **argv);
void print_addr(uint8_t *addr, size_t addr_len);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_H_ */
/** @} */
