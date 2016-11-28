/*
 * Copyright (C) 2016 Antonio Galea
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the Daisy24 I2C 16x2 LCD monitor
 *
 * @author      Antonio Galea <antonio.galea@gmail.com>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "periph_conf.h"
#include "periph/i2c.h"
#include "shell.h"

#define BUFSIZE        (128U)

#define LCD_ADDR 0x3E
#define EXT_ADDR 0x3F

static int i2c_dev = -1;

int cmd_init(int argc, char **argv)
{
    int i, res, dev;
    uint8_t data[2];
    uint8_t lcd_init[] = { 0x38, 0x39, 0x14, 0x72, 0x54, 0x6F, 0x0C };

    /* Initialize I2C_0 at 100kHz */
    dev = 0;
    res = i2c_init_master(dev, 1);
    if (res == -1) {
        puts("Error: Init: Given device not available");
        return 1;
    }
    else if (res == -2) {
        puts("Error: Init: Unsupported speed value SPEED_NORMAL");
        return 1;
    }
    else {
        printf("I2C_0 successfully initialized as master!\n");
        i2c_dev = dev;
    }

    /* LCD init */
    for(i=0;i<sizeof(lcd_init);i++){
      data[0] = 0;
      data[1] = lcd_init[i];
      res = i2c_write_bytes(i2c_dev, LCD_ADDR, data, 2);
    }

    return 0;
}
int cmd_backlight(int argc, char **argv)
{
    int res;
    uint8_t state;

    if (i2c_dev < 0) {
        puts("Error: no I2C device initialized");
        return 1;
    }
    if (argc < 2) {
        puts("Error: not enough arguments given");
        printf("Usage:\n%s [0 | 1]\n", argv[0]);
        return 1;
    }

    state = atoi(argv[1]) ? 16 : 0;
    res = i2c_write_byte(i2c_dev, EXT_ADDR, state);
    if (res < 0) {
        puts("Error: no bytes were written");
        return 1;
    }
    return 0;
}

int cmd_clear(int argc, char **argv)
{
    int res;
    char data[2];

    if (i2c_dev < 0) {
        puts("Error: no I2C device initialized");
        return 1;
    }
    /* LCD clear */
    data[0] = 0;
    data[1] = 0x01;
    res = i2c_write_bytes(i2c_dev, LCD_ADDR, data, 2);
    if (res < 0) {
        puts("Error: no bytes were written");
        return 1;
    }
    return 0;
}

int cmd_set_position(int argc, char **argv)
{
    int res;
    char data[2];
    uint8_t x, y;

    if (i2c_dev < 0) {
        puts("Error: no I2C device initialized");
        return 1;
    }
    if (argc < 3) {
        puts("Error: not enough arguments given");
        printf("Usage:\n%s X Y\n", argv[0]);
        return 1;
    }
    x = atoi(argv[1]);
    y = atoi(argv[2]);

    data[0] = 0;
    data[1] = 0x80 + (y*0x40) + x;
    res = i2c_write_bytes(i2c_dev, LCD_ADDR, data, 2);
    if (res < 0) {
        puts("Error: no bytes were written");
        return 1;
    }
    return 0;
}

int cmd_write(int argc, char **argv)
{
    int res;
    uint8_t i, length;
    uint8_t data[2];

    if (i2c_dev < 0) {
        puts("Error: no I2C device was initialized");
        return 1;
    }
    if (argc < 2) {
        puts("Error: not enough arguments given");
        printf("Usage:\n%s: STRING\n", argv[0]);
        return 1;
    }

    length = strlen(argv[1]); 
    for(i=0;i<length;i++){
        data[0] = 0x40;
        data[1] = argv[1][i];
        res = i2c_write_bytes(i2c_dev, LCD_ADDR, data, 2);
        if (res < 0) {
            puts("Error: no bytes were written");
            return 1;
        }
    }
    return 0;
}

int cmd_button_states(int argc, char **argv)
{
    int res;
    uint8_t state;

    if (i2c_dev < 0) {
        puts("Error: no I2C device initialized");
        return 1;
    }

    state = 0xFF;
    res = i2c_write_byte(i2c_dev, EXT_ADDR, state);
    if (res < 0) {
        puts("Error: no bytes were written");
        return 1;
    }
    res = i2c_read_byte(i2c_dev, EXT_ADDR, &state);
    if (res < 1) {
        puts("Error: no bytes were read");
        return 1;
    }
    state &= 0x0F;
    state ^= 0x0F;
    printf("button states: A=%d B=%d C=%d D=%d\n", state&1?1:0, state&2?1:0, state&8?1:0, state&4?1:0);
    return 0;
}


static const shell_command_t shell_commands[] = {
    { "i", "initialize lcd", cmd_init },
    { "b", "set backlight on or off", cmd_backlight },
    { "c", "clear screen", cmd_clear },
    { "p", "set position", cmd_set_position },
    { "w", "write string", cmd_write },
    { "s", "read button states", cmd_button_states },
    { NULL, NULL, NULL }
};

int main(void)
{
    puts("Test for the Daisy24 I2C 16x2 LCD monitor");

    /* define own shell commands */
    char line_buf[SHELL_DEFAULT_BUFSIZE];


    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
