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
 * @brief       Example application for ATA8510 network device driver
 *
 * @author      Antonio Galea <antonio.galea@gmail.com>
 *
 * @}
 */

#include <stdio.h>

#include "net/netdev2.h"
#include "shell.h"
#include "shell_commands.h"
#include "thread.h"
#include "xtimer.h"
#include "timex.h"

#include "ps.h"

#include "ata8510.h"
#include "ata8510_params.h"
#include "daisy24.h"
#include "net/ieee802154.h"
#include "net/netdev2.h"

#define ATA8510_NUM   (sizeof(ata8510_params) / sizeof(ata8510_params[0]))
extern ata8510_t devs[ATA8510_NUM];

#define MAX_LINE    (80)

#define ENABLE_DEBUG (0)
#include "debug.h"

#include "msg.h"
#include <string.h>

#define _STACKSIZE      (THREAD_STACKSIZE_DEFAULT + THREAD_EXTRA_STACKSIZE_PRINTF)
#define MSG_TYPE_ISR    (0x3456)

#define LCD_ADDR 0x3E
#define EXT_ADDR 0x3F
daisy24_t _lcd, *lcd;

#define RCV_QUEUE_SIZE  (2)
static msg_t rcv_queue[RCV_QUEUE_SIZE];

void recv(netdev2_t *dev);

static char stack[_STACKSIZE];
static kernel_pid_t _recv_pid;

ata8510_t devs[ATA8510_NUM];

static void _event_cb(netdev2_t *dev, netdev2_event_t event)
{
    if (event == NETDEV2_EVENT_ISR) {
        msg_t msg;

        msg.type = MSG_TYPE_ISR;
        msg.content.ptr = dev;

        if (msg_send(&msg, _recv_pid) <= 0) {
            puts("event_cb gnrc_netdev2: possibly lost interrupt.");
        }
    }
    else {
        switch (event) {
            case NETDEV2_EVENT_RX_COMPLETE:
                recv(dev);
                break;
            case NETDEV2_EVENT_TX_MEDIUM_BUSY:
                puts("event_cb gnrc_netdev2: transfer still pending.");
                break;
            case NETDEV2_EVENT_TX_COMPLETE:
                puts("event_cb gnrc_netdev2: transfer complete.");
                break;
            default:
                printf("Unexpected event received %d\n", event);
                break;
        }
    }
}

void *_recv_thread(void *arg)
{
    msg_init_queue(rcv_queue, RCV_QUEUE_SIZE);
    while (1) {
        msg_t msg;
        msg_receive(&msg);
        if (msg.type == MSG_TYPE_ISR) {
            netdev2_t *dev = msg.content.ptr;
            dev->driver->isr(dev);
        }
        else {
            puts("unexpected message type");
        }
    }
}

// ------------------------------------------------------------------------------

int cmd_send(int argc, char **argv)
{
    ata8510_t *dev = &devs[0]; // acquires the 8510 device handle
    uint8_t n;
    struct iovec vector[1];

    if (argc < 2) {
        printf("usage: %s <message>\n", argv[0]);
        return 1;
    }

    dev->service = 0;
    dev->channel = 0;

    n = strlen(argv[1]);
    printf("Sending %d bytes using service %d:\n%s\n", n, dev->service, argv[1]);

    vector[0].iov_base = argv[1];
    vector[0].iov_len  = n;
    ((netdev2_t *)dev)->driver->send((netdev2_t *)dev, vector, 1);

    return 0;
}

static uint8_t buffer[ATA8510_MAX_PKT_LENGTH];

void recv(netdev2_t *dev)
{
    size_t data_len;
    netdev2_ieee802154_rx_info_t rx_info;
    int i;

    data_len = dev->driver->recv(dev, buffer, sizeof(buffer), &rx_info);
#if ENABLE_DEBUG
    DEBUG(
        "RECV %d bytes on service %d, channel %d:\n",
        data_len,
        ((ata8510_t *)dev)->service,
        ((ata8510_t *)dev)->channel
    );
#endif
    putchar('#');
    for (i = 0; i < data_len; i++) {
        if ((buffer[i] > 0x1F) && (buffer[i] < 0x80)) {
            putchar((char)buffer[i]);
        }
        else {
            putchar('?');
        }
    }
    putchar('\n');
#if ENABLE_DEBUG
    DEBUG("RSSI: %u, dBm: %d\n", rx_info.rssi, ata8510_calc_dbm(rx_info.rssi));
#endif
    if(lcd){
        daisy24_clear(lcd);
        daisy24_set_position(lcd, 0, 0);
        daisy24_write(lcd, (char *)buffer, data_len);
        daisy24_set_position(lcd, 0, 1);
        char msg[16];
        i = sprintf(msg,"dBm: %d", ata8510_calc_dbm(rx_info.rssi));
        msg[i]=0;
        daisy24_set_position(lcd, 0, 1);
        daisy24_write(lcd, msg, i);
    }
}

static const shell_command_t shell_commands[] = {
    { "send", "Send a string through radio", cmd_send },
    { NULL, NULL, NULL }
};

int main(void)
{
    int res;

    puts("\n\r\n\rATA8510 serial repeater");
    xtimer_init();

    for (unsigned i = 0; i < ATA8510_NUM; i++) {
        const ata8510_params_t *p = &ata8510_params[i];
        netdev2_t *dev = (netdev2_t *)(&devs[i]);

        printf("Initializing ATA8510 radio at SPI_%d\n", p->spi);
        ata8510_setup(&devs[i], (ata8510_params_t*) p);
        dev->event_callback = _event_cb;
        dev->driver->init(dev);
        printf("STATE: %d\n", ata8510_get_state(&devs[i]));
    }

    _recv_pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 1,
                              THREAD_CREATE_STACKTEST, _recv_thread, NULL,
                              "recv_thread");

    if (_recv_pid <= KERNEL_PID_UNDEF) {
        puts("Creation of receiver thread failed");
        return 1;
    }

    res = daisy24_init(&_lcd, I2C_0, LCD_ADDR, EXT_ADDR);
    if (res) {
        printf("No LCD found.\n");
        lcd = NULL;
    } else {
        lcd = &_lcd;
        printf("Daisy24 successfully initialized.\n");
        res = daisy24_clear(lcd);
        if (res < 0) {
            printf("Error: cant' clear LCD.\n");
        }
    }

    /* start the shell */
    puts("Initialization successful - starting the shell now");
    puts("\nWelcome to YARM for RIOT test8!");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
