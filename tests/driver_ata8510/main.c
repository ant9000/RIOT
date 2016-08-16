/*
 * Copyright (C) 2016 Antonio Galea <antonio.galea@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test application for ATA8510 network device driver
 *
 * @author      Antonio Galea <antonio.galea@gmail.com>
 * @author      Roberto Asquini <bobasquins@gmail.com>
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

#include "common.h"

/* set intervals */
#define INTERVALTX (7U * SEC_IN_USEC)
#define INTERVALRX 300000U
#include "msg.h"


#include <string.h>


#if FEATURE_PERIPH_RTC
#include "periph/rtc.h"
#endif


#define _STACKSIZE      (THREAD_STACKSIZE_DEFAULT + THREAD_EXTRA_STACKSIZE_PRINTF)
#define MSG_TYPE_ISR    (0x3456)

static char stack[_STACKSIZE];
static kernel_pid_t _recv_pid;

ata8510_t devs[ATA8510_NUM];

static const shell_command_t shell_commands[] = {
    { "ifconfig", "Configure netdev2", ifconfig },
    { "txtsnd", "Send IEEE 802.15.4 packet", txtsnd },
    { NULL, NULL, NULL }
};

int event8510;  // total number of 8510 events occurred
uint8_t ata8510BufferRx[260];
uint8_t ata8510BufferTx[260];
uint8_t receivedMsg = 0;
uint8_t data[37];

static void _event_cb(netdev2_t *dev, netdev2_event_t event)
{
    uint8_t mystate8510;
	uint8_t rxlen;
	uint8_t i;
    ata8510_t *mydev = (ata8510_t *)dev;
    if (event == NETDEV2_EVENT_ISR) {
        msg_t msg;

        msg.type = MSG_TYPE_ISR;
        msg.content.ptr = dev;

        if (msg_send(&msg, _recv_pid) <= 0) {
            puts("IRQ_CB gnrc_netdev2: possibly lost interrupt.");
        }

		event8510++;
		ata8510_GetEventBytes(mydev, data);
		mystate8510 = ata8510_get_state(mydev);
		printf("IRQ_CB 8510 Event Status %02x %02x %02x %02x event8510 %d state8510 %d\n", data[0], data[1], data[2], data[3], event8510, mystate8510);
#if 1
		switch (mystate8510) {
			case IDLE:
				printf("IRQ_CB State IDLE!\n");
			break;
			case TX_ON:
				printf("IRQ_CB State TX_ON!\n");
				switch (data[0]) {
					case 0x44:
					case 0x64:
					// end of transmission
						ata8510_SetIdleMode(mydev);
						xtimer_usleep(70);
						ata8510_SetPollingMode(mydev);
						ata8510_set_state(mydev, POLLING);
					break;
					default:
						printf("IRQ_CB Unknown events.system [%02x] %02x %02x %02x \n", data[0], data[1], data[2], data[3]);
				}
			break;
			case RX_ON:
				printf("IRQ_CB State RX_ON!\n");
			break;
			case POLLING:
				printf("IRQ_CB State POLLING!\n");
#if 1
				switch (data[0]) {
					case 0x02:
						// receiving!
						if (data[1] & 0x10) {  // EOT in Rx. Message complete. We can read
							rxlen = ata8510_ReadFillLevelRxFIFO(mydev);
							ata8510_ReadRxFIFO(mydev, rxlen, data);
							data[rxlen+3]=0x00; // close the string received
							ata8510_SetIdleMode(mydev);
							xtimer_usleep(70);
							ata8510_SetPollingMode(mydev);
//							printf("IRQ_CB RxLen %d  8510event %d Data Received: %s\n", rxlen, event8510,(char *)&data[3]);
							for (i=0; i<rxlen+1; i++) {
								ata8510BufferRx[i] = data[i+3];  // raw copy of received bytes in ata8510BufferRx					
							}
							receivedMsg = 1;
						}
						break;			
					default:
							printf("IRQ_CB Unknown case %02x %02x %02x %02x event8510 %d\n", data[0], data[1], data[2], data[3], event8510);
//							ata8510_SetPollingMode(dev);			
						break;
				}
#endif
			break;
			default:
				printf("IRQ_CB Unknown case: state 8510 = %d\n", mystate8510);
		}
#endif
    }
    else {
        switch (event) {
            case NETDEV2_EVENT_RX_COMPLETE:
            {
                recv(dev);

                break;
            }
            default:
                puts("Unexpected event received");
                break;
        }
    }
}

void *_recv_thread(void *arg)
{
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

//#define ROBTHREADTX
//#define ROBTHREADRX
#define ROBTHREADTRX


#ifdef ROBTHREADTX
void *rob_thread_tx(void *arg)
{
    (void) arg;
    ata8510_t *dev = &devs[0]; // acquires the 8510 device handle


    printf("rob thread tx started, pid: %" PRIkernel_pid "\n", thread_getpid());

    uint32_t last_wakeup = xtimer_now();
    char msg[32];

    sprintf(msg, "Yarm RIOT TX2OK!");
		

    while (1) {
//        printf("Rob %" PRIu32 "\n", xtimer_now());
        xtimer_periodic_wakeup(&last_wakeup, INTERVALTX);
		ata8510_send(dev, (uint8_t *)msg, strlen(msg));
   }

    return NULL;
}

char rob_thread_tx_stack[THREAD_STACKSIZE_MAIN];
#endif

#ifdef ROBTHREADRX
void *rob_thread_rx(void *arg)
{
	(void) arg;
	ata8510_t *dev = &devs[0]; // acquires the 8510 device handle
	uint8_t data[4]={0,0,0,0};

	printf("rob thread rx started, pid: %" PRIkernel_pid "\n", thread_getpid());
	uint32_t last_wakeup = xtimer_now();

// ROB this while should be put in init 8510 since you connaot start polling mode if 8510 is not telling it is ready (first byte = 0x20).
	while(1) {
		xtimer_periodic_wakeup(&last_wakeup, INTERVALRX);
		ata8510_GetEventBytes(dev, data);
		if (data[0] == 0x20) {  // wait for 8510 to be ready
			xtimer_usleep(70);
			ata8510_SetPollingMode(dev);  // enter in polling mode			
			break;
		}
    }
// ROB

	while (1) {
		xtimer_periodic_wakeup(&last_wakeup, INTERVALRX);

		// simple semaphore for testing
		if (receivedMsg) {
			printf("Rob_Thread_Rx Message Received: %s\n\n",ata8510BufferRx);
			receivedMsg = 0;
		}

	}
	return NULL;
}

char rob_thread_rx_stack[THREAD_STACKSIZE_MAIN];
#endif

#ifdef ROBTHREADTRX
void *rob_thread_trx(void *arg)
{
	(void) arg;
	ata8510_t *dev = &devs[0]; // acquires the 8510 device handle
	uint8_t data[4]={0,0,0,0};
    char msg[32];

	printf("rob thread trx started, pid: %" PRIkernel_pid "\n", thread_getpid());
	uint32_t last_wakeup = xtimer_now();

// ROB this while should be put in init 8510 since you connaot start polling mode if 8510 is not telling it is ready (first byte = 0x20).
	while(1) {
		xtimer_periodic_wakeup(&last_wakeup, INTERVALRX);
		ata8510_GetEventBytes(dev, data);
		if (data[0] == 0x20) {  // wait for 8510 to be ready
			xtimer_usleep(70);
			ata8510_SetPollingMode(dev);  // enter in polling mode			
			break;
		}
    }
// ROB

	while (1) {
		xtimer_periodic_wakeup(&last_wakeup, INTERVALRX);

		// simple semaphore for testing
		if (receivedMsg) {
			printf("Rob_Thread_TRx Message Received: %s\n\n",ata8510BufferRx);
			receivedMsg = 0;
			sprintf(msg, "Yarm RIOT ACK!");
			ata8510_send(dev, (uint8_t *)msg, strlen(msg));
			printf("Rob_Thread_TRx Message Sent: %s\n\n",msg);
		}

	}
	return NULL;
}

char rob_thread_trx_stack[THREAD_STACKSIZE_MAIN];
#endif




int main(void)
{
    puts("\n\r\n\rATA8510 device driver test");
    xtimer_init();

    for (unsigned i = 0; i < ATA8510_NUM; i++) {
        const ata8510_params_t *p = &ata8510_params[i];
        netdev2_t *dev = (netdev2_t *)(&devs[i]);

        printf("Initializing ATA8510 radio at SPI_%d\n", p->spi);
        ata8510_setup(&devs[i], (ata8510_params_t*) p);
        dev->event_callback = _event_cb;
        dev->driver->init(dev);
    }

    _recv_pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 1,
                              THREAD_CREATE_STACKTEST, _recv_thread, NULL,
                              "recv_thread");

    if (_recv_pid <= KERNEL_PID_UNDEF) {
        puts("Creation of receiver thread failed");
        return 1;
    }

#ifdef ROBTHREADTX
    printf("[main] Starting rob tx thread...\n");
    thread_create(rob_thread_tx_stack, sizeof(rob_thread_tx_stack),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            rob_thread_tx, NULL, "tx_daemon");
#endif

#ifdef ROBTHREADRX
    printf("[main] Starting rob rx thread...\n");
    thread_create(rob_thread_rx_stack, sizeof(rob_thread_rx_stack),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            rob_thread_rx, NULL, "rx_daemon");
#endif


#ifdef ROBTHREADTRX
    printf("[main] Starting rob trx thread...\n");
    thread_create(rob_thread_trx_stack, sizeof(rob_thread_trx_stack),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            rob_thread_trx, NULL, "trx_daemon");
#endif


#ifdef FEATURE_PERIPH_RTC
    rtc_init();
#endif


    /* start the shell */
    puts("Initialization successful - starting the shell now");
    puts("Welcome to RIOT Rob4!");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
