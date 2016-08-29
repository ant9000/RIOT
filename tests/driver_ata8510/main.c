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
#include "ps.h"

#include "ata8510.h"
#include "od.h"
#include "net/ieee802154.h"
#include "net/netdev2.h"

#include "common.h"

#define MAX_LINE    (80)

/* set intervals */
#define INTERVALTX (5U * SEC_IN_USEC)
#define INTERVALRX 300000U
#define INTERVALRSSI 100000U
#define INTERVALTXRANDMAX 3000000U
#define INTERVALTXRANDMIN 1000000U
#define INTERVALCHECKRX 200000U

#define ENABLE_DEBUG (0)
#include "debug.h"

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
static uint8_t buffer[ATA8510_MAX_PKT_LENGTH];

static const shell_command_t shell_commands[] = {
    { "ifconfig", "Configure netdev2", ifconfig },
    { "txtsnd", "Send IEEE 802.15.4 packet", txtsnd },
    { NULL, NULL, NULL }
};

int lost_interrupts=0;

static void _event_cb(netdev2_t *dev, netdev2_event_t event)
{
    if (event == NETDEV2_EVENT_ISR) {
        msg_t msg;

        msg.type = MSG_TYPE_ISR;
        msg.content.ptr = dev;

        if (msg_send(&msg, _recv_pid) <= 0) {
			lost_interrupts++;
            puts("_isr gnrc_netdev2: possibly lost interrupt.");
        }
    }
    else {
        switch (event) {
            case NETDEV2_EVENT_RX_COMPLETE:
                recv(dev);
                break;
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

void recv(netdev2_t *dev)
{
    size_t data_len;
    netdev2_ieee802154_rx_info_t rx_info;

    putchar('\n');
    data_len = dev->driver->recv(dev, buffer, sizeof(buffer), &rx_info);
    printf("Recv: ");
    od_hex_dump(buffer, data_len, 0);
    printf("txt: ");
    for (int i = 0; i < data_len; i++) {
        if ((buffer[i] > 0x1F) && (buffer[i] < 0x80)) {
            putchar((char)buffer[i]);
        }
        else {
            putchar('?');
        }
        if ((((i + 1) % (MAX_LINE - sizeof("txt: "))) == 1) && i != 0) {
            printf("\n     ");
        }
    }
    printf("\n");
    printf("RSSI: %u, dBm: %d\n\n", rx_info.rssi, ata8510_calc_dbm(rx_info.rssi));
}
// ------------------------------------------------------------------------------


//#define THREADRSSIMEAS
//#define THREADTXRAND
//#define THREADCHECKRXERRORS


#ifdef THREADRSSIMEAS
void *thread_RSSI_meas(void *arg)
{
    (void) arg;
    ata8510_t *dev = &devs[0]; // acquires the 8510 device handle
	uint8_t data[4]={0,0,0,0};
	
    printf("thread RSSI measurement, pid: %" PRIkernel_pid "\n", thread_getpid());

    uint32_t last_wakeup = xtimer_now();

	ata8510_write_sram_register(dev, 0x294, 0x29);  // set RSSI polling to 9 (6.8ms) to enable fast sniffing
    while (1) {
        xtimer_periodic_wakeup(&last_wakeup, INTERVALRSSI);
		ata8510_StartRSSI_Measurement(dev, 0, 0);
		xtimer_usleep(7000);
		ata8510_GetRSSI_Value(dev, data);
		if (data[2] > 80) printf("  RSSI Avg: %03d  Peak: %03d\n", data[2], data[3]);
   }
    return NULL;
}

char thread_RSSI_meas_stack[THREAD_STACKSIZE_MAIN];
#endif

#ifdef THREADTXRAND
#include "random.h"
#include "checksum/fletcher16.h"
#define RAND_SEED 0xC0FFEE
#define ID8510 1   // used as first character transmitted

void *thread_tx_rand(void *arg)     // Still has a problem on the very first message sent: the sniffing is returning zeroes. To be fixed..
{
	// Transmit  a message every .... seconds where first byte is ID8510, then 6 bytes as counter of transmissions sent.
    (void) arg;
    ata8510_t *dev = &devs[0]; // acquires the 8510 device handle
	int numtx = 1;  // counter for trasmissions
	char msg[32];	
	char msg2[32];	
    uint32_t time_between_tx;
	uint8_t checksum;
    uint32_t last_wakeup = xtimer_now();
	uint8_t myturn;
	uint8_t i;

    printf("thread tx rand, pid: %" PRIkernel_pid "\n", thread_getpid());
	random_init(RAND_SEED);

	time_between_tx = 1000000U;
    while (1) {
		xtimer_periodic_wakeup(&last_wakeup, time_between_tx);
		printf("state: %d\n", ata8510_get_state(dev));

		// test if previous transmission has ended well. (now it should not be needed anymore...)
		// This block was useful where there were still SFIFO problems now solved.
		if (ata8510_get_state(dev) != IDLE) {	//	dirty way to reenable a blocked 8510. 
												// To be investigated in the CB the error 
												//	codes in RAM when bit 0x80 of data[0] status is set.
			ata8510_SetIdleMode(dev);  // stay in idle mode
			numtx--; // to repeat transmission of faulty tx msg
			xtimer_usleep(70); // let the 8510 exec the command before issuing another one.
		}

		numtx++;
		sprintf(msg2, "%d%06d_", ID8510, numtx);
		printf("Sending: %s \n", msg2);
		checksum = fletcher16((const uint8_t*)msg2, strlen(msg));
		sprintf(msg,"%s%02x",msg2, checksum);

		// test listen before talk
		do {		
			while (!ata8510_cca(dev)) {
				xtimer_usleep(100);
			}
			myturn = 1;
//			xtimer_usleep(20000);
			// after the end of other transmission, sense channel for a random number of times + 2 
			// if someone else took control, restart waiting with first loop, otherwise start transmit 
			for (i=0; i<(((random_uint32() % 10)+2)*2); i++) {
			    if (!ata8510_cca(dev)) { // if sensing channel occupied abort second loop and restart first loop of sensing
					myturn = 0;
					break;
				}
			}
		} while (myturn == 0);

		ata8510_send(dev, (uint8_t *)msg, strlen(msg), 0, 0, IDLE);  // service 0 channel 0 go to idle mode after Tx
		time_between_tx = 1000000U + (random_uint32() % 3000000 ); // between 1 and 4 s
		last_wakeup = xtimer_now();
   }

    return NULL;
}

char thread_tx_rand_stack[THREAD_STACKSIZE_MAIN];
#endif

#ifdef THREADCHECKRXERRORS

#include "checksum/fletcher16.h"
#define MAXYARMTX 5    // max permitted value is 9 for now
void *thread_check_rx_errors(void *arg)
{
	ata8510_t *dev = &devs[0]; // acquires the 8510 device handle
	int valRSSI=0;
	int valRSSIdBm=0;
	int i;
	int rxcounter[MAXYARMTX+1];  // YARM TX ID start from 1 
	int rxerrors[MAXYARMTX+1];    // YARM TX ID start from 1
	int msgcounter[MAXYARMTX+1];  // YARM TX ID start from 1 
	int rxfirstreceived[MAXYARMTX+1];  // marker for first message received or not 
	int yarmtxidreceived = 0;
	int yarmtxreceivedcounter = 0;
	int power10;
	int tot_errors=0;
	int tot_messages=0;
	int perc_error_integer;
	int perc_error_decimal;
	int time_elapsed;
	int time_elapsed_saved;
	int overflows_time = 0;
	int time_printed = -1;
	int tot_restarts = 0;
	uint32_t start_time;
	uint8_t checksum, checksum_received;
	int discard;
    uint8_t mystate8510, mynextstate8510;
	uint8_t mycount=0;
//	uint8_t data[5];
// 	uint16_t errorcode;
	uint8_t ramdata;
	int stops=0;
		
	printf("thread check rx errors started, pid: %" PRIkernel_pid "\n", thread_getpid());
	uint32_t last_wakeup = xtimer_now();

	start_time = xtimer_now();
	// zero errors and counter current tx value for each YARM Tx
	for (i=0; i<MAXYARMTX+1; i++) {
		rxcounter[i] = 0;
		rxerrors[i] = 0;
		msgcounter[i] = 0;
		rxfirstreceived[i] = 0;
	}
	time_elapsed_saved = 0;

	ramdata = ata8510_read_sram_register(dev, 0x294);
	printf("0x294 = %02x\n",ramdata);

	ata8510_SetPollingMode(dev);
	while (1) {
		xtimer_periodic_wakeup(&last_wakeup, INTERVALCHECKRX);
		uint8_t channel;
		uint8_t service;
		uint8_t len;
		uint8_t Buffer[260];

		if (ata8510_get_message(dev, &len, Buffer, &service, &channel, &valRSSI, &valRSSIdBm) ) {
			mycount = 0;
			printf("RxCheck Msg Rec: %s ",Buffer);  // only if ASCII messages
			printf("RSSI=%d dBm=%d Srvc=%d Ch=%d\n", 
					valRSSI, valRSSIdBm, service, channel);
			// analysis of received message
			tot_messages++;
			discard = 0;
			if (Buffer[0] > '0' && Buffer[0] <= ('0'+MAXYARMTX)) {
				yarmtxidreceived = Buffer[0] - 0x30;  // extract YARM ID
				power10 = 1;	
				yarmtxreceivedcounter = 0;				
				for (i=6; i>=1; i--) {  // extract counter sent by YARM (bytes 1 to 7)
					if (Buffer[i] >= '0' && Buffer[i] <= '9') { // is a digit
						yarmtxreceivedcounter += ((Buffer[i]-0x30) * power10);
						power10 *= 10;
					} else {
						printf("     ERROR: not a digit in %d position of message: %c. Discard Message!\n", 
								i, Buffer[i]);
						discard = 1;						
						break;								
					}
				}
//				printf(" Message extracted: Yarm ID %d:  Counter %d\n", yarmtxidreceived, yarmtxreceivedcounter);
				if (discard == 0) {
					if (rxfirstreceived[yarmtxidreceived] == 1) {
						// check msg length
						if (strlen((char *)Buffer) != 10) {
							printf("     ERROR: wrong message length: Discard Message!\n");
						} else {
							// checksum control
							checksum_received = (Buffer[8]<=0x39 ? (Buffer[8]-0x30)*16 : 
												((Buffer[8]-0x61)+10)*16) + 
												(Buffer[9]<=0x39 ? (Buffer[9]-0x30) : 
												(Buffer[9]-0x61)+10);
							checksum = fletcher16(Buffer, 8);
							if (checksum != checksum_received) {
								printf("     ERROR: wrong checksum received %02x instead of %02x: Discard Message!\n", 
										checksum_received, checksum);
							} else {						
								// length and checksum ok and already received a message from this YARM. We can analyze errors	
								if (yarmtxreceivedcounter == (rxcounter[yarmtxidreceived] + 1)) { // message received is correct 
									rxcounter[yarmtxidreceived]++;
									msgcounter[yarmtxidreceived]++;
								} else {
									if (yarmtxreceivedcounter < rxcounter[yarmtxidreceived]) {
										// YARM TX probably restarted
										printf("     ERROR: YARM TX %d probably restarted. Reset counters for it\n", yarmtxidreceived);
										rxerrors[yarmtxidreceived] = 0;
										tot_restarts++;
									} else {
										// errors happened on this YARM TX								
										rxerrors[yarmtxidreceived] += yarmtxreceivedcounter - rxcounter[yarmtxidreceived] -1;
									}
				   					rxcounter[yarmtxidreceived] = yarmtxreceivedcounter;
								}
							}
						}
					} else {
						// first message received from this YARM: set the counter to this value and skip errors analysis
						rxcounter[yarmtxidreceived] = yarmtxreceivedcounter;
						// mark first message received
						rxfirstreceived[yarmtxidreceived] = 1;	 
					}
				}
				// print statistics
				time_elapsed = (xtimer_now() - start_time) / 1000000;
				if (time_elapsed % 10 == 0 && time_elapsed != time_printed) { // every 10 seconds prints statistics
					time_printed = time_elapsed;
					// there is here still an error on the calc of overflows since the printf at the end is wrong...
					if (time_elapsed < time_elapsed_saved) {
						// overflow at 4295 s
						overflows_time++;
					} else {
						time_elapsed_saved = time_elapsed;
					}
					printf("Errors/msgs: ");
					tot_errors = 0; 
					for (i=1; i<=MAXYARMTX; i++) {
						if (rxfirstreceived[i] == 1) {	
							printf("[%d]: %d/%d  ", i, rxerrors[i], msgcounter[i]);
							tot_errors += rxerrors[i];
						}	
					}
					perc_error_integer = 100*tot_errors/tot_messages;
					perc_error_decimal = 10000*tot_errors/tot_messages - 100*perc_error_integer;

					// the print of Time(s) still is not working well after 4295s
					printf("\nTime(s) %d Msgs: %d Restarts: %d SysErrs %d Errors: %d %%Error: %d.%02d%%\n", 
							time_elapsed + overflows_time*4295, tot_messages, tot_restarts, dev->sys_errors, 
							tot_errors, perc_error_integer, perc_error_decimal);
					mystate8510 = ata8510_get_state(dev);
					mynextstate8510 = ata8510_get_state_after_tx(dev);
					printf("  8510st. %d  nextst. %d mycount %d stops %d\n\n", 
							mystate8510, mynextstate8510, mycount, stops);
				}
			} else {
				printf("     ERROR: wrong YARM TX ID received: %c. Discard Message\n",Buffer[0]);
			} 				
    	} else {
			mystate8510 = ata8510_get_state(dev);
			mycount++;
			if (mycount > 41) { // over 4s
				printf("mycount %d\n",mycount);
				mycount = 0;
/*
				stops++;
				ata8510_GetEventBytes(dev, data);
				if (data[0] & 0x80) {
					// SYS_ERR happened
					errorcode = ata8510_read_error_code(dev);
					printf("     _isr SYS_ERR ! Errorcode = 0x%04x  SysErr=%d  SSM state= %d\n   ",
							errorcode, errorcode>>8, errorcode&0xff);
				}
				printf("Check Ev.St. %02x %02x %02x %02x st8510 %d stops %d\n", 
						data[0], data[1], data[2], data[3], mystate8510, stops);
				if (dev->blocked >0) {
					// probably something went wrong. It is not anymore in polling state
					ata8510_GetEventBytes(dev, data);
					xtimer_usleep(70);
					ata8510_SetIdleMode(dev);
					xtimer_usleep(70);
					ata8510_SetPollingMode(dev);
					printf("Something wrong: blocked %d Set polling mode\n\n\n\n\n", dev->blocked);
				}
*/
			}
		}
	}
	return NULL;
}

char thread_check_rx_errors_stack[THREAD_STACKSIZE_MAIN];
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
	ata8510_t *dev = &devs[0]; // acquires the 8510 device handle

    _recv_pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 1,
                              THREAD_CREATE_STACKTEST, _recv_thread, NULL,
                              "recv_thread");

    if (_recv_pid <= KERNEL_PID_UNDEF) {
        puts("Creation of receiver thread failed");
        return 1;
    }

	int ramdata = ata8510_read_sram_register(dev, 0x294);
	printf("0x294 = %02x\n",ramdata);
	ata8510_SetPollingMode(dev);

#ifdef THREADRSSIMEAS
    printf("[main] Starting RSSI measurement thread...\n");
    thread_create(thread_RSSI_meas_stack, sizeof(thread_RSSI_meas_stack),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            thread_RSSI_meas, NULL, "RSSI_daemon");
#endif

#ifdef THREADTXRAND
    printf("[main] Starting tx rand thread...\n");
    thread_create(thread_tx_rand_stack, sizeof(thread_tx_rand_stack),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            thread_tx_rand, NULL, "tx_rand_daemon");
#endif

#ifdef THREADCHECKRXERRORS
    printf("[main] Starting check rx errors thread...\n");
    thread_create(thread_check_rx_errors_stack, sizeof(thread_check_rx_errors_stack),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            thread_check_rx_errors, NULL, "check_rx_errors_daemon");
#endif

#ifdef FEATURE_PERIPH_RTC
    rtc_init();
#endif

    /* start the shell */
    puts("Initialization successful - starting the shell now");
    puts("\nWelcome to YARM for RIOT test8!");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}

