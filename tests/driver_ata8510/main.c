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

static const shell_command_t shell_commands[] = {
    { "ifconfig", "Configure netdev2", ifconfig },
    { "txtsnd", "Send IEEE 802.15.4 packet", txtsnd },
    { NULL, NULL, NULL }
};

int event8510;  // total number of 8510 events occurred
uint8_t ata8510BufferRx[40];
uint8_t ata8510msgreclen;
uint8_t ata8510receivedMsg = 0;
uint8_t ata8510receivedService;
uint8_t ata8510receivedChannel;
uint8_t RSSI=0;
uint8_t dBm=0;
uint8_t data[37];
uint8_t saved8510status[4];
uint8_t dataSFIFO[19];
uint8_t dataRSSI[19];
int syserrors = 0;
uint16_t errorcode;
int lost_interrupts=0;
uint8_t blocked = 0;
int	unknown_case;

static void _isr(netdev2_t *dev){
    uint8_t mystate8510, mynextstate8510;
	uint8_t rxlen, rssilen, sfifolen = 0;;
	uint8_t i;
    ata8510_t *mydev = (ata8510_t *)dev;
	event8510++;
	ata8510_GetEventBytes(mydev, data);
	if (data[0] & 0x80) {
		// SYS_ERR happened
		errorcode = ata8510_read_error_code(mydev);
		DEBUG("     _isr SYS_ERR! %02x %02x %02x %02x Errorcode = 0x%04x  SysErr=%d  SSM state= %d\n",
				data[0], data[1], data[2], data[3], errorcode, errorcode>>8, errorcode&0xff);
		syserrors++;
	}
	mystate8510 = ata8510_get_state(mydev);
	mynextstate8510 = ata8510_get_state_after_tx(mydev);
	saved8510status[0] = data[0];  // save it now since if we print we lost the message	
	saved8510status[1] = data[1];	
	saved8510status[2] = data[2];	
	saved8510status[3] = data[3];	
	if (mystate8510 == 0) mystate8510 = 0;


	switch (mystate8510) {
		case IDLE:
			DEBUG("_isr State IDLE!\n");
		break;
		case TX_ON:
			switch (data[1]&0x10) {
				case 0x10:
				// end of transmission
					DEBUG("End of Transmission!\n");
					ata8510_SetIdleMode(mydev);
					xtimer_usleep(70);
					switch(mynextstate8510) {
						case IDLE:
						break;
						case POLLING:
							ata8510_SetPollingMode(mydev);
							ata8510_set_state(mydev, POLLING);
						break;	
						default:
							printf("_isr Cannot handle state 8510 %d after TX \n", mynextstate8510);
					}
				break;
				default: {
					printf("_isr Unknown events.events %02x [%02x] %02x %02x \n", 
							data[0], data[1], data[2], data[3]);
					unknown_case++;
				}
			DEBUG("_isr State TX_ON!\n");
			}
		break;
		case RX_ON:
			DEBUG("_isr State RX_ON!\n");
		break;
		case POLLING:
			switch (data[0]) {
				case 0x00:
				case 0x02:
				case 0x22:
				case 0x20:
				case 0x80: // SYS_ERR: probably after a wrong reception. For now restart 8510
				case 0x82:	
					// receiving! Even after an error..
					if (data[1] & 0x10) {  // EOT in Rx. Message complete. We can read
						ata8510receivedService = (data[3] & 0x07);
						ata8510receivedChannel = (data[3] & 0x30)>>4;
						rxlen = ata8510_ReadFillLevelRxFIFO(mydev);
						ata8510msgreclen = rxlen;							
						ata8510_ReadRxFIFO(mydev, rxlen, data);
						data[rxlen+3]=0x00; // close in any case the message received
						rssilen=ata8510_ReadFillLevelRSSIFIFO(mydev);
						if (rssilen>4) {
							ata8510_ReadRSSIFIFO(mydev, rssilen, dataRSSI);
							dataRSSI[2]=rssilen; // uses the dummy location to save the length of the RSSI buffer.
						} else {
							// if interrupt SFIFO has emptied the SFIFO get from dataSFIFO some RSSI values
							for (i=3; i< 10; i++)	dataRSSI[i] = dataSFIFO[i];
							dataRSSI[2] = 7;
							for (i=3; i< 10; i++) printf(" %d", dataSFIFO[i]);
							DEBUG("--\n");
						}
						ata8510_SetIdleMode(mydev);
						xtimer_usleep(70);
						ata8510_SetPollingMode(mydev);
						for (i=0; i<rxlen+1; i++) {
							ata8510BufferRx[i] = data[i+3];  // raw copy of received bytes in ata8510BufferRx					
						}
						ata8510receivedMsg = 1; 
						DEBUG("_isr RxLen %d  8510event %d Data Received: %s\n", 
								rxlen, event8510,(char *)&data[3]);
					}
					break;	
				case 0x04:
				case 0x24:
						// SFIFO event
						;
					break;
				default:
						printf("   _isr Unknown case %02x %02x %02x %02x event8510 %d\n", 
								data[0], data[1], data[2], data[3], event8510);
						blocked = 1;
						unknown_case++;
//							ata8510_SetPollingMode(mydev);			
					break;
			DEBUG("_isr State POLLING!\n");
			}
		break;
		case RSSIMEAS:
			DEBUG("_isr State RSSI Measure!\n");
			;
		break;
		default:
			printf("_isr Unknown case: state 8510 = %d\n", mystate8510);
			unknown_case++;
	}

	if (saved8510status[0]&0x04) { 
		// SFIFO event. In any case read the SFIFO
		sfifolen=ata8510_ReadFillLevelRSSIFIFO(mydev);
		DEBUG("_isr rssilen = %d\n",(int)sfifolen);
		if (sfifolen>0) {
			ata8510_ReadRSSIFIFO(mydev, sfifolen, dataSFIFO);
			for (i=0; i< sfifolen; i++) DEBUG(" %d", dataSFIFO[i]);
		}
		DEBUG("\n");
	}

	DEBUG("_isr 8510 Ev. St. %02x %02x %02x %02x ev8510 %d st8510 %d lostint %d unknown %d\n", 
			saved8510status[0], saved8510status[1], saved8510status[2], saved8510status[3], 
			event8510, mystate8510, lost_interrupts, unknown_case);

}

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
            // dev->driver->isr(dev);
            _isr(dev);
            // ps();  // troppi ps. Meglio con il thread ogni 5s..
        }
        else {
            puts("unexpected message type");
        }
    }
}

uint8_t ata8510_get_message(uint8_t *len, uint8_t *bufRx, uint8_t *service, uint8_t *channel, int *rssi, int *dBm){
	uint8_t i;
	if (ata8510receivedMsg == 0) return 0;
	else {
		for (i=0; i<=ata8510msgreclen; i++) {  // copies also the 0x00
			bufRx[i] = ata8510BufferRx[i];
		}
		*len = ata8510msgreclen;
		// calculate RSSI and dBm values
		*rssi=0;
		if (dataRSSI[2] > 0) {
			for (i=0; i<dataRSSI[2]; i++) {
				*rssi+=dataRSSI[i+3];
			}
			*rssi /= dataRSSI[2];
			*dBm = (*rssi>>1) - 135;
			DEBUG(" RSSI values = %d RSSI = %d   dBm = %d\n", dataRSSI[2], *rssi, *dBm);
		} else {
			DEBUG("no RSSI values read\n");
		}
		*service = 	ata8510receivedService;
		*channel = 	ata8510receivedChannel;

		ata8510receivedMsg = 0;
		return 1;
	}
} 

// ------------------------------------------------------------------------------


//#define THREADTX
//#define THREADRX
//#define THREADTRX
//#define THREADRSSIMEAS
//#define THREADTXRAND
#define THREADCHECKRXERRORS
//#define THREADPS5S


#ifdef THREADTX
void *thread_tx(void *arg)
{
    (void) arg;
    ata8510_t *dev = &devs[0]; // acquires the 8510 device handle

    printf("thread tx started, pid: %" PRIkernel_pid "\n", thread_getpid());

    uint32_t last_wakeup = xtimer_now();
    char msg[32];
    sprintf(msg, "Yarm RIOT TX2OK!");
    while (1) {
        xtimer_periodic_wakeup(&last_wakeup, INTERVALTX);
		printf("Message sent: %s\n",msg);
		ata8510_send(dev, (uint8_t *)msg, strlen(msg), 0, 0, IDLE);  // service 0 channel 0
   }
    return NULL;
}

char thread_tx_stack[THREAD_STACKSIZE_MAIN];
#endif

#ifdef THREADRX
void *thread_rx(void *arg)
	{
	(void) arg;
	ata8510_t *dev = &devs[0]; // acquires the 8510 device handle
	int valRSSI=0;
	int valRSSIdBm=0;
	uint8_t channel;
	uint8_t service;
	uint8_t len;
	uint8_t Buffer[40];

	printf("thread rx started, pid: %" PRIkernel_pid "\n", thread_getpid());

	uint32_t last_wakeup = xtimer_now();
	ata8510_SetPollingMode(dev);
	while (1) {
		xtimer_periodic_wakeup(&last_wakeup, INTERVALRX);

		if (ata8510_get_message(&len, Buffer, &service, &channel, &valRSSI, &valRSSIdBm) ) {
			printf("Thread_Rx Message Received: %s\n",Buffer);  // only if ASCII messages
			printf("  RSSI = %d   dBm = %d  service = %d  channel = %d\n", 
					valRSSI, valRSSIdBm, service, channel);
		}
	}
	return NULL;
}

char thread_rx_stack[THREAD_STACKSIZE_MAIN];
#endif

#ifdef THREADTRX
void *thread_trx(void *arg)
{
	(void) arg;
	ata8510_t *dev = &devs[0]; // acquires the 8510 device handle
    char msg[32];

	int valRSSI=0;
	int valRSSIdBm=0;
	uint8_t channel;
	uint8_t service;
	uint8_t len;
	uint8_t Buffer[40];

	printf("thread trx started, pid: %" PRIkernel_pid "\n", thread_getpid());
	uint32_t last_wakeup = xtimer_now();

	ata8510_SetPollingMode(dev);
	while (1) {
		xtimer_periodic_wakeup(&last_wakeup, INTERVALRX);
		if (ata8510_get_message(&len, Buffer, &service, &channel, &valRSSI, &valRSSIdBm) ) {
			printf("Thread_TRx Message Received: %s\n",Buffer);  // only if ASCII messages
			printf("  RSSI = %d   dBm = %d  service = %d  channel = %d\n", valRSSI, valRSSIdBm, service, channel);
			sprintf(msg, "YARM RIOT ACK!");
			ata8510_send(dev, (uint8_t *)msg, strlen(msg), 0, 1, POLLING);  // transmit on service 0 channel 1. After Tx go in Polling
			printf("Thread_TRx Message Sent: %s\n\n",msg);
		}
	}
	return NULL;
}

char thread_trx_stack[THREAD_STACKSIZE_MAIN];
#endif

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
#define ID8510 3   // used as first character transmitted

#define LISTENBEFORETALK
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

#ifdef LISTENBEFORETALK
		uint8_t myturn;
		uint8_t i;
		uint8_t data[4]={0,0,0,0};


		// test listen before talk
		ata8510_write_sram_register(dev, 0x294, 0x27);  // set RSSI polling to 9 (6.8ms) to enable fast sniffing

		do {		
			do {
				// first loop of sensing
				ata8510_StartRSSI_Measurement(dev, 0, 0);  // 0,0 are service and channel
				xtimer_usleep(6000);
				ata8510_GetRSSI_Value(dev, data);
//				printf("RSSI = %d\n",data[2]);
			} while (data[2] > 50 ); // exits when the present transmission by others ends.
			myturn = 1;
//			xtimer_usleep(20000);
			// after the end of other transmission, sense channel for a random number of times + 2 
			// if someone else took control, restart waiting with first loop, otherwise start transmit 
			for (i=0; i<(((random_uint32() % 10)+2)*2); i++) {
				ata8510_StartRSSI_Measurement(dev, 0, 0);
				xtimer_usleep(6000);
				ata8510_GetRSSI_Value(dev, data);

//				printf("   RSSI = %d\n",data[2]);
				if (data[2] > 50) { // if sensing channel occupied abort second loop and restart first loop of sensing
					myturn = 0;
					break;
				}
			}
		} while (myturn == 0);
#endif
		ata8510_write_sram_register(dev, 0x294, 0x2b);  // set RSSI polling back to 11 (27.1ms) to avoid SFIFO 															// interrupts (for transmissions up to 380ms)
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
	(void) arg;
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
	uint8_t data[5];
	uint16_t errorcode;
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

		if (ata8510_get_message(&len, Buffer, &service, &channel, &valRSSI, &valRSSIdBm) ) {
			mycount = 0;
			printf("RxCheck Msg Rec: %s ",Buffer);  // only if ASCII messages
			printf("RSSI=%d dBm=%d Srvc=%d Ch=%d\n", 
					valRSSI, valRSSIdBm, service, channel);
			// analysis of received message
			tot_messages++;
			discard = 0;
			if (ata8510BufferRx[0] > '0' && ata8510BufferRx[0] <= ('0'+MAXYARMTX)) {
				yarmtxidreceived = ata8510BufferRx[0] - 0x30;  // extract YARM ID
				power10 = 1;	
				yarmtxreceivedcounter = 0;				
				for (i=6; i>=1; i--) {  // extract counter sent by YARM (bytes 1 to 7)
					if (ata8510BufferRx[i] >= '0' && ata8510BufferRx[i] <= '9') { // is a digit
						yarmtxreceivedcounter += ((ata8510BufferRx[i]-0x30) * power10);
						power10 *= 10;
					} else {
						printf("     ERROR: not a digit in %d position of message: %c. Discard Message!\n", 
								i, ata8510BufferRx[i]);
						discard = 1;						
						break;								
					}
				}
//				printf(" Message extracted: Yarm ID %d:  Counter %d\n", yarmtxidreceived, yarmtxreceivedcounter);
				if (discard == 0) {
					if (rxfirstreceived[yarmtxidreceived] == 1) {
						// check msg length
						if (strlen((char *)ata8510BufferRx) != 10) {
							printf("     ERROR: wrong message length: Discard Message!\n");
						} else {
							// checksum control
							checksum_received = (ata8510BufferRx[8]<=0x39 ? (ata8510BufferRx[8]-0x30)*16 : 
												((ata8510BufferRx[8]-0x61)+10)*16) + 
												(ata8510BufferRx[9]<=0x39 ? (ata8510BufferRx[9]-0x30) : 
												(ata8510BufferRx[9]-0x61)+10);
							checksum = fletcher16(ata8510BufferRx, 8);
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
							time_elapsed + overflows_time*4295, tot_messages, tot_restarts, syserrors, 
							tot_errors, perc_error_integer, perc_error_decimal);
					mystate8510 = ata8510_get_state(dev);
					mynextstate8510 = ata8510_get_state_after_tx(dev);
					printf("  8510st. %d  nextst. %d lostint %d mycount %d stops %d unknown %d\n\n", 
							mystate8510, mynextstate8510, lost_interrupts, mycount, stops, unknown_case);
				}
			} else {
				printf("     ERROR: wrong YARM TX ID received: %c. Discard Message\n",ata8510BufferRx[0]);
			} 				
		} else {
			mystate8510 = ata8510_get_state(dev);
			mycount++;
			if (mycount > 41) { // over 4s
				printf("mycount %d\n",mycount);
				mycount = 0;
				stops++;
				ata8510_GetEventBytes(dev, data);
				if (data[0] & 0x80) {
					// SYS_ERR happened
					errorcode = ata8510_read_error_code(dev);
					printf("     _isr SYS_ERR ! Errorcode = 0x%04x  SysErr=%d  SSM state= %d\n   ",
							errorcode, errorcode>>8, errorcode&0xff);
				}
				printf("Check Ev.St. %02x %02x %02x %02x ev8510 %d st8510 %d lostint %d stops %d\n", 
						data[0], data[1], data[2], data[3], event8510, mystate8510, lost_interrupts, stops);
				if (blocked >0 || mycount > 41) {
					// probably something went wrong. It is not anymore in polling state
					ata8510_GetEventBytes(dev, data);
					xtimer_usleep(70);
					ata8510_SetIdleMode(dev);
					xtimer_usleep(70);
					ata8510_SetPollingMode(dev);
					printf("Something wrong: blocked %d Set polling mode\n\n\n\n\n", blocked);
				}
			}
		}
	}
	return NULL;
}

char thread_check_rx_errors_stack[THREAD_STACKSIZE_MAIN];
#endif

#ifdef THREADPS5S
void *thread_ps_every_5s(void *arg)
	{
	(void) arg;

	printf("thread ps every 5s started, pid: %" PRIkernel_pid "\n", thread_getpid());

	uint32_t last_wakeup = xtimer_now();
	while (1) {
		xtimer_periodic_wakeup(&last_wakeup, 5000000U);
		ps();
	}
	return NULL;
}

char thread_ps_every_5s_stack[THREAD_STACKSIZE_MAIN];
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

#ifdef THREADTX
    printf("[main] Starting tx thread...\n");
    thread_create(thread_tx_stack, sizeof(thread_tx_stack),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            thread_tx, NULL, "tx_daemon");
#endif

#ifdef THREADRX
    printf("[main] Starting rx thread...\n");
    thread_create(thread_rx_stack, sizeof(thread_rx_stack),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            thread_rx, NULL, "rx_daemon");
#endif


#ifdef THREADTRX
    printf("[main] Starting trx thread...\n");
    thread_create(thread_trx_stack, sizeof(thread_trx_stack),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            thread_trx, NULL, "trx_daemon");
#endif

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

#ifdef THREADPS5S
    printf("[main] Starting ps every 5s thread to look for stack problems...\n");
    thread_create(thread_ps_every_5s_stack, sizeof(thread_ps_every_5s_stack),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            thread_ps_every_5s, NULL, "ps_every_5s_daemon");
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

