/*
 * main.c
 * Initialisation of the threads
 *
 *  Created on: 30. oct. 2018
 *      Author: Moritz Laim
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "hal.h"
#include "ch.h"
#include "memory_protection.h"

#include "usbcfg.h"

#include "leds.h"

#include "can.h"

#include <main.h>


static THD_WORKING_AREA(my_can_rx_thread_wa, 256);
static THD_FUNCTION(my_can_rx_thread, arg)
{
    (void)arg;
    chRegSetThreadName("My CAN rx");

    CANRxFrame rxf;
    uint8_t inter_can = 'B' ;
    uint32_t inter_id ;

    while (1) {
    	msg_t m = canReceive(&CAND1, CAN_ANY_MAILBOX, &rxf, MS2ST(1000));
		if (m != MSG_OK) {
			continue;
		}
		if (rxf.IDE) {
			continue; // no extended id frames
		}
		if (rxf.RTR) {
			continue; // no remote transmission request frames
		}
		chThdSleepMilliseconds(100);
		set_led(LED2,1) ;

		if(rxf.DLC == 1){
			inter_can = rxf.data8[0];
			inter_id = rxf.SID ;

			if (inter_can == '1')
				set_led(LED3,0) ;
			if (inter_can == '0')
				set_led(LED3,1) ;

			chSequentialStreamWrite(&SDU1, (uint8_t *) &inter_id, sizeof(uint32_t));
			chSequentialStreamWrite(&SDU1, &inter_can, sizeof(uint8_t));
		}
		chThdSleepMilliseconds(100);
		set_led(LED2,0) ;
    }
}


void my_can_init(void)
{

 //   can_init() ;
	static const CANConfig can1_config = {
        .mcr = (1 << 6)  /* Automatic bus-off management enabled. */
               | (1 << 2), /* Message are prioritized by order of arrival. */

        /* APB Clock is 42 Mhz
           42MHz / 2 / (1tq + 12tq + 8tq) = 1MHz => 1Mbit */
        .btr = (1 << 0)  /* Baudrate prescaler (10 bits) */
               | (11 << 16)/* Time segment 1 (3 bits) */
               | (7 << 20) /* Time segment 2 (3 bits) */
               | (0 << 24) /* Resync jump width (2 bits) */
    };

    canStart(&CAND1, &can1_config);
}


void send_uint8_t(const uint8_t inter)
{
    CANTxFrame txf;
    txf.DLC = sizeof(inter);
    txf.RTR = 0;
    txf.IDE = 0;
    txf.SID = 'y';

    txf.data8[0] = inter;


    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txf, MS2ST(100));

    chThdSleepMilliseconds(1);
}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    my_can_init() ;
    chThdCreateStatic(my_can_rx_thread_wa,
					  sizeof(my_can_rx_thread_wa),
					  NORMALPRIO + 1,
					  my_can_rx_thread,
					  NULL);

	usb_start();

    uint8_t inter_uart = 'A' ;

    while (1) {
    	chSequentialStreamRead(&SDU1, &inter_uart, sizeof(uint8_t)) ;

    	chThdSleepMilliseconds(100);

    	set_led(LED1,0) ;
    	chSequentialStreamWrite(&SDU1, &inter_uart, sizeof(uint8_t));
    	send_uint8_t(inter_uart) ;

    	// Waits 0,1 second
        chThdSleepMilliseconds(100);
    	set_led(LED1,1) ;
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
