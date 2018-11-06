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

#include <main.h>
#include "connector.h"


int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    connector_init() ;

	usb_start();

    uint8_t inter_uart = 'A' ;

    while (1) {
    	//chSequentialStreamRead(&SDU1, &inter_uart, sizeof(uint8_t)) ;

    	//chThdSleepMilliseconds(100);

    	//set_led(LED1,0) ;
    	//chSequentialStreamWrite(&SDU1, &inter_uart, sizeof(uint8_t));

    	// Waits 0,1 second
        chThdSleepMilliseconds(100);
    	//set_led(LED1,1) ;
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
