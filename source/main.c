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

#include "motor.h"
#include "sensor.h"
#include "ecu.h"


int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    connector_init() ;


	usb_start();

    source_init() ;

    while (1) {
        chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
