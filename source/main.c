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
#include "ecu.h"
#include "sensor.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    source_init() ;

    connector_init() ;

	usb_start();

	// I tried to use IMU, but did not finish
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

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
