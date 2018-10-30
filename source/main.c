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

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include "chprintf.h"
#include "ch.h"

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();
    char inter ;

    while (1) {
    	chSequentialStreamRead(&SDU1, inter, sizeof(char)) ;
    	chThdSleepMilliseconds(100);

    	chSequentialStreamWrite(&SDU1, inter, sizeof(char));
    	// Waits 0,1 second
        chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
