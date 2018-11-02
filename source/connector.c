/*
 * connector.c
 *
 *  Created on: 1 nov. 2018
 *      Author: Moritz Laim
 */

#include <math.h>
#include <stdlib.h>

#include "hal.h"
#include "ch.h"
#include "usbcfg.h"
#include "can.h"

#include "connector.h"


#define MCU		0
enum MCU_messages{SPEED_SETPOINT, TORQUE_SETPOINT, MCU_STATUS,
					CONTROL, MCU_ERROR, MCU_REQUEST, BMS_CONTACTOR} ;
#define MOTOR	1
enum Motor_messages{SPEED_VALUE, TORQUE_VALUE, MOTOR_STATUS} ;
#define SENSOR	2
enum Sensor_messages{BREAK_PEDAL, ACCELERATOR, STEERING, WHEEL_SPEED, ACCELERATION,
						ENVIRONMENT, COCKPIT, BMS_EVENTS, SENEOR_STATUS,
						SENSOR_ERROR, SENOR_REQUEST} ;

#define MY_TYPE		MCU
#define NO_REP		0
#define SCM_PERIODS	100 //ms


#define MCU_INTEREST_LIST_SIZE		12
#define MOTOR_INTEREST_LIST_SIZE	1
#define SENSOR_INTEREST_LIST_SIZE	5

#define MAX_MESSAGE_LENGTH		8		//Max CAN length
#define GetDestenee(dest)	(dest & (1<<7))
#define GetNode(dest) 		((dest & (11<<5))>>5)
#define GetMessageNb(dest)	(dest & 0x3F)

// global message table
typedef struct MyMessage_struct
{
	uint16_t id ;
	uint8_t length ;
	union{
		uint8_t data_8[8] ;
		uint16_t data_16[4] ;
		uint32_t data_32[2];
	} ;
} MyMessage;

typedef struct MyCanMessage_struct
{
	MyMessage msg ;
	uint8_t period ; // in ms
} MyCanMessage;

BSEMAPHORE_DECL(can_messages_sem, TRUE) ;
BSEMAPHORE_DECL(can_lock_sem, TRUE) ;

//messages are sorted by sender and period
static MyCanMessage messages[3][16] =
{
		[MCU][SPEED_SETPOINT].msg.id = 0x181, 	[MCU][SPEED_SETPOINT].msg.length = 3, 	[MCU][SPEED_SETPOINT].period = 1,
		[MCU][TORQUE_SETPOINT].msg.id = 0x181,	[MCU][TORQUE_SETPOINT].msg.length = 3, 	[MCU][TORQUE_SETPOINT].period = 1,
		[MCU][MCU_STATUS].msg.id = 0x3D0, 		[MCU][MCU_STATUS].msg.length = 1, 		[MCU][MCU_STATUS].period = 100,
		[MCU][CONTROL].msg.id = 0x100, 			[MCU][CONTROL].msg.length = 1, 			[MCU][CONTROL].period = NO_REP ,
		[MCU][MCU_ERROR].msg.id = 0x101, 		[MCU][MCU_ERROR].msg.length = 1, 		[MCU][MCU_ERROR].period = NO_REP,
		[MCU][MCU_REQUEST].msg.id = 0x111, 		[MCU][MCU_REQUEST].msg.length = 1, 		[MCU][MCU_REQUEST].period = NO_REP,
		[MCU][BMS_CONTACTOR].msg.id = 0x331,	[MCU][BMS_CONTACTOR].msg.length = 1, 	[MCU][BMS_CONTACTOR].period = NO_REP,

		[MOTOR][SPEED_VALUE].msg.id = 0x27F, 	[MOTOR][SPEED_VALUE].msg.length = 3, 	[MOTOR][SPEED_VALUE].period = 10,
		[MOTOR][TORQUE_VALUE].msg.id = 0x27F, 	[MOTOR][TORQUE_VALUE].msg.length = 3, 	[MOTOR][TORQUE_VALUE].period = 10,
		[MOTOR][MOTOR_STATUS].msg.id = 0x27F, 	[MOTOR][MOTOR_STATUS].msg.length = 5, 	[MOTOR][MOTOR_STATUS].period = 100,

		[SENSOR][BREAK_PEDAL].msg.id = 0x1B0, 	[SENSOR][BREAK_PEDAL].msg.length = 2, 	[SENSOR][BREAK_PEDAL].period = 1,
		[SENSOR][ACCELERATOR].msg.id = 0x1B1, 	[SENSOR][ACCELERATOR].msg.length = 2, 	[SENSOR][ACCELERATOR].period = 1,
		[SENSOR][STEERING].msg.id = 0x1F0, 		[SENSOR][STEERING].msg.length = 2, 		[SENSOR][STEERING].period = 10,
		[SENSOR][WHEEL_SPEED].msg.id = 0x1F1, 	[SENSOR][WHEEL_SPEED].msg.length = 4, 	[SENSOR][WHEEL_SPEED].period = 10,
		[SENSOR][ACCELERATION].msg.id = 0x1F2, 	[SENSOR][ACCELERATION].msg.length = 4, 	[SENSOR][ACCELERATION].period = 10,
		[SENSOR][ENVIRONMENT].msg.id = 0x2C0, 	[SENSOR][ENVIRONMENT].msg.length = 2, 	[SENSOR][ENVIRONMENT].period = 50,
		[SENSOR][COCKPIT].msg.id = 0x2C1, 		[SENSOR][COCKPIT].msg.length = 2, 		[SENSOR][COCKPIT].period = 10,
		[SENSOR][BMS_EVENTS].msg.id = 0x335, 	[SENSOR][BMS_EVENTS].msg.length = 2, 	[SENSOR][BMS_EVENTS].period = 10,
		[SENSOR][SENEOR_STATUS].msg.id = 0x3D1, [SENSOR][SENEOR_STATUS].msg.length = 1, [SENSOR][SENEOR_STATUS].period = 100,
		[SENSOR][SENSOR_ERROR].msg.id = 0x102, 	[SENSOR][SENSOR_ERROR].msg.length = 2, 	[SENSOR][SENSOR_ERROR].period = NO_REP,
		[SENSOR][SENOR_REQUEST].msg.id = 0x112, [SENSOR][SENOR_REQUEST].msg.length = 2, [SENSOR][SENOR_REQUEST].period = NO_REP
};

static const uint16_t MCU_interest_list[12] =
{
	0x102, 0x112, 0x1B0, 0x1B1, 0x1F0, 0x1F1, 0x1F2, 0x27F, 0x2C0, 0x2C1, 0x335, 0x3D1
};

static const uint16_t Motor_interest_list[MOTOR_INTEREST_LIST_SIZE] =
{
	0x181
};

static const uint16_t Sensor_interest_list[SENSOR_INTEREST_LIST_SIZE] =
{
	0x100, 0x101, 0x111, 0x331, 0x3D0
};

void send_my_can(const MyCanMessage out) ;
void send_one_type(uint8_t type, uint8_t period_counter) ;

void my_can_init(void) ;
void can_lock(void);
void can_unlock(void);
void send_to_PC(const MyMessage out) ;
void messages_table_lock(void);
void messages_table_unlock(void) ;


static bool output ;
static bool treat_MCU ;
static bool treat_Motor ;
static bool treat_Sensor ;

// threads
static THD_WORKING_AREA(waCANSend, 256);
static THD_FUNCTION(CANSend, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    uint8_t period_count = 0 ;

    while(1){
    	time = chVTGetSystemTime();

    	period_count %= SCM_PERIODS ;
    	period_count++ ;				//counter from 1 to SCM_PERIODS

    	if(treat_MCU && output)
    		send_one_type(MCU, period_count);
    	if(treat_Motor&& output)
    		send_one_type(MOTOR, period_count) ;
    	if(treat_Sensor && output)
    		send_one_type(SENSOR, period_count) ;

    	chThdSleepUntilWindowed(time, time + MS2ST(1));

    }
}

static THD_WORKING_AREA(waCANReceive, 256);
static THD_FUNCTION(CANReceive, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    CANRxFrame rxf;
    MyCanMessage input ;
    uint8_t i ;

    while (1) {
    	msg_t m = canReceive(&CAND1, CAN_ANY_MAILBOX, &rxf, MS2ST(10));
		if (m != MSG_OK) {
			continue;
		}
		if (rxf.IDE) {
			continue; // no extended id frames
		}
		if (rxf.RTR) {
			continue; // no remote transmission request frames
		}

		input.msg.id = rxf.SID ;
		input.msg.length = rxf.DLC ;
		input.msg.data_32[0] = rxf.data32[0] ;
		input.msg.data_32[1] = rxf.data32[1] ;

		if(treat_MCU)
			for (i = 0; i<MCU_INTEREST_LIST_SIZE; i++)
				if(rxf.SID == MCU_interest_list[i])
				{
#if MY_TYPE != MCU
					send_to_PC(input.msg) ;
#else
					//send to source code
#endif
					break;
				}
		if(treat_Motor)
			for (i = 0; i<MOTOR_INTEREST_LIST_SIZE; i++)
				if(rxf.SID == Motor_interest_list[i])
				{
#if MY_TYPE != MOTOR
					send_to_PC(input.msg) ;
#else
					//send to source code
#endif
					break;
				}
		if(treat_Sensor)
			for (i = 0; i<SENSOR_INTEREST_LIST_SIZE; i++)
				if(rxf.SID == Sensor_interest_list[i])
				{
#if MY_TYPE != SENSOR
					send_to_PC(input.msg) ;
#else
					//send to source code
#endif
					break;
				}
    }
}

static THD_WORKING_AREA(waUARTReceive, 256);
static THD_FUNCTION(UARTReceive, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    uint8_t destination, node, message_nb ;
    uint8_t length ; //in bytes
    uint8_t data[MAX_MESSAGE_LENGTH] ;

    while(1){

    	chSequentialStreamRead(&SDU1, &destination, sizeof(uint8_t)) ;
    	chSequentialStreamRead(&SDU1, &length, sizeof(uint8_t)) ;
    	chSequentialStreamRead(&SDU1, data, length) ;

    	if (GetDestenee(destination))
    	{
    		//send to source code
    	}
    	else
    	{
    		node = GetNode(destination);
    		message_nb = GetMessageNb(destination) ;
    		if(messages[node][message_nb].msg.length == length)
    		{
    			messages_table_lock() ;
    			for(int i = 0; i<8; i++)
    				messages[node][message_nb].msg.data_8[i] = data[i] ;
    			messages_table_unlock() ;
    		}
    	}
    }
}

// exported functions
void connector_init(void)
{
	my_can_init() ;

	chThdCreateStatic(waCANSend,
					  sizeof(waCANSend),
					  NORMALPRIO,
					  CANSend,
					  NULL);
	chThdCreateStatic(waCANReceive,
					  sizeof(waCANReceive),
					  NORMALPRIO-1,
					  CANReceive,
					  NULL);
	chThdCreateStatic(waUARTReceive,
					  sizeof(waUARTReceive),
					  NORMALPRIO-2,
					  UARTReceive,
					  NULL);

#if MY_TYPE == MCU
	treat_MCU  =  1 ;
#elif MY_TYPE == MOTOR
	treat_Motor  = 1 ;
#elif MY_TYPE == SENSOR
	treat_Sensor  = 1 ;
#endif
}

// local functions
void my_can_init(void)
{
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

void send_one_type(uint8_t type, uint8_t period_counter)
{
	uint8_t message_counter = 0 ;
    MyCanMessage local_copy ;

	while(messages[type][message_counter].period <= period_counter
			&& messages[type][message_counter].period != NO_REP )
	{
		if(!(period_counter%messages[type][message_counter].period)) // if period_count is multiple of period
		{
			messages_table_lock() ;

			local_copy = messages[type][message_counter] ;
			messages_table_unlock() ;

			send_my_can(local_copy) ;
		}
	}
}

void send_my_can(const MyCanMessage out)
{
    CANTxFrame txf;
    txf.DLC = out.msg.length;
    txf.RTR = 0;
    txf.IDE = 0;
    txf.SID = out.msg.id;

    txf.data32[0] = out.msg.data_32[0] ;
    txf.data32[1] = out.msg.data_32[1];

    can_lock() ;
    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txf, MS2ST(10));
    can_unlock() ;
}

void send_to_PC(const MyMessage out)
{
	chSequentialStreamWrite(&SDU1, (uint8_t *) &out.id, sizeof(uint16_t));
	chSequentialStreamWrite(&SDU1, (uint8_t *) &out.length, sizeof(uint8_t));
	chSequentialStreamWrite(&SDU1, (uint8_t *) &out.data_8, out.length);
}

void can_lock(void)
{
	chBSemWait(&can_lock_sem) ;
}

void can_unlock(void)
{
	chBSemSignal(&can_lock_sem) ;
}

void messages_table_lock(void)
{
	chBSemWait(&can_messages_sem) ;
}

void messages_table_unlock(void)
{
	chBSemWait(&can_messages_sem) ;
}
