/*
 * connector.c
 *
 *  Created on: 1 nov. 2018
 *      Author: Moritz Laim
 */

#include <math.h>

#include "hal.h"
#include "ch.h"
#include "usbcfg.h"
#include "can.h"

#include "leds.h"

#include "connector.h"
#include "message_table.h"


#define MY_TYPE						MOTOR

#define ECU_INTEREST_LIST_SIZE		12
#define MOTOR_INTEREST_LIST_SIZE	2
#define SENSOR_INTEREST_LIST_SIZE	5

#define MAX_CAN_MESSAGE_LENGTH		8		//Max message length
#define MAX_UART_MESSAGE_LENGTH		8
#define GetDestenee(dest)			(dest & (1<<7))
#define GetNode(dest) 				((dest & (3<<5))>>5)
#define GetMessageNb(dest)			(dest & 0x1F)

#define REQUEST_TYPE_MSG				0x80
#define REQUEST_TYPE_MSG_ANSWER			(1<<11)
#define SET_TYPES_TREATED_MSG			0x81
#define SET_TYPES_TREATED_MSG_ANSWER	((1<<15)+1)
#define SET_TREAT_ECU					(1<<0)
#define SET_TREAT_MOTOR					(1<<1)
#define SET_TREAT_SENSOR				(1<<2)
#define SET_READOUT						(1<<3)
#define START_OUTPUT					(1<<4)

typedef struct MyCanMessage_struct
{
	MyMessage msg ;
	uint8_t period ; // in ms
} MyCanMessage;

BSEMAPHORE_DECL(canMessagesSem, 0) ;
BSEMAPHORE_DECL(canLockSem, 0) ;

// global message table
//messages are sorted by emitter and period
static MyCanMessage messages[3][16] =
{
		[ECU][SPEED_SETPOINT].msg.id = SETPOINT_ID, 		[ECU][SPEED_SETPOINT].msg.length = SETPOINT_LENGTH, 		[ECU][SPEED_SETPOINT].period = SETPOINT_PERIOD,
		[ECU][TORQUE_SETPOINT].msg.id = SETPOINT_ID,		[ECU][TORQUE_SETPOINT].msg.length = SETPOINT_LENGTH, 		[ECU][TORQUE_SETPOINT].period = SETPOINT_PERIOD,
		[ECU][ECU_STATUS].msg.id = ECU_STATUS_ID, 			[ECU][ECU_STATUS].msg.length = ECU_STATUS_LENGTH, 			[ECU][ECU_STATUS].period = ECU_STATUS_PERIOD,
		[ECU][CONTROL].msg.id = CONTROL_ID, 				[ECU][CONTROL].msg.length = CONTROL_LENGTH, 				[ECU][CONTROL].period = CONTROL_PERIOD ,
		[ECU][ECU_ERROR].msg.id = ECU_ERROR_ID, 			[ECU][ECU_ERROR].msg.length = ECU_ERROR_LENGTH, 			[ECU][ECU_ERROR].period = ECU_ERROR_PERIOD,
		[ECU][ECU_REQUEST].msg.id = ECU_REQUEST_ID, 		[ECU][ECU_REQUEST].msg.length = ECU_REQUEST_LENGTH, 		[ECU][ECU_REQUEST].period = ECU_REQUEST_PERIOD,
		[ECU][BMS_CONTACTOR].msg.id = BMS_CONTACTOR_ID,		[ECU][BMS_CONTACTOR].msg.length = BMS_CONTACTOR_LENGTH, 	[ECU][BMS_CONTACTOR].period = BMS_CONTACTOR_PERIOD,

		[MOTOR][SPEED_VALUE].msg.id = MOTOR_SEND_ID, 		[MOTOR][SPEED_VALUE].msg.length = SPEED_VALUE_LENGTH, 		[MOTOR][SPEED_VALUE].period = SPEED_VALUE_PERIOD,
		[MOTOR][TORQUE_VALUE].msg.id = MOTOR_SEND_ID, 		[MOTOR][TORQUE_VALUE].msg.length = TORQUE_VALUE_LENGTH, 	[MOTOR][TORQUE_VALUE].period = TORQUE_VALUE_PERIOD,
		[MOTOR][MOTOR_STATUS].msg.id = MOTOR_SEND_ID, 		[MOTOR][MOTOR_STATUS].msg.length = MOTOR_STATUS_LENGTH, 	[MOTOR][MOTOR_STATUS].period = MOTOR_STATUS_PERIOD,

		[SENSOR][BREAK_PEDAL].msg.id = BREAK_PEDAL_ID, 		[SENSOR][BREAK_PEDAL].msg.length = BREAK_PEDAL_LENGTH, 		[SENSOR][BREAK_PEDAL].period = BREAK_PEDAL_PERIOD,
		[SENSOR][ACCELERATOR].msg.id = ACCELERATOR_ID, 		[SENSOR][ACCELERATOR].msg.length = ACCELERATOR_LENGTH, 		[SENSOR][ACCELERATOR].period = ACCELERATOR_PERIOD,
		[SENSOR][STEERING].msg.id = STEERING_ID, 			[SENSOR][STEERING].msg.length = STEERING_LENGTH, 			[SENSOR][STEERING].period = STEERING_PERIOD,
		[SENSOR][WHEEL_SPEED].msg.id = WHEEL_SPEED_ID, 		[SENSOR][WHEEL_SPEED].msg.length = WHEEL_SPEED_LENGTH, 		[SENSOR][WHEEL_SPEED].period = WHEEL_SPEED_PERIOD,
		[SENSOR][ACCELERATION].msg.id = ACCELERATION_ID,	[SENSOR][ACCELERATION].msg.length = ACCELERATION_LENGTH,	[SENSOR][ACCELERATION].period = ACCELERATION_PERIOD,
		[SENSOR][ENVIRONMENT].msg.id = ENVIRONMENT_ID, 		[SENSOR][ENVIRONMENT].msg.length = ENVIRONMENT_LENGTH, 		[SENSOR][ENVIRONMENT].period = ENVIRONMENT_PERIOD,
		[SENSOR][COCKPIT].msg.id = COCKPIT_ID, 				[SENSOR][COCKPIT].msg.length = COCKPIT_LENGTH, 				[SENSOR][COCKPIT].period = COCKPIT_PERIOD,
		[SENSOR][BMS_EVENTS].msg.id = BMS_EVENTS_ID, 		[SENSOR][BMS_EVENTS].msg.length = BMS_EVENTS_LENGTH, 		[SENSOR][BMS_EVENTS].period = BMS_EVENTS_PERIOD,
		[SENSOR][SENEOR_STATUS].msg.id = SENEOR_STATUS_ID, 	[SENSOR][SENEOR_STATUS].msg.length = SENEOR_STATUS_LENGTH, 	[SENSOR][SENEOR_STATUS].period = SENEOR_STATUS_PERIOD,
		[SENSOR][SENSOR_ERROR].msg.id = SENSOR_ERROR_ID, 	[SENSOR][SENSOR_ERROR].msg.length = SENSOR_ERROR_LENGTH, 	[SENSOR][SENSOR_ERROR].period = SENSOR_ERROR_PERIOD,
		[SENSOR][SENOR_REQUEST].msg.id = SENOR_REQUEST_ID, 	[SENSOR][SENOR_REQUEST].msg.length = SENOR_REQUEST_LENGTH, 	[SENSOR][SENOR_REQUEST].period = SENOR_REQUEST_PERIOD
};

static const uint16_t ECU_interest_list[12] =
{
	SENSOR_ERROR_ID, SENOR_REQUEST_ID, BREAK_PEDAL_ID, ACCELERATOR_ID, STEERING_ID, WHEEL_SPEED_ID, ACCELERATION_ID, MOTOR_SEND_ID, ENVIRONMENT_ID, COCKPIT_ID, BMS_EVENTS_ID, SENEOR_STATUS_ID
};

static const uint16_t Motor_interest_list[MOTOR_INTEREST_LIST_SIZE] =
{
	SETPOINT_ID, STEERING_ID
};

static const uint16_t Sensor_interest_list[SENSOR_INTEREST_LIST_SIZE] =
{
	CONTROL_ID, ECU_ERROR_ID, ECU_REQUEST_ID, BMS_CONTACTOR_ID, ECU_STATUS_ID
};

void send_my_can(const MyCanMessage out) ;
void send_one_type(uint8_t type, uint8_t periodCounter) ;
void preprocess_can_input(MyMessage input, uint8_t fromUART) ;				//distribute input message
void preprocess_uart_input(MyMessage input) ;

void my_can_init(void) ;

void can_lock(void);
void can_unlock(void);

void messages_table_lock(void);
void messages_table_unlock(void) ;

// Configuration variables
static bool output ;		// send the messages on CAN bus
static bool doReadout ;	// send all CAN messages to UART
// Set
#if MY_TYPE == ECU
static bool treatECU  =  1 ;
static bool treatMotor = 0 ;
static bool treatSensor = 0 ;
#endif
#if MY_TYPE == MOTOR
static bool treatECU  =  0 ;
static bool treatMotor = 1 ;
static bool treatSensor = 0 ;
#endif
#if MY_TYPE == SENSOR
static bool treatECU  =  0 ;
static bool treatMotor = 0 ;
static bool treatSensor = 1 ;
#endif

static MessageToSource toProtocolFP;
static MessageToSource toSourceFP ;
static UpdateTiming TimingUpdateFP;

// threads
// Sends prestored messages on CAN bus with base period 100 ms
static THD_WORKING_AREA(waCANSend, 256) ;
static THD_FUNCTION(CANSend, arg) {

    chRegSetThreadName(__FUNCTION__) ;
    (void)arg;

    systime_t time;
    uint8_t periodCount = 0 ;

    while(1){
    	time = chVTGetSystemTime() ;
    	set_led(LED1,0) ;
    	periodCount %= SCM_PERIODS ;
    	periodCount++ ;				//counter from 1 to SCM_PERIODS to limit value of period_count

    	if(treatECU && output)
    		send_one_type(ECU, periodCount);
    	if(treatMotor&& output)
    		send_one_type(MOTOR, periodCount) ;
    	if(treatSensor && output)
    		send_one_type(SENSOR, periodCount) ;
    	set_led(LED1,1) ;

    	TimingUpdateFP(MY_TYPE) ;

    	chThdSleepUntilWindowed(time, time + MS2ST(200)) ;

    }
}

// check periodically the CAN bus for new messages.
static THD_WORKING_AREA(waCANReceive, 256);
static THD_FUNCTION(CANReceive, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    CANRxFrame rxf;
    MyMessage input ;

    while (1) {
    	set_led(LED3,1) ;
    	msg_t m = canReceive(&CAND1, CAN_ANY_MAILBOX, &rxf, MS2ST(100));
    	set_led(LED3,0) ;

		if (m != MSG_OK)
			continue;
		if (rxf.IDE)
			continue; // no extended id frames
		if (rxf.RTR)
			continue; // no remote transmission request frames


		// reformat information
		input.id = rxf.SID ;
		input.length = rxf.DLC ;
		input.data32[0] = rxf.data32[0] ;
		input.data32[1] = rxf.data32[1] ;

		preprocess_can_input(input, FALSE) ;
    }
}

static THD_WORKING_AREA(waUARTReceive, 256) ;
static THD_FUNCTION(UARTReceive, arg) {

    chRegSetThreadName(__FUNCTION__) ;
    (void)arg;

    uint8_t destination = 0;
    uint8_t length = 0; //in bytes

    MyMessage input;

    chSequentialStreamRead(&SDU1,&destination, sizeof(uint8_t)) ;

    while(1){
    	set_led(LED5,1) ;
    	chSequentialStreamRead(&SDU1,&destination, sizeof(uint8_t)) ;
    	input.id = destination ;
    	set_led(LED5,0) ;
    	chSequentialStreamRead(&SDU1, &(length), sizeof(uint8_t)) ;
    	input.length = length ;
    	set_led(LED5,1) ;
    	if(input.length)											// Maximal value of read data must be strictly positive
    		chSequentialStreamRead(&SDU1, input.data8, input.length) ;
    	set_led(LED5,0) ;

    	preprocess_uart_input(input) ;

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
					  NORMALPRIO-1,
					  UARTReceive,
					  NULL);
	output = 0;
	doReadout = 0 ;

}

void can_send(bool send)
{
	output = send ;
}

void send_to_PC(const MyMessage out)
{
	chSequentialStreamWrite(&SDU1, (uint8_t *) &out.id, sizeof(uint16_t)) ;
	chSequentialStreamWrite(&SDU1, (uint8_t *) &out.length, sizeof(uint8_t)) ;
	chSequentialStreamWrite(&SDU1, (uint8_t *) &out.data8, out.length) ;
}

// write to message table if length is right
void write_to_table(uint8_t messageNb, uint8_t data[8], uint8_t length, uint8_t node)
{
	MyCanMessage localCopy ;

	if(messages[node][messageNb].msg.length == length)
	{
		messages_table_lock() ;

		for(int i=0; i<messages[node][messageNb].msg.length;i++)
			messages[node][messageNb].msg.data8[i] = data[i] ;

		localCopy = messages[node][messageNb] ;

		messages_table_unlock() ;

		if(doReadout)
			send_to_PC(messages[node][messageNb].msg) ;

		if(localCopy.period == NO_REP)
			send_my_can(localCopy) ;

		if(node != MY_TYPE)
			toProtocolFP(localCopy.msg) ;
	}
}

void init_to_protocol(MessageToSource ProtocolFP, UpdateTiming TimingFP)
{
	toProtocolFP = ProtocolFP ;
	TimingUpdateFP = TimingFP ;
}

void init_to_source(MessageToSource SourceFP)
{
	toSourceFP = SourceFP ;
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

    canStart(&CAND1, &can1_config) ;
}

void send_one_type(uint8_t type, uint8_t periodCounter)
{
	uint8_t messageCounter = 0 ;
    MyCanMessage localCopy ;

    // test only the messages with period smaller than period counter
	while(messages[type][messageCounter].period <= periodCounter			//!!
			&& messages[type][messageCounter].period != NO_REP )
	{
		if(!(periodCounter%messages[type][messageCounter].period)) // if period_count is multiple of period
		{
			messages_table_lock() ;

			localCopy = messages[type][messageCounter] ;

			messages_table_unlock() ;

			send_my_can(localCopy) ;
			if (type != MY_TYPE)
				preprocess_can_input(localCopy.msg,true) ;
		}
		messageCounter++ ;
	}
}


void preprocess_can_input(MyMessage input, uint8_t fromUART)
{
	uint8_t i = 0;

	if (doReadout && !fromUART)
		send_to_PC(input) ;

#if MY_TYPE != ECU	//send to PC only the messages of ECU node when needed if it is not already sent
	if(!doReadout && treatECU && !fromUART)
		for (i = 0; i<ECU_INTEREST_LIST_SIZE; i++)
			if(input.id == ECU_interest_list[i])
			{
				send_to_PC(input) ;
				break;
			}
#else				// pass information of the node to the source code
	for (i = 0; i<ECU_INTEREST_LIST_SIZE; i++)
		if(input.id == ECU_interest_list[i])
		{
			//send to protocol code
			toProtocolFP(input) ;
			break;
		}
#endif


#if MY_TYPE != MOTOR
	if(!doReadout && treatMotor && !fromUART)
		for (i = 0; i<MOTOR_INTEREST_LIST_SIZE; i++)
			if(input.id == Motor_interest_list[i])
			{
				send_to_PC(input) ;
				break;
			}
#else

	for (i = 0; i<MOTOR_INTEREST_LIST_SIZE; i++)
		if(input.id == Motor_interest_list[i])
		{
			//send to protocol code
			toProtocolFP(input) ;
			break;
		}
#endif


#if MY_TYPE != SENSOR
	if(!doReadout && treatSensor && !fromUART)
		for (i = 0; i<SENSOR_INTEREST_LIST_SIZE; i++)
			if(input.id == Sensor_interest_list[i])
			{
				send_to_PC(input) ;
				break;
			}
#else
	for (i = 0; i<SENSOR_INTEREST_LIST_SIZE; i++)
		if(input.id == Sensor_interest_list[i])
		{
			//send to protocol code
			toProtocolFP(input) ;
			break;
		}
#endif

}

void preprocess_uart_input(MyMessage input)
{
	MyMessage response ;
	uint8_t node, messageNb;
	if (GetDestenee(input.id)) // destinations are sourcecode (1) or message table (0)
		{
			// pick messages that are important for this node, forward the rest
			if (input.id == REQUEST_TYPE_MSG)	//respond to node type request message
			{
				response.id = REQUEST_TYPE_MSG_ANSWER ;
				response.data8[0] = MY_TYPE ;
				response.length = sizeof(uint8_t) ;

				send_to_PC(response) ;
			}
			else if(input.id == SET_TYPES_TREATED_MSG && input.length == 1) // treat a set treat message
			{
				response.id = SET_TYPES_TREATED_MSG_ANSWER ;
				response.data8[0] = input.data8[0] ;
				response.length = sizeof(uint8_t) ;
				//only adjust nodes value that are different from the type of the node
#if (MY_TYPE != ECU)
				treatECU = input.data8[0]&SET_TREAT_ECU ;
#endif
#if MY_TYPE != MOTOR
				treatMotor = input.data8[0]&SET_TREAT_MOTOR ;
#endif
#if MY_TYPE != SENSOR
				treatSensor = input.data8[0]&SET_TREAT_SENSOR ;
#endif
				// set readout variable
				doReadout = input.data8[0]&SET_READOUT ;

				send_to_PC(response) ;
			}
			else
			{
				//send to source code
				toSourceFP(input) ;
			}
		}
		else	// write to message table if
		{
			node = GetNode(input.id);
			messageNb = GetMessageNb(input.id) ;

			preprocess_can_input(input, true);

			write_to_table(messageNb, input.data8, input.length, node) ;
		}
}

// Format MyCanMessage to CANTxFrame
void send_my_can(const MyCanMessage out)
{
    CANTxFrame txf;
    txf.DLC = out.msg.length;
    txf.RTR = 0;
    txf.IDE = 0;
    txf.SID = out.msg.id;

    txf.data32[0] = out.msg.data32[0] ;
    txf.data32[1] = out.msg.data32[1];

    can_lock() ;
    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txf, MS2ST(10));
    can_unlock() ;
}

void can_lock(void)
{
	chBSemWait(&canLockSem) ;
}

void can_unlock(void)
{
	chBSemSignal(&canLockSem) ;
}

void messages_table_lock(void)
{
	chBSemWait(&canMessagesSem) ;
}

void messages_table_unlock(void)
{
	chBSemSignal(&canMessagesSem) ;
}
