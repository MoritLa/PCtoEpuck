/*
 * connector.c
 *
 *  Created on: 1 nov. 2018
 *      Author: Moritz Laim
 */

#include "connector.h"

#define MCU		0
enum MCU_messages{SPEED_SETPOINT, TORQUE_SETPOINT, MCU_STATUS,
					CONTROL, MCU_ERROR, MCU_REQUEST, BMS_CONTACTOR} ;
#define MOTOR	1
enum Motor_messages{SPEED_VALUE, TORQUE_VALUE, MOTOR_STATUS,
						MOTOR_ERROR,MOTOR_REQUEST} ;
#define SENSOR	2
enum Sensor_messages{BREAK_PEDAL, ACCELERATOR, STEERING, WHEEL_SPEED, ACCELERATION,
						ENVIRONMENT, COCKPIT, SENEOR_STATUS,
						SENSOR_ERROR, SENOR_REQUEST} ;

// global message table

typedef struct MyCanMessage
{
	uint16_t id ;
	uint8_t length ;
	uint8_t data[8] ;
	uint8_t period ;
} MyCanMessage;

static MyCanMessage messages[3][16] =
{
		[MCU][SPEED_SETPOINT].id = 0x181, 	[MCU][SPEED_SETPOINT].length = 1, 	[MCU][SPEED_SETPOINT].period = 1,
		[MCU][TORQUE_SETPOINT].id = 0x181,	[MCU][TORQUE_SETPOINT].length = 3, 	[MCU][SPEED_SETPOINT].period = 1,
		[MCU][MCU_STATUS].id = 0x3D0, 		[MCU][MCU_STATUS].length = 1, 		[MCU][MCU_STATUS].period = 100,
		[MCU][CONTROL].id = 0x100, 			[MCU][CONTROL].length = 1, 			[MCU][CONTROL].period = 0 ,
		[MCU][MCU_ERROR].id = 0x101, 		[MCU][MCU_ERROR].length = 1, 		[MCU][MCU_ERROR].period = 0,
		[MCU][MCU_REQUEST].id = 0x111, 		[MCU][MCU_REQUEST].length = 1, 		[MCU][MCU_REQUEST].period = 0,
		[MCU][BMS_CONTACTOR].id = 0x331,	[MCU][BMS_CONTACTOR].length = 1, 	[MCU][MCU_ERROR].period = 0
};
