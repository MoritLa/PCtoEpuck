/*
 * connector.h
 *
 *  Created on: 1 nov. 2018
 *      Author: Moritz Laim
 */

#ifndef CONNECTOR_H
#define CONNECTOR_H

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

#define MCU		0
enum MCU_messages{SPEED_SETPOINT, TORQUE_SETPOINT, MCU_STATUS,
					CONTROL, MCU_ERROR, MCU_REQUEST, BMS_CONTACTOR} ;
#define MOTOR	1
enum Motor_messages{SPEED_VALUE, TORQUE_VALUE, MOTOR_STATUS} ;
#define SENSOR	2
enum Sensor_messages{BREAK_PEDAL, ACCELERATOR, STEERING, WHEEL_SPEED, ACCELERATION,
						ENVIRONMENT, COCKPIT, BMS_EVENTS, SENEOR_STATUS,
						SENSOR_ERROR, SENOR_REQUEST} ;

void connector_init(void) ;

void can_send(bool send) ;
void send_to_PC(const MyMessage out) ;
void write_to_table(uint8_t message_nb, uint8_t data[8]) ;

#endif /* CONNECTOR_H */
