/*
 * motor.c
 *
 *  Created on: 9 déc. 2018
 *      Author: Moritz Laim
 */
#include "motors.h"
#include "motor.h"
#include "connector.h"

#include "CAN_protocol.h"
#include "UART_protocol.h"
#include "message_table.h"

void treat_UART_data(uint8_t messageNb, uint16_t data[4], uint8_t node) ;
void treat_CAN_data(uint8_t messageNb, uint16_t data[4], uint8_t node) ;

void source_init(void)
{
	motors_init() ;

	CAN_protocol_init(treat_CAN_data) ;
	UART_protocol_init(treat_UART_data) ;
}

//local functions
void treat_UART_data(uint8_t messageNb, uint16_t data[4], uint8_t node)
{
	int speed ;
	if(node == MOTOR)
		switch(messageNb)
		{
		case TRUE_MOTOR_SPEED_U:
			speed = data[0] ;
			right_motor_set_speed(speed);
			left_motor_set_speed(speed);
			send_on_CAN(SPEED_VALUE, data, MOTOR) ;
		}
}

void treat_CAN_data(uint8_t messageNb, uint16_t data[4], uint8_t node)
{
	if(node == MOTOR)
		switch(messageNb)
		{
		case SPEED_SETPOINT:
			send_on_UART(SPEED_SETPOINT_U, data, MOTOR) ; break;
		default: break ;
		}
}
