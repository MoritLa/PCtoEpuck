/*
 * motor.c
 *
 *  Created on: 9 déc. 2018
 *      Author: Moritz Laim
 */
#if 1
#include "motors.h"
#include "motor.h"
#include "connector.h"

#include "CAN_protocol.h"
#include "UART_protocol.h"
#include "message_table.h"

bool running ;

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
	int16_t speed ;
	if(node == MOTOR)
		switch(messageNb)
		{
		case MOTOR_CONFIG_U:
			running = data[0]&RUN_SIM ;
			run_simulation(running) ; break;
		case TRUE_MOTOR_SPEED_U://stop when not running
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
		case TORQUE_SETPOINT:
			send_on_UART(TORQUE_SETPOINT_U, data, MOTOR) ; break;
		default: break ;
		}
}
#endif
