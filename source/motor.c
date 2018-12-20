/*
 * motor.c
 *
 *  Created on: 9 déc. 2018
 *      Author: Moritz Laim
 */
#if 0
#include "motors.h"
#include "leds.h"
#include "motor.h"
#include "connector.h"

#include "CAN_protocol.h"
#include "UART_protocol.h"
#include "message_table.h"

#define RUN_SIM		(1<<0)

#define STOP		0

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
	static int16_t speed ;
	int16_t torque ;
	if(node == MOTOR)
		switch(messageNb)
		{
		case MOTOR_CONFIG_U:
			running = data[0]&RUN_SIM ;
			run_simulation(running) ;
			if (!running)
			{
				right_motor_set_speed(STOP);
				left_motor_set_speed(STOP);
			}
			break;
		case TRUE_MOTOR_SPEED_U:
			speed = data[0] ;
			if(running)
			{
				right_motor_set_speed(speed);
				left_motor_set_speed(speed);
			}
			else
			{
				right_motor_set_speed(STOP);
				left_motor_set_speed(STOP);
			}
			send_on_CAN(SPEED_VALUE, data, MOTOR) ; break;
		case TRUE_MOTOR_TORQUE_U:
			torque = data[0] ;
//			if(abs(data[0])>0)
//				set_led(LED1,1) ;
//			else
//				set_led(LED1,0) ;
//
//			if(abs(data[0])>0xF)
//				set_led(LED3,1) ;
//			else
//				set_led(LED3,0) ;
//
//			if(abs(data[0])>0xFF)
//				set_led(LED5,1) ;
//			else
//				set_led(LED5,0) ;
//
//			if(abs(data[0])>0xFFF)
//				set_led(LED7,1) ;
//			else
//				set_led(LED7,0) ;
			send_on_CAN(TORQUE_VALUE, data, MOTOR) ;break;
		default: break;
		}
}

void treat_CAN_data(uint8_t messageNb, uint16_t data[4], uint8_t node)
{
	if(running && node == MOTOR)
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
