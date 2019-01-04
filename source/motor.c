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

#define SPEED_CONVERT	30
#define STEERING_OFFSET 0x7FFF
#define SPEED_MAX		0x7FFF

static bool running ;

static int16_t delta_v ;

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
			// start/stop running the simulation
			running = data[0]&RUN_SIM ;
			run_simulation(running) ;
			if (!running)
			{
				right_motor_set_speed(STOP);
				left_motor_set_speed(STOP);
			}
			break;
		case TRUE_MOTOR_SPEED_U:
			// set speed value of the wheels
			speed = data[0]/ SPEED_CONVERT;
			if(running)
			{
				right_motor_set_speed(speed-((int32_t)speed)*delta_v/SPEED_MAX);
				left_motor_set_speed(speed+((int32_t)speed)*delta_v/SPEED_MAX);
			}
			else
			{
				right_motor_set_speed(STOP);
				left_motor_set_speed(STOP);
			}
			send_on_CAN(SPEED_VALUE, data, MOTOR) ; break;
		case TRUE_MOTOR_TORQUE_U:
			torque = data[0] ;
//// show torque with LEDs, but LEDs are used otherwise
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
		// send Speed and torque setpoints on UART
		case SPEED_SETPOINT:
			send_on_UART(SPEED_SETPOINT_U, data, MOTOR) ; break;
		case TORQUE_SETPOINT:
			send_on_UART(TORQUE_SETPOINT_U, data, MOTOR) ; break;
		// set the offset for the wheelspeed that makes the robot turn
		case STEERING:
			delta_v = (((int16_t) data[0])-STEERING_OFFSET); break ;
		default: break ;
		}
}
#endif
