/*
 * ecu.c
 *
 *  Created on: 16 déc. 2018
 *      Author: Moritz Laim
 */
#if 0
#include "stdbool.h"

#include "ecu.h"
#include "motor.h"
#include "connector.h"

#include "CAN_protocol.h"
#include "UART_protocol.h"
#include "message_table.h"

#define RUN_SIM		(1<<0)

#define MAX_SPEED_VAL	0x7FFF
#define SPEED_OFFSET	0x7FFF
#define MAX_SPEED		0xE8B9
#define MAX_WHEEL_SPEED	0xBEAA
#define SPEED_RED		2
#define SLIP_SPEED		100

#define MAX_BRAKE_VAL	0xFFFF
#define TORQUE_OFFSET	0x7FFF
#define BRAKE_THRESHOLD	(MAX_BRAKE_VAL/20)
#define NO_TORQUE		0

enum{RL, RR, FL, FR} ;

uint16_t max_speed = MAX_SPEED;
bool brake_override = 0 ;
bool running ;

void treat_UART_data(uint8_t messageNb, uint16_t data[4], uint8_t node) ;
void treat_CAN_data(uint8_t messageNb, uint16_t data[4], uint8_t node) ;

void source_init(void)
{
	CAN_protocol_init(treat_CAN_data) ;
	UART_protocol_init(treat_UART_data) ;
}

//local functions
void treat_UART_data(uint8_t messageNb, uint16_t data[4], uint8_t node)
{
	if(node == ECU)
		switch(messageNb)
		{
		// start simulation
		case ECU_CONFIG_U:
			running = data[0]&RUN_SIM ;
			run_simulation(running) ; break;
		default: break;
		}
}

void treat_CAN_data(uint8_t messageNb, uint16_t data[4], uint8_t node)
{
	int16_t speed[4];
	int32_t mean_speed;
	int16_t setpoint ;
	uint16_t outData[4] ;

	if(running && node == ECU)
		switch(messageNb)
		{
		case WHEEL_SPEED:
			// transform message value to speed value
			for(uint8_t i = 0; i<4;i++)
			{
				speed[i] = ((int16_t)data[i])-SPEED_OFFSET;
				speed[i] = speed[i]/2 ;
			}
			mean_speed = speed[FL]+speed[FR];
			// add slip speed
			if(mean_speed*MAX_WHEEL_SPEED/MAX_SPEED_VAL+SLIP_SPEED>=MAX_SPEED)
				setpoint = MAX_SPEED ;
			else
				setpoint = mean_speed*MAX_WHEEL_SPEED/MAX_SPEED_VAL+SLIP_SPEED ;

			// transform speed value to massage value
			outData[0] = ((uint16_t) setpoint) + SPEED_OFFSET ;
			send_on_CAN(SPEED_SETPOINT, outData, ECU) ; break;
		case STEERING:
			max_speed = MAX_SPEED-data[0]/SPEED_RED ; break;
		case ACCELERATOR:
			if(!brake_override)
				outData[0] = data[0] ;
			else
				outData[0] = NO_TORQUE ;
			outData[0] = (outData[0]>>1)+TORQUE_OFFSET ;
			send_on_CAN(TORQUE_SETPOINT,outData,ECU);break;
		case BREAK_PEDAL:
			// put no torque, when brakeing
			if (data[0]>BRAKE_THRESHOLD)
				brake_override = true;
			else
				brake_override = false;
			break;
		default: break ;
		}
}
#endif
