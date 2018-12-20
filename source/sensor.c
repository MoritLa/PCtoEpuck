/*
 * sensor.c
 *
 *  Created on: 16 déc. 2018
 *      Author: Moritz Laim
 */
#if 0
#include "stdint.h"

#include "sensor.h"
#include "connector.h"

#include "CAN_protocol.h"
#include "UART_protocol.h"
#include "message_table.h"

#define RUN_SIM		(1<<0)

enum{PC_FL,PC_FR,PC_RR,PC_RL} ;
enum{RL, RR, FL, FR} ;

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
	uint16_t outdata[4] ;

	if(node == SENSOR)
		switch(messageNb)
		{
		case SENSOR_CONFIG_U:
			running = data[0]&RUN_SIM ;
			run_simulation(running) ; break;
		case TRUE_SPEED_U:
			outdata[FL] = data[PC_FL] ;
			outdata[FR] = data[PC_FR] ;
			outdata[RL] = data[PC_RL] ;
			outdata[RR] = data[PC_RR] ;
			send_on_CAN(WHEEL_SPEED, outdata, SENSOR) ;break;
		case STEERING_U:
			outdata[0] = data[0] ;
			send_on_CAN(STEERING, outdata,SENSOR) ; break;
		case PEDALS_U :
			outdata[0] = data[0] ;
			send_on_CAN(BREAK_PEDAL,outdata,SENSOR) ;
			outdata[0] = data[1] ;
			send_on_CAN(ACCELERATOR, outdata,SENSOR) ;
			break;
		default: break;
		}
}

void treat_CAN_data(uint8_t messageNb, uint16_t data[4], uint8_t node)
{
	if(running && node == SENSOR)
		switch(messageNb)
		{
		default: break ;
		}
}
#endif
