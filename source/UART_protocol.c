/*
 * UART_protocol.c
 *
 *  Created on: 9 déc. 2018
 *      Author: Moritz Laim
 */

#include "UART_protocol.h"

#include "connector.h"
#include "message_table.h"

static MessageFromProtocol toSourceUARTFP ;

void treat_data_UART(MyMessage input) ;

void UART_protocol_init(MessageFromProtocol SourceFP)
{
	toSourceUARTFP = SourceFP ;
	init_to_source(treat_data_UART) ;
}

void send_on_UART(uint8_t messageNb, uint16_t data[4], uint8_t node)
{
	MyMessage output ;
	switch(messageNb)
	{
	case SPEED_SETPOINT_U:
		output.id = SPEED_SETPOINT_ID_U ;
		output.length = SPEED_SETPOINT_LENGTH_U ;
		for(int i = 0; i < (uint8_t) SPEED_SETPOINT_LENGTH_U/2;i++)
			output.data16[i] = data[i] ;
		if(node == MOTOR)
			send_to_PC(output) ;
		break ;
	default: break;
	}
}

//local functions
void treat_data_UART(MyMessage input)
{
	switch(input.id)
	{
	case TRUE_MOTOR_SPEED_ID_U:
		toSourceUARTFP(TRUE_MOTOR_SPEED_U, input.data16, MOTOR) ; break;
	default: break;
	}
}
