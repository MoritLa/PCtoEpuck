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
	switch(node)
	{
	case MOTOR:
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
		case TORQUE_SETPOINT_U:
			output.id = TORQUE_SETPOINT_ID_U ;
			output.length = TORQUE_SETPOINT_LENGTH_U ;
			for(int i = 0; i < (uint8_t) TORQUE_SETPOINT_LENGTH_U/2;i++)
				output.data16[i] = data[i] ;
			if(node == MOTOR)
				send_to_PC(output) ;
			break ;
		default: break;
		} break ;
	default: break;
	}
}

//local functions
void treat_data_UART(MyMessage input)
{
	switch(input.id)
	{
	case ECU_CONFIG_ID_U:
		if(input.length == ECU_CONFIG_SPEED_LENGTH_U)
			toSourceUARTFP(ECU_CONFIG_SPEED_U, input.data16, ECU) ; break;
	case MOTOR_CONFIG_ID_U:
		if(input.length == MOTOR_CONFIG_SPEED_LENGTH_U)
			toSourceUARTFP(MOTOR_CONFIG_SPEED_U, input.data16, MOTOR) ; break;
	case TRUE_MOTOR_SPEED_ID_U:
		if(input.length == TRUE_MOTOR_SPEED_LENGTH_U)
			toSourceUARTFP(TRUE_MOTOR_SPEED_U, input.data16, MOTOR) ; break;
	case SENSOR_CONFIG_ID_U:
		if(input.length == SENSOR_CONFIG_LENGTH_U)
			toSourceUARTFP(SENSOR_CONFIG_U, input.data16, SENSOR) ; break;
	case TRUE_SPEED_ID_U:
		if(input.length == TRUE_SPEED_SPEED_LENGTH_U)
			toSourceUARTFP(TRUE_SPEED_SPEED_U, input.data16, SENSOR) ; break;

	case STEERING_ID_U:
		if(input.length == STEERING_LENGTH_U)
			toSourceUARTFP(STEERING_U, input.data16, SENSOR) ; break;

	case STEERING_ID_U:
		if(input.length == PEDALS_LENGTH_U)
			toSourceUARTFP(PEDALS_U, input.data16, SENSOR) ; break;

	default: break;
	}
}
