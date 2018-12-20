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
void transfer_table_16(uint16_t in[4], uint16_t out[4], uint8_t lengthBytes) ;

void UART_protocol_init(MessageFromProtocol SourceFP)
{
	toSourceUARTFP = SourceFP ;
	init_to_source(treat_data_UART) ;
}

void send_on_UART(uint8_t messageNb, uint16_t data[4], uint8_t node)
{
	MyMessage output ;
	switch (node)
	{
	case MOTOR:
		switch(messageNb)
		{
		case SPEED_SETPOINT_U:
			output.id = SPEED_SETPOINT_ID_U ;
			output.length = SPEED_SETPOINT_LENGTH_U ;
			transfer_table_16(data, output.data16, SPEED_SETPOINT_LENGTH_U) ;

			send_to_PC(output) ; break;
		case TORQUE_SETPOINT_U:
			output.id = TORQUE_SETPOINT_ID_U ;
			output.length = TORQUE_SETPOINT_LENGTH_U ;
			transfer_table_16(data, output.data16, TORQUE_SETPOINT_LENGTH_U) ;

			send_to_PC(output) ;
			break ;
		default : break ;
		}break ;

	case SENSOR:
		switch(messageNb)
		{
		case STEERING_ANGLE_U:
			output.id = STEERING_ANGLE_ID_U ;
			output.length = STEERING_ANGLE_LENGTH_U ;
			transfer_table_16(data, output.data16, STEERING_ANGLE_LENGTH_U) ;
			send_to_PC(output) ;
			break;
		default: break;
		}break ;

	default : break;
	}
}

//local functions
void treat_data_UART(MyMessage input)
{
	switch(input.id)
	{
	case MOTOR_CONFIG_ID_U:
		if(input.length == MOTOR_CONFIG_LENGTH_U)
			toSourceUARTFP(MOTOR_CONFIG_U, input.data16, MOTOR) ;
		break ;
	case TRUE_MOTOR_SPEED_ID_U:
		if(input.length == TRUE_MOTOR_SPEED_LENGTH_U)
			toSourceUARTFP(TRUE_MOTOR_SPEED_U, input.data16, MOTOR) ;
		break;
	case SENSOR_CONFIG_ID_U:
		if(input.length == SENSOR_CONFIG_LENGTH_U)
			toSourceUARTFP(SENSOR_CONFIG_U, input.data16, SENSOR) ;
		break ;
	case TRUE_SPEED_ID_U:
		if(input.length == TRUE_SPEED_LENGTH_U)
			toSourceUARTFP(TRUE_SPEED_U, input.data16,SENSOR) ;
		break;
	case PEDALS_ID_U:
		if(input.length == PEDALS_LENGTH_U)
			toSourceUARTFP(PEDALS_U, input.data16,SENSOR) ;
		break;
	case STEERING_ID_U:
		if(input.length == STEERING_LENGTH_U)
			toSourceUARTFP(STEERING_U, input.data16,SENSOR) ;
		break;
	case ECU_CONFIG_ID_U:
		if(input.length == ECU_CONFIG_LENGHT_U)
			toSourceUARTFP(ECU_CONFIG_U, input.data16, ECU) ;
		break ;
	default: break;
	}
}

void transfer_table_16(uint16_t in[4], uint16_t out[4], uint8_t lengthBytes)
{
	for(int i = 0; i <lengthBytes/2;i++)
		out[i] = in[i] ;
}
