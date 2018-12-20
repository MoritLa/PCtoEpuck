/*
 * LRT_protocol.c
 *
 *  Created on: 8 d�c. 2018
 *      Author: Moritz Laim
 */

#include "CAN_protocol.h"

#include "connector.h"
#include "message_table.h"

#define N_LIM_REG			0x34
#define TORQUE_SETPOINT_REG	0x90
#define SPEED_IST			0x30
#define I_IST_REG			0x20

#define K_PHI				(3/4)
#define CONVERT_TORQUE		1			//conversion Iact = 2/10*REG_C6*(REG_20/REG_D9)

#define CONVERT_SPEED		1

static MessageFromProtocol toSourceCANFP ;

void treat_data_CAN(MyMessage input) ;
void convert_16to8_table(uint16_t in[4], uint8_t out[8], uint8_t length, uint8_t Factor) ;

//global functions
void CAN_protocol_init(MessageFromProtocol SourceFP)
{
	toSourceCANFP = SourceFP ;
	init_to_protocol(treat_data_CAN) ;
}

void send_on_CAN(uint8_t messageNb, uint16_t data[4], uint8_t node)
{
	uint8_t outData[8] ;
	switch(node)
	{
	case ECU :
		switch(messageNb)
		{
		case SPEED_SETPOINT:
			outData[0] = N_LIM_REG ;
			outData[1] = data[i] ;
			outData[2] = data[i]>>8 ;

			write_to_table(messageNb, outData, SPEED_VALUE_LENGTH,node); break;
		case TORQUE_SETPOINT:
			outData[0] = TORQUE_SETPOINT_REG ;
			outData[1] = data[i] ;
			outData[2] = data[i]>>8 ;

			write_to_table(messageNb, outData, TORQUE_VALUE_LENGTH,node); break;
		default: break;
		} break ;
	case MOTOR :
		switch(messageNb)
		{
		case SPEED_VALUE:
			convert_16to8_table(data,outData,SPEED_VALUE_LENGTH, CONVERT_TORQUE) ;

			write_to_table(messageNb, outData, SPEED_VALUE_LENGTH,node); break;
		case TORQUE_VALUE: break ;
			convert_16to8_table(data,outData,TORQUE_VALUE_LENGTH, CONVERT_TORQUE) ;

			write_to_table(messageNb, outData, TORQUE_VALUE_LENGTH,node); break;
		default: break ;
		}break ;
	case SENSOR :
		switch(messageNb)
		{
		case WHEEL_SPEED:
			convert_16to8_table(data,outData,WHEEL_SPEED_LENGTH, CONVERT_TORQUE) ;

			write_to_table(messageNb, outData, WHEEL_SPEED_LENGTH,node); break;
		case STEERING:
			convert_16to8_table(data,outData,STEERING_LENGTH, CONVERT_TORQUE) ;

			write_to_table(messageNb, outData, STEERING_LENGTH,node); break;
		case BREAK_PEDAL:
			convert_16to8_table(data,outData,BREAK_PEDAL_LENGTH, CONVERT_TORQUE) ;

			write_to_table(messageNb, outData, BREAK_PEDAL_LENGTH, node); break;
		case ACCELERATOR:
			convert_16to8_table(data,outData,ACCELERATOR_LENGTH, CONVERT_TORQUE) ;

			write_to_table(messageNb, outData, ACCELERATOR_LENGTH,node); break;
		default : break ;
		}
		break ;
	default: break ;
	}
}

//local functions
void treat_data_CAN(MyMessage input)
{
	uint16_t inData[4];
	switch (input.id)
	{
	case SETPOINT_ID :
		switch (input.data8[0])
		{
		case N_LIM_REG:
			for(int i = 0; i < (uint8_t)(SETPOINT_LENGTH-1)/2;i++)
				inData[i]=(input.data8[2*i+1]+(input.data8[2*i+2]<<8))*CONVERT_SPEED ;

			toSourceCANFP(SPEED_SETPOINT,inData,MOTOR);
			break ;
		case TORQUE_SETPOINT_REG:
			break ;
		}	break ;
	case SENSOR_ERROR_ID: break ;

	default : break ;
	}

}

void convert_16to8_table(uint16_t in[4], uint8_t out[8], uint8_t length, uint8_t Factor)
{
	for(int i = 0; i<length/2;i++)
	{
		out[2*i] = in[i]/Factor ;					//verify
		out[2*i+1] = (in[i]/Factor)>>8 ;
	}
}
