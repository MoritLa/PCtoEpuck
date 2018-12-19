/*
 * LRT_protocol.c
 *
 *  Created on: 8 déc. 2018
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
void convert_16to8_table(uint16_t in[4],uint8_t out[8],
						 uint8_t lengthByte, uint16_t convertionFactor) ;

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
	case MOTOR:
		switch(messageNb)
		{
		case SPEED_VALUE:
			convert_16to8_table(data, outData, SPEED_VALUE_LENGTH, node) ;

			write_to_table(messageNb, outData, SPEED_VALUE_LENGTH,node);break;
		default: break;
		}break;

	case SENSOR:
		switch(messageNb)
		{
		case WHEEL_SPEED:
			convert_16to8_table(data,outData,WHEEL_SPEED_LENGTH,1) ;

			write_to_table(messageNb, outData, WHEEL_SPEED_LENGTH,node); break;
		case STEERING:
			convert_16to8_table(data,outData,STEERING_LENGTH,1) ;

			write_to_table(messageNb, outData, STEERING_LENGTH, node); break;
		case ACCELERATOR:
			convert_16to8_table(data, outData, ACCELERATOR_LENGTH,1) ;

			write_to_table(messageNb, outData, ACCELERATOR_LENGTH, node); break;
		case BREAK_PEDAL:
			convert_16to8_table(data, outData, BREAK_PEDAL_LENGTH,1) ;

			write_to_table(messageNb, outData, BREAK_PEDAL_LENGTH, node);break;
		default: break;
		}break;
	case ECU:
		switch(messageNb)
		{
		case SPEED_SETPOINT:
			outData[0] = N_LIM_REG ;
			data[0] = data[0]/CONVERT_SPEED ;
			outData[1] = (data[0]);
			outData[2] = (data[0]>>8);

			write_to_table(messageNb, outData,SETPOINT_LENGTH,node) ;
			break;
		case TORQUE_SETPOINT:
			outData[0] = TORQUE_SETPOINT_REG ;
			data[0]=data[0]/CONVERT_TORQUE ;
			outData[1] = (data[0]);
			outData[2] = (data[0]>>8);

			write_to_table(messageNb, outData,SETPOINT_LENGTH,node) ;
			break;
		default: break;
		}break;
	default: break;

	}
}

void run_simulation(bool run)
{
	can_send(run) ;
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

			toSourceCANFP(SPEED_SETPOINT,inData,MOTOR);	break ;
		case TORQUE_SETPOINT_REG:
			for(int i = 0; i < (uint8_t)(SETPOINT_LENGTH-1)/2;i++)
				inData[i]=(input.data8[2*i+1]+(input.data8[2*i+2]<<8))*CONVERT_SPEED ;

			toSourceCANFP(TORQUE_SETPOINT,inData,MOTOR); break ;
		}	break ;
	case WHEEL_SPEED_ID:
		toSourceCANFP(WHEEL_SPEED,input.data16,ECU); break;
	case STEERING_ID:
		toSourceCANFP(STEERING,input.data16,ECU); break;
	case ACCELERATOR_ID:
		toSourceCANFP(ACCELERATOR, input.data16, ECU) ; break;
	case BREAK_PEDAL_ID:
		toSourceCANFP(BREAK_PEDAL,input.data16,ECU) ; break;
	default : break ;
	}

}

//divided by convertionFActor
void convert_16to8_table(uint16_t in[4],uint8_t out[8],
						 uint8_t lengthByte, uint16_t convertionFactor)
{
	for(uint8_t i = 0; i<lengthByte/2;i++)
	{
		in[i] = in[i]/convertionFactor ;
		out[2*i]= in[i] ;
		out[2*i+1] = in[i]>>8 ;
	}
}

void convert_8to16_table(uint8_t in[8], uint16_t out[4],
						 uint8_t lengthByte, uint16_t convertionFactor)
{
	for(uint8_t i = 0; i < lengthByte/2;i++)
		out[i]=(in[2*i+1]+(in[2*i+2]<<8))*convertionFactor ;

}
