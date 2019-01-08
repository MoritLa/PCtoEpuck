/*
 * LRT_protocol.c
 *
 *  Created on: 8 déc. 2018
 *      Author: Moritz Laim
 */

#include "CAN_protocol.h"

#include "connector.h"
#include "message_table.h"

#include "leds.h"

#define N_LIM_REG			0x34
#define TORQUE_SETPOINT_REG	0x90
#define SPEED_IST			0x30
#define I_IST_REG			0x20

#define K_PHI				(3/4)
#define CONVERT_TORQUE		1			//conversion Iact = 2/10*REG_C6*(REG_20/REG_D9)

#define CONVERT_SPEED		1

#define ON					1
#define OFF					0

#define NODE				0
#define MESSAGE				1

#define NOW					0

typedef struct ProtocolTiming_struct
{
	uint8_t protocol_time ;
	uint8_t last_time ;
} ProtocolTiming;

// table that keeps track of the timing of the the messages
static ProtocolTiming lastMessageTime[3][16] =
{
		[ECU][SPEED_SETPOINT].protocol_time = SETPOINT_PERIOD,			[ECU][SPEED_SETPOINT].last_time = NOW ,
		[ECU][TORQUE_SETPOINT].protocol_time = SETPOINT_PERIOD,			[ECU][TORQUE_SETPOINT].last_time = NOW ,
		[ECU][ECU_STATUS].protocol_time = ECU_STATUS_PERIOD,			[ECU][ECU_STATUS].last_time = NOW ,
		[ECU][ECU_ERROR].protocol_time = ECU_ERROR_PERIOD,				[ECU][ECU_ERROR].last_time = NOW ,
		[ECU][CONTROL].protocol_time = CONTROL_PERIOD ,					[ECU][CONTROL].last_time = NOW ,
		[ECU][ECU_REQUEST].protocol_time = ECU_REQUEST_PERIOD,			[ECU][ECU_REQUEST].last_time = NOW ,
		[ECU][BMS_CONTACTOR].protocol_time = BMS_CONTACTOR_PERIOD,		[ECU][BMS_CONTACTOR].last_time = NOW ,

		[MOTOR][SPEED_VALUE].protocol_time = SPEED_VALUE_PERIOD,		[MOTOR][SPEED_VALUE].last_time = NOW ,
		[MOTOR][TORQUE_VALUE].protocol_time = TORQUE_VALUE_PERIOD,		[MOTOR][TORQUE_VALUE].last_time = NOW ,
		[MOTOR][MOTOR_STATUS].protocol_time = MOTOR_STATUS_PERIOD,		[MOTOR][MOTOR_STATUS].last_time = NOW ,

		[SENSOR][BREAK_PEDAL].protocol_time = BREAK_PEDAL_PERIOD,		[SENSOR][BREAK_PEDAL].last_time = NOW ,
		[SENSOR][ACCELERATOR].protocol_time = ACCELERATOR_PERIOD,		[SENSOR][ACCELERATOR].last_time = NOW ,
		[SENSOR][STEERING].protocol_time = STEERING_PERIOD,				[SENSOR][STEERING].last_time = NOW ,
		[SENSOR][WHEEL_SPEED].protocol_time = WHEEL_SPEED_PERIOD,		[SENSOR][WHEEL_SPEED].last_time = NOW ,
		[SENSOR][ACCELERATION].protocol_time = ACCELERATION_PERIOD,		[SENSOR][ACCELERATION].last_time = NOW ,
		[SENSOR][ENVIRONMENT].protocol_time = ENVIRONMENT_PERIOD,		[SENSOR][ENVIRONMENT].last_time = NOW ,
		[SENSOR][COCKPIT].protocol_time = COCKPIT_PERIOD,				[SENSOR][COCKPIT].last_time = NOW ,
		[SENSOR][BMS_EVENTS].protocol_time = BMS_EVENTS_PERIOD,			[SENSOR][BMS_EVENTS].last_time = NOW ,
		[SENSOR][SENEOR_STATUS].protocol_time = SENEOR_STATUS_PERIOD,	[SENSOR][SENEOR_STATUS].last_time = NOW ,
		[SENSOR][SENSOR_ERROR].protocol_time = SENSOR_ERROR_PERIOD,		[SENSOR][SENSOR_ERROR].last_time = NOW ,
		[SENSOR][SENOR_REQUEST].protocol_time = SENOR_REQUEST_PERIOD,	[SENSOR][SENOR_REQUEST].last_time = NOW
};

// list of the messages, that should be received by the nodes
#define ECU_LIST_LENGTH		4

static const uint8_t ECU_list[2][ECU_LIST_LENGTH] =
{
		{SENSOR,		SENSOR,			SENSOR,		SENSOR},
		{BREAK_PEDAL, 	ACCELERATOR, 	STEERING, 	WHEEL_SPEED}
};

#define MOTOR_LIST_LENGTH		4

static const uint8_t Motor_list[2][MOTOR_LIST_LENGTH] =
{
		{ECU,				ECU,			 ECU,			SENSOR},
		{SPEED_SETPOINT, 	TORQUE_SETPOINT, ECU_STATUS,	STEERING}
};

#define SENSOR_LIST_LENGTH		1

static const uint8_t Sensor_list[2][SENSOR_LIST_LENGTH] =
{
		{ECU	},
		{ECU_STATUS}
};


static MessageFromProtocol toSourceCANFP ;
static bool running;

void treat_data_CAN(MyMessage input) ;
void update_timing(uint8_t node) ;
void convert_16to8_table(uint16_t in[4],uint8_t out[8],
						 uint8_t lengthByte, uint16_t convertionFactor) ;


//global functions
void CAN_protocol_init(MessageFromProtocol SourceFP)
{
	toSourceCANFP = SourceFP ;
	init_to_protocol(treat_data_CAN, update_timing) ;
}

void send_on_CAN(uint8_t messageNb, uint16_t data[4], uint8_t node)
{
	uint8_t outData[8] ;
	// change format form internal identifier/data size to CAN format
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
		// add register of the Bamocar CAN protocol
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
	running = run ;
}

//local functions
void treat_data_CAN(MyMessage input)
{
	uint16_t inData[4];
	switch (input.id)
	{
	case SETPOINT_ID :
		// check for the register of the Bamocar CAN-protocol
		switch (input.data8[0])
		{
		case N_LIM_REG:
			for(int i = 0; i < (uint8_t)(SETPOINT_LENGTH-1)/2;i++)
				inData[i]=(input.data8[2*i+1]+(input.data8[2*i+2]<<8))*CONVERT_SPEED ;

			toSourceCANFP(SPEED_SETPOINT,inData,MOTOR);
			lastMessageTime[ECU][SPEED_SETPOINT].last_time = NOW ; break ;
		case TORQUE_SETPOINT_REG:
			for(int i = 0; i < (uint8_t)(SETPOINT_LENGTH-1)/2;i++)
				inData[i]=(input.data8[2*i+1]+(input.data8[2*i+2]<<8))*CONVERT_SPEED ;

			toSourceCANFP(TORQUE_SETPOINT,inData,MOTOR);
			lastMessageTime[ECU][TORQUE_SETPOINT].last_time = NOW ;break ;
		}	break ;
	case ECU_STATUS_ID:
		lastMessageTime[ECU][ECU_STATUS].last_time = NOW ; break;
	case WHEEL_SPEED_ID:
		toSourceCANFP(WHEEL_SPEED,input.data16,ECU);
		lastMessageTime[SENSOR][WHEEL_SPEED].last_time = NOW ; break;
	case STEERING_ID:
		toSourceCANFP(STEERING,input.data16,ECU);
		toSourceCANFP(STEERING,input.data16,MOTOR);
		lastMessageTime[SENSOR][STEERING].last_time = NOW ; break;
	case ACCELERATOR_ID:
		toSourceCANFP(ACCELERATOR, input.data16, ECU) ;
		lastMessageTime[SENSOR][ACCELERATOR].last_time = NOW ; break;
	case BREAK_PEDAL_ID:
		toSourceCANFP(BREAK_PEDAL,input.data16,ECU) ;
		lastMessageTime[SENSOR][BREAK_PEDAL].last_time = NOW ; break;
	default : break ;
	}

}

// increace the counter for the messages the node should receive. When bigger than
// two times the expected period, the body LED is acivated.
void update_timing(uint8_t node)
{
	if(running)
		switch (node)
		{
		case ECU :
			for(int i = 0; i< ECU_LIST_LENGTH;i++)
			{
				lastMessageTime[ECU_list[NODE][i]][ECU_list[MESSAGE][i]].last_time++ ;
				if(lastMessageTime[ECU_list[NODE][i]][ECU_list[MESSAGE][i]].last_time>
					2*lastMessageTime[ECU_list[NODE][i]][ECU_list[MESSAGE][i]].protocol_time)
					set_body_led(ON) ;
				else
					set_body_led(OFF) ;
			}break;
		case MOTOR :
			for(int i = 0; i< MOTOR_LIST_LENGTH;i++)
			{
				lastMessageTime[Motor_list[NODE][i]][Motor_list[MESSAGE][i]].last_time++ ;
				if(lastMessageTime[Motor_list[NODE][i]][Motor_list[MESSAGE][i]].last_time>
					2*lastMessageTime[Motor_list[NODE][i]][Motor_list[MESSAGE][i]].protocol_time)
					set_body_led(ON) ;
				else
					set_body_led(OFF) ;
			}break;
		case SENSOR :
			for(int i = 0; i< SENSOR_LIST_LENGTH;i++)
			{
				lastMessageTime[Sensor_list[NODE][i]][Sensor_list[MESSAGE][i]].last_time++ ;
				if(lastMessageTime[Sensor_list[NODE][i]][Sensor_list[MESSAGE][i]].last_time>
					2*lastMessageTime[Sensor_list[NODE][i]][Sensor_list[MESSAGE][i]].protocol_time)
					set_body_led(ON) ;
				else
					set_body_led(OFF) ;
			}break;
		default: break ;
		}
}

// transfomr 16 bit to 8 bit table and divided by convertionFactor
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
