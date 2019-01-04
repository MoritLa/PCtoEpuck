/*
 * sensor.c
 *
 *  Created on: 16 déc. 2018
 *      Author: Moritz Laim
 */
#if 1
#include "stdint.h"

#include "sensor.h"
#include "connector.h"

#include "CAN_protocol.h"
#include "UART_protocol.h"
#include "message_table.h"

#include "sensors\imu.h"

#define RUN_SIM		(1<<0)

#define MAX_VAL				0x7FFF
#define STEERING_OFFSET		0x7FFF

enum{PC_FL,PC_FR,PC_RR,PC_RL} ;
enum{RL, RR, FL, FR} ;

bool running ;

void treat_UART_data(uint8_t messageNb, uint16_t data[4], uint8_t node) ;
void treat_CAN_data(uint8_t messageNb, uint16_t data[4], uint8_t node) ;


static THD_WORKING_AREA(waAcceleration, 256) ;
static THD_FUNCTION(Acceleration, arg) {

    chRegSetThreadName(__FUNCTION__) ;
    (void)arg;
    int16_t acc_val[3] ;
    uint16_t steering_angle ;
    uint16_t outData[4] ;

    while(1){
    	if(running)
    	{
    		acc_val[0] = get_acc_filtered(0,10) ;
    		acc_val[1] = get_acc_filtered(1,10) ;
    		acc_val[2] = get_acc_filtered(2,10) ;

    		steering_angle = (acc_val[1]>>1)+MAX_VAL ;

    		outData[0] = steering_angle ;
    		send_on_UART( STEERING_ANGLE_U,  outData,  SENSOR) ;
    	}
    		chThdSleepMilliseconds(150);

    }
}


void source_init(void)
{
	chThdCreateStatic(waAcceleration,
					  sizeof(waAcceleration),
					  NORMALPRIO,
					  Acceleration,
					  NULL);

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
			run_simulation(running) ;

			break;
		case TRUE_SPEED_U:
			// change order of the data
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
	// no data treated
	if(running && node == SENSOR)
		switch(messageNb)
		{
		default: break ;
		}
}
#endif
