/*
 * connector.h
 *
 *  Created on: 1 nov. 2018
 *      Author: Moritz Laim
 */

#ifndef CONNECTOR_H
#define CONNECTOR_H

typedef struct MyMessage_struct
{
	uint16_t id ;
	uint8_t length ;
	union{
		uint8_t data_8[8] ;
		uint16_t data_16[4] ;
		uint32_t data_32[2];
	} ;
} MyMessage;


void connector_init(void) ;

void can_send(bool send) ;
void send_to_PC(const MyMessage out) ;
void write_to_table(uint8_t messageNb, uint8_t data[8],uint8_t node) ;

#endif /* CONNECTOR_H */
