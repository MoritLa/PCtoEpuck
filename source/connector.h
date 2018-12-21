/*
 * connector.h
 *
 *  Created on: 1 nov. 2018
 *      Author: Moritz Laim
 */

#ifndef CONNECTOR_H
#define CONNECTOR_H

#include <stdint.h>
#include <stdbool.h>

typedef struct MyMessage_struct
{
	uint16_t id ;
	uint8_t length ;
	union{
		uint8_t data8[8] ;
		uint16_t data16[4] ;
		uint32_t data32[2];
	} ;
} MyMessage;

typedef void (*MessageToSource)(MyMessage input);
typedef void (*UpdateTiming)(uint8_t node);

// initialise threads and variables
void connector_init(void) ;

// set the output to active (send = 1) or inactive (send = 0)
void can_send(bool send) ;

// sends the message out to the PC via UART
void send_to_PC(const MyMessage out) ;

// writes the data in the CAN message table (at position node, message_nb) if the length is right
void write_to_table(uint8_t messageNb, uint8_t data[8], uint8_t length, uint8_t node) ;

//
void init_to_protocol(MessageToSource ProtocolFP, UpdateTiming TimingFP) ;
void init_to_source(MessageToSource SourceFP) ;

#endif /* CONNECTOR_H */
