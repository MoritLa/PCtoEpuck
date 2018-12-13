/*
 * CAN_protocol.h
 *
 *  Created on: 8 déc. 2018
 *      Author: Moritz Laim
 */


#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H
#include <stdint.h>

typedef void (*MessageFromProtocol)(uint8_t messageNb, uint16_t data[4], uint8_t node);

void CAN_protocol_init(MessageFromProtocol SourceFP) ;

void send_on_CAN(uint8_t messageNb, uint16_t data[4], uint8_t node) ;

#endif /* CAN_PROTOCOL_H */
