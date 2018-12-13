/*
 * UART_protocol.h
 *
 *  Created on: 9 déc. 2018
 *      Author: Moritz Laim
 */

#ifndef SOURCE_UART_PROTOCOL_H_
#define SOURCE_UART_PROTOCOL_H_

#include <stdint.h>
#include "connector.h"

typedef void (*MessageFromProtocol)(uint8_t messageNb, uint16_t data[4], uint8_t node);

void UART_protocol_init(MessageFromProtocol SourceFP) ;
void send_on_UART(uint8_t messageNb, uint16_t data[4], uint8_t node) ;

#endif /* SOURCE_UART_PROTOCOL_H_ */
