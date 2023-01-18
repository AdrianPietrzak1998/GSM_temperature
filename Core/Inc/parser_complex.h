/*
 * parser_simple.h
 *
 *  Created on: 4 cze 2022
 *      Author: Adrian
 */

#ifndef INC_PARSER_COMPLEX_H_
#define INC_PARSER_COMPLEX_H_

#define ENDLINE '\n'

extern double SignalQuality;

extern uint8_t year, month, day, hour, minute, second;



extern SMSUartTxState_t SMSUartTxState;
extern GSM_t GSM;

extern char SMSMessage[SMS_SIZE];

extern int32_t temperature;

void Parser_TakeLine(RingBuffer_t *Buff, uint8_t *Destination);
void Parser_parse(uint8_t * DataToParse);

#endif /* INC_PARSER_COMPLEX_H_ */
