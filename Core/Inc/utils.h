/*
 * utils.h
 *
 *  Created on: 4 cze 2022
 *      Author: Adrian
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

void UartLog(char * Message);
void UartSend(char * Message);
void UartSendWoRxCtrl(char * Message);

extern uint8_t ReceivedState;
extern uint8_t Uart1isBusy;


#endif /* INC_UTILS_H_ */
