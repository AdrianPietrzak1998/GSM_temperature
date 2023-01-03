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
void Temperature100ToString(int32_t temp, char *StringBuf);

extern GSM_t GSM;

//extern uint8_t ReceivedState;
extern uint8_t *Uart1isBusyPtr;


#endif /* INC_UTILS_H_ */
