/*
 * ring_buffer.h
 *
 *  Created on: Jun 3, 2022
 *      Author: Adrian
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#define RING_BUFFER_SIZE 512
// Success status
typedef enum
{
	RB_OK     = 0,
	RB_ERROR  = 1
}RbStatus_t;
// Ring buffer object

typedef struct
{
	uint16_t Head, Tail;
	uint8_t Buffer[RING_BUFFER_SIZE];
} RingBuffer_t;

// Function
//Write
RbStatus_t Ring_Buffer_Write(RingBuffer_t * Buff, uint8_t value);
//Read
RbStatus_t Ring_Buffer_Read(RingBuffer_t * Buff, uint8_t *value);
//Flush
void Ring_Buffer_Flush(RingBuffer_t * Buff);
#endif /* INC_RING_BUFFER_H_ */
