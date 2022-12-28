/*
 * ring_buffer.c
 *
 *  Created on: Jun 3, 2022
 *      Author: Adrian
 */
#include "main.h"
#include "ring_buffer.h"

RbStatus_t Ring_Buffer_Read(RingBuffer_t * Buff, uint8_t *value)
{
	if(Buff->Head == Buff->Tail)
	{
		return RB_ERROR;
	}
	*value = Buff->Buffer[Buff->Tail];

	Buff->Tail = (Buff->Tail + 1) % RING_BUFFER_SIZE;

	return RB_OK;
}

RbStatus_t Ring_Buffer_Write(RingBuffer_t * Buff, uint8_t value)
{
	uint16_t Head_tmp = (Buff->Head + 1) % RING_BUFFER_SIZE;

	if(Head_tmp == Buff->Tail)
	{
		return RB_ERROR;
	}
	Buff->Buffer[Buff->Head] = value;
	Buff->Head = Head_tmp;

	return RB_OK;
}

void Ring_Buffer_Flush(RingBuffer_t * Buff)
{
	Buff->Head = 0;
	Buff->Tail = 0;
}

