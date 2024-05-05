/*
 * programm_function.h
 *
 *  Created on: May 3, 2024
 *      Author: emils
 */

#ifndef INC_PROGRAMM_FUNCTION_H_
#define INC_PROGRAMM_FUNCTION_H_

#include "main.h"
#include <stdbool.h>
#include "spi.h"

#define BURST_FIFO_READ			0x3C

//Picture start and end
volatile bool checkForLastBit(uint8_t temp, uint8_t temp_last);
volatile bool checkForFirstBit(uint8_t temp, uint8_t temp_last);
void readSPIbuff(const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);

#endif /* INC_PROGRAMM_FUNCTION_H_ */
