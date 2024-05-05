/*
 * programm_function.c
 *
 *  Created on: May 3, 2024
 *      Author: emils
 */


#include <programm_function.h>


bool checkForLastBit(uint8_t temp, uint8_t temp_last) {
	if (temp != 0xD9) {
		return true;
	}
	if (temp_last != 0xFF) {
		return true;
	}
	return false;
}

bool checkForFirstBit(uint8_t temp, uint8_t temp_last) {
	if (temp != 0xD8) {
		return true;
	}
	if (temp_last != 0xFF) {
		return true;
	}
	return false;
}

void readSPIbuff(const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	uint8_t spi_recv_buf = 0;
	uint8_t spi_buf;
	uint8_t temp = 0;
	HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_RESET);

	spi_buf = BURST_FIFO_READ;
	temp = HAL_SPI_TransmitReceive(&hspi3, &spi_buf, &spi_recv_buf, 1,
			100);

	temp = HAL_SPI_TransmitReceive(&hspi3, pTxData, pRxData,
			Size, 200);
	HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_SET);
}
