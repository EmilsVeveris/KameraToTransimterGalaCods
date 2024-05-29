/*
 * programm_function.c
 *
 *  Created on: May 3, 2024
 *      Author: emils
 */


#include <programm_function.h>

/**
 * @brief Checks if recived data from camera module is the last bytes of a jpeg picture
 *
 * @param byte Byte to check
 * @param lastRecivedByte one byte before
 *
 * @retval returns boolean, true if last bytes found, else false
 */
bool checkForLastByte(uint8_t byte, uint8_t byteBefore)
{
	if (byte != 0xD9)
	{
		return false;
	}
	if (byteBefore != 0xFF)
	{
		return false;
	}
	return true;
}

/**
 * @brief Checks if recived data from camera module is the first bytes of a jpeg picture
 *
 * @param byte Byte to check
 * @param lastRecivedByte one byte before
 *
 * @retval returns boolean, true if first bytes found, else false
 */
bool checkForFirstByte(uint8_t byte, uint8_t byteBefore)
{
	if (byte != 0xD8)
	{
		return false;
	}
	if (byteBefore != 0xFF)
	{
		return false;
	}
	return true;
}

/**
 * @brief Read camera modules SPI buffer
 *
 * @param pTxData Pointer to buffer which values need to be sent
 * @param pRxData Pointer to buffer where to save recived values
 * @param Size buffer size, both buffer have to be the same size
 */
void readSPIbuff(const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	uint8_t spi_recv_buf = 0;
	uint8_t spi_buf;
	uint8_t temp = 0;
	HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_RESET);

	spi_buf = BURST_FIFO_READ;
	temp = HAL_SPI_TransmitReceive(&hspi3, &spi_buf, &spi_recv_buf, 1, 100);
	temp = HAL_SPI_TransmitReceive(&hspi3, pTxData, pRxData, Size, 200);

	HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_SET);
}
