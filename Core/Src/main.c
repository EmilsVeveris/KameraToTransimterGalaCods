/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "memorymap.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include "app_fatfs.h"
#include "fatfs_sd.h"

#include <string.h> /* memset */

#include "string.h"
#include "ArduCAM.h"
#include <stdbool.h>

#include "Spirit1_function.h"
#include "programm_function.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Spirit1 parameters


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//Camera function prototypes
int writeSensorRegs16_8(const struct sensor_reg reglist[]);
void initCam();
uint8_t writeSensorReg16_8(int regID, int regDat);
uint8_t readSensorReg16_8(uint16_t regID, uint8_t* regDat);
void write_reg(int address,int value);
uint8_t get_bit(uint8_t addr, uint8_t bit);

void turnCameraOff();
void turnCameraOn();

void takePicture();

void reciveAndSavePicture(FIL* fp);
void turnOnFilter();
void turnOffFilter();
void getTimeAndDate(RTC_TimeTypeDef *sTime, RTC_DateTypeDef *sDate);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Spirit1 flag status
volatile SpiritFlagStatus xTxDoneFlag;
volatile SpiritFlagStatus xRxDoneFlag;

FATFS fs;  // file system
FIL testFile; // Files
FILINFO fno;
FRESULT fresult;  // status result
UINT br, bw;  // File read/write count

FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//Variables
	uint8_t buffer[80];   /* File copy buffer */

	uint8_t spi_recv_buf = 0;
	uint8_t spi_buf;

	uint8_t vid, pid;

    RTC_DateTypeDef gDate;
    RTC_TimeTypeDef gTime;
    char time[55];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	turnCameraOff();
  //init fatfs
  if (MX_FATFS_Init() != APP_OK) {
      Error_Handler();
   }




  //set_time();




  	// MOUNT SD CARD
  	fresult = f_mount(&fs, "/", 1);
	if (fresult != FR_OK) {
		HAL_GPIO_WritePin(DIODE1_GPIO_Port, DIODE1_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(DIODE1_GPIO_Port, DIODE1_Pin, GPIO_PIN_RESET);
	}

  //Create directorys for picture storage
	fresult = f_mkdir("RedzamaGaisma");
	fresult = f_mkdir("IRGaisma");
	fresult = f_mkdir("SutitieAtteli");
	fresult = f_mkdir("Temp");

	Spirit1_init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
			/* USER CODE END WHILE */
			/* USER CODE BEGIN 3 */
			/*
			 * Init camera module after turn off
			 */
			turnCameraOn();
			HAL_Delay(50);
			HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_SET);
			HAL_Delay(20);
			//Check if SPI  communication with camera module is working
			while (spi_recv_buf != 0x55) {
				spi_buf = 0x00 | 0x80;
				HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi3, &spi_buf, &spi_recv_buf, 1, 100);
				spi_buf = 0x55;
				HAL_SPI_TransmitReceive(&hspi3, &spi_buf, &spi_recv_buf, 1, 100);
				HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_SET);
				spi_buf = 0x00;
				HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi3, &spi_buf, &spi_recv_buf, 1, 100);
				spi_buf = 0x55;
				HAL_SPI_TransmitReceive(&hspi3, &spi_buf, &spi_recv_buf, 1, 100);
				HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_SET);
				if (spi_recv_buf != 0x55) {
					HAL_GPIO_TogglePin(DIODE2_GPIO_Port, DIODE2_Pin); //Toogle diode to show if  Camera hasnt responded, or we dont recive correct data
					HAL_Delay(1000);
				} else {
					HAL_GPIO_TogglePin(DIODE1_GPIO_Port, DIODE1_Pin); //Tooge diode when Camera  responds correctly
					HAL_Delay(1000);
				}
			}
			//Check if the camera module type is OV5642
			writeSensorReg16_8(0xff, 0x01);
			readSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
			readSensorReg16_8(OV5642_CHIPID_LOW, &pid);
			//Check if camera module responds
			if ((vid != 0x56) || (pid != 0x42)) {
				HAL_GPIO_TogglePin(DIODE2_GPIO_Port, DIODE2_Pin); //Toogle diode to show if  Camera hasnt responded, or we dont recive correct data
				while (1);
			}
			// init cam
			initCam();
			// Write ARDUCHIP_TIM, VSYNC_LEVEL_MASK to spi
			write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
			//Change picture size
			// Close auto exposure mode
			//uint8_t _x3503;
			//wrSensorReg16_8(0x5001,_x3503|0x01);
			//Manually set the exposure value
			writeSensorReg16_8(0x3500, 0x00);
			writeSensorReg16_8(0x3501, 0x79);
			writeSensorReg16_8(0x3502, 0xe0);
			/*
			 * Start picture taking and saving into sd card
			 */
			writeSensorRegs16_8(ov5642_320x240);
			sprintf(time, "Temp/temp.JPEG");
			//Create an file
			fresult = f_open(&testFile, time, FA_CREATE_ALWAYS | FA_WRITE);

			// set all camera registers to take a picture

			takePicture();
			//Wait for capture to be done
			while (get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK) == 0);
			writeSensorRegs16_8(ov5642_2592x1944);
			//Recive picture save it into sd card
			reciveAndSavePicture(&testFile);
			f_close(&testFile);
			/* Get the RTC current Time */
			HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
			/* Get the RTC current Date */
			HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
			//Create file for next picture
			sprintf(time, "RedzamaGaisma/PIC-%02d-%02d-%02d-%02d-%02d-%2d.JPEG",
					gTime.Hours, gTime.Minutes, gTime.Seconds, gDate.Date,
					gDate.Month, (2000 + gDate.Year));
			//Create an file
			fresult = f_open(&testFile, time, FA_CREATE_ALWAYS | FA_WRITE);

			// set all camera registers to take a picture
			takePicture();
			//Wait for capture to be done
			while (get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK) == 0);
			//Chose picture size
			//Recive picture save it into sd card
			reciveAndSavePicture(&testFile);
		//	reciveAndSavePicture(&testFile);
			f_close(&testFile);
			turnOnFilter();
			HAL_Delay(100);
			/* Get the RTC current Time */
			HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
			/* Get the RTC current Date */
			HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
			//Create file for next picture
			sprintf(time, "IRGaisma/PIC-%02d-%02d-%02d-%02d-%02d-%2d.JPEG",
					gTime.Hours, gTime.Minutes, gTime.Seconds, gDate.Date,
					gDate.Month, (2000 + gDate.Year));
			//Create an file
			fresult = f_open(&testFile, time,
					FA_CREATE_ALWAYS | FA_WRITE | FA_READ);


			// set all kamera registers to take a picture
			takePicture();
		//	turnOffFilter();
			//Wait for capture to be done
			while (get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK) == 0);
			//Recive picture save it into sd card
			writeSensorRegs16_8(ov5642_320x240);
			reciveAndSavePicture(&testFile);
			f_close(&testFile);
			turnOffFilter();
			/* Get the RTC current Time */
			HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
			/* Get the RTC current Date */
			HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
			/* Display time Format: hh:mm:ss-dd-mm-yy */
			sprintf(time, "SutitieAtteli/PIC-%02d-%02d-%02d-%02d-%02d-%2d.JPEG",
					gTime.Hours, gTime.Minutes, gTime.Seconds, gDate.Date,
					gDate.Month, (2000 + gDate.Year));
			//Create an file
			fresult = f_open(&testFile, time, FA_CREATE_ALWAYS | FA_WRITE);

			// set all camera registers to take a picture
			takePicture();
			//Wait for capture to be done
			while (get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK) == 0);
			writeSensorRegs16_8(ov5642_2592x1944);
			//Recive picture save it into sd card, and send it to ground base station
			reciveAndSavePicture(&testFile);
			f_close(&testFile);
			turnCameraOff();
			fresult = f_open(&testFile, time, FA_READ);
			for (;;) {
				fresult = f_read(&testFile, buffer, sizeof(buffer), &br);
				if (br == 0)
					break; /* error or eof */
				xTxDoneFlag = S_RESET;
				SPSGRF_StartTx(buffer, sizeof(buffer));
				while (!xTxDoneFlag);
			}
			f_close(&testFile);
			HAL_Delay(10000);
		}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	SpiritIrqs xIrqStatus;

	if (GPIO_Pin != SPIRIT1_GPIO3_Pin)
	{
		return;
	}

	SpiritIrqGetStatus(&xIrqStatus);
	if (xIrqStatus.IRQ_TX_DATA_SENT)
	{
		xTxDoneFlag = S_SET;
	}
	if (xIrqStatus.IRQ_RX_DATA_READY)
	{
		xRxDoneFlag = S_SET;
	}
	if (xIrqStatus.IRQ_RX_DATA_DISC || xIrqStatus.IRQ_RX_TIMEOUT)
	{
		SpiritCmdStrobeRx();
	}
}

//Camera functions

/**
 * @brief Writes all the neccasery values to registers
 * to set camera in capture mode, and for it
 * to output jpeg images
 *
 *
 */
void initCam()
{
	writeSensorReg16_8(0x3008, 0x80);
	writeSensorRegs16_8(OV5642_QVGA_Preview);

	HAL_Delay(200);

	writeSensorRegs16_8(OV5642_JPEG_Capture_QSXGA);
	writeSensorRegs16_8(ov5642_320x240);

	HAL_Delay(100);

	writeSensorReg16_8(0x3818, 0xa8);
	writeSensorReg16_8(0x3621, 0x10);
	writeSensorReg16_8(0x3801, 0xb0);
	writeSensorReg16_8(0x4407, 0x08);
	writeSensorReg16_8(0x5888, 0x00);
	writeSensorReg16_8(0x5000, 0xFF);
}

/**
 * @brief Write data to ov5642 sensor using I2C multiple register
 *
 * @param reglist structure whis has all the registers and value
 * that have to be writen to the sensor
 */
int writeSensorRegs16_8(const struct sensor_reg reglist[])
{
	unsigned int reg_addr;
	unsigned char reg_val;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xffff) | (reg_val != 0xff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);

		writeSensorReg16_8(reg_addr, reg_val);

		next++;
	}
	return 1;
}

/**
 * @brief Read data from ov5642 sensor using I2C single register
 *
 * @param regID register address
 * @param regDat pointer to data
 */
uint8_t readSensorReg16_8(uint16_t regID, uint8_t *regDat)
{
	uint8_t I2C_buf_register[2];

	I2C_buf_register[0] = regID >> 8;
	I2C_buf_register[1] = regID & 0x00FF;
	HAL_I2C_Master_Transmit(&hi2c1, 0x78, I2C_buf_register, 2, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, 0x79, regDat, 1, HAL_MAX_DELAY);
	HAL_Delay(1);

	return 1;
}

/**
 * @brief Write data to ov5642 sensor using I2C single register
 *
 * @param regID register address
 * @param regDat pointer to data
 */
uint8_t writeSensorReg16_8(int regID, int regDat)
{
	uint8_t I2C_buf_register[3];

	I2C_buf_register[0] = regID >> 8;
	I2C_buf_register[1] = regID & 0x00FF;
	I2C_buf_register[2] = regDat & 0x00FF;
	HAL_I2C_Master_Transmit(&hi2c1, 0x78, I2C_buf_register, 3, HAL_MAX_DELAY);

	return 1;
}

/**
 * @brief Write data to camera module register using SPI
 *
 * @param address register address
 * @param value data to be writen in the register
 */
void write_reg(int address, int value)
{
	uint8_t spi_recv_buf = 0;
	uint8_t spi_buf;

	spi_buf = address | 0x80;
	HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi3, &spi_buf, &spi_recv_buf, 1, 100);

	spi_buf = value;
	HAL_SPI_TransmitReceive(&hspi3, &spi_buf, &spi_recv_buf, 1, 100);
	HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_SET);

}

/**
 * @brief Read data to camera module register using SPI
 *
 * @param address register address
 *
 * @retval Returns read data from register
 *
 */
uint8_t read_reg(int address)
{
	uint8_t spi_recv_buf = 0;
	uint8_t spi_buf;

	spi_buf = address;
	HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi3, &spi_buf, &spi_recv_buf, 1, 100);

	spi_buf = 0x00;
	HAL_SPI_TransmitReceive(&hspi3, &spi_buf, &spi_recv_buf, 1, 100);
	HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_SET);
	return spi_recv_buf;

}

/**
 * @brief get a single bit from camera module register
 *
 * @param address register address
 * @param bit bit you want to read
 *
 * @retval Returns bit read from register
 */
uint8_t get_bit(uint8_t addr, uint8_t bit)
{
	uint8_t temp;
	temp = read_reg(addr & 0x7F);
	temp = temp & bit;
	return temp;
}

/**
 * @brief Disconnects the power supply of the camera module
 */
void turnCameraOff()
{
	HAL_GPIO_WritePin(Mosfet_controll_GPIO_Port, Mosfet_controll_Pin, 1);
}

/**
 * @brief Connect the power supply to the camera module
 */
void turnCameraOn()
{
	HAL_GPIO_WritePin(Mosfet_controll_GPIO_Port, Mosfet_controll_Pin, 0);
}


/**
 * @brief Set all camera module registers, to take a single picture
 */
void takePicture()
{
	  //Clear fifo flag
	  write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);

	  //reset fifo read flag
	  write_reg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);

	  ///Flush FIFO buffer
	  write_reg(ARDUCHIP_FIFO, FIFO_WRPTR_RST_MASK);

	  //Clear fifo flag
	  write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);

	  // Set the amount of pictures to be taken to 1
	  write_reg(0x01, 1);

	  // Start capture
	  write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

/**
 * @brief Function that recives a picture from camera module,
 * and saves it in micro SD card, in the given path
 *
 * @param fp Pointer to fail object struct
 */
void reciveAndSavePicture(FIL* fp)
{
	uint8_t buffer_TX[PAYLOAD_LEN] = {0x00};
	uint8_t buffer_RX[PAYLOAD_LEN] = {0x00};

	uint8_t lastByteFound = 0;
	uint32_t count = 0, var = 0;
	uint8_t tempData_last;

	while (1)
	{
		readSPIbuff(buffer_TX, buffer_RX, PAYLOAD_LEN);

		count = count +1;

		for (var = 0; var < PAYLOAD_LEN; ++var)
		{
			fresult = f_putc((unsigned char)buffer_RX[var], fp);
			if (checkForLastByte(buffer_RX[var], tempData_last))
			{
				lastByteFound = 1;
				HAL_GPIO_TogglePin(DIODE4_GPIO_Port, DIODE4_Pin); //Toogle if last bit is found
				break;
			}
			if (checkForFirstByte(buffer_RX[var], tempData_last))
			{
				HAL_GPIO_TogglePin(DIODE3_GPIO_Port, DIODE3_Pin); //Toogle if first bit is found
			}
			tempData_last = buffer_RX[var];
		}
		if (lastByteFound == 1)
		{
			lastByteFound = 0;
			if(var > 77)
			{
				SPSGRF_StartTx(buffer_TX, sizeof(buffer_TX));
			}
			break;
		}
	}

	HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_SET);

	// Show the amount of bytes sent to ground base station
	count = count * PAYLOAD_LEN;

}

/**
 * @brief Turn on IR filter, by changing H  bridge values
 */
void turnOnFilter()
{
	HAL_GPIO_WritePin(H_BRIDGE_nSLEEP_GPIO_Port, H_BRIDGE_nSLEEP_Pin, GPIO_PIN_SET);

	HAL_Delay(10);

	HAL_GPIO_WritePin(H_BRIDGE_EN1_GPIO_Port, H_BRIDGE_EN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(H_BRIDGE_EN2_GPIO_Port, H_BRIDGE_EN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(H_BRIDGE_IN1_GPIO_Port, H_BRIDGE_IN1_Pin, GPIO_PIN_SET);

	HAL_Delay(200);

	HAL_GPIO_WritePin(H_BRIDGE_EN1_GPIO_Port, H_BRIDGE_EN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(H_BRIDGE_EN2_GPIO_Port, H_BRIDGE_EN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(H_BRIDGE_IN1_GPIO_Port, H_BRIDGE_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(H_BRIDGE_nSLEEP_GPIO_Port, H_BRIDGE_nSLEEP_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Turn off IR filter, by changing H  bridge values
 */
void turnOffFilter()
{
	HAL_GPIO_WritePin(H_BRIDGE_nSLEEP_GPIO_Port, H_BRIDGE_nSLEEP_Pin, GPIO_PIN_SET);

	HAL_Delay(10);

	HAL_GPIO_WritePin(H_BRIDGE_EN1_GPIO_Port, H_BRIDGE_EN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(H_BRIDGE_EN2_GPIO_Port, H_BRIDGE_EN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(H_BRIDGE_IN1_GPIO_Port, H_BRIDGE_IN2_Pin, GPIO_PIN_SET);

	HAL_Delay(200);

	HAL_GPIO_WritePin(H_BRIDGE_EN1_GPIO_Port, H_BRIDGE_EN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(H_BRIDGE_EN2_GPIO_Port, H_BRIDGE_EN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(H_BRIDGE_IN1_GPIO_Port, H_BRIDGE_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(H_BRIDGE_nSLEEP_GPIO_Port, H_BRIDGE_nSLEEP_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Turn off IR filter, by changing H  bridge values
 *
 * @param fp Pointer to time structure
 * @param fp Pointer to date structure
 */
void getTimeAndDate(RTC_TimeTypeDef *sTime, RTC_DateTypeDef *sDate)
{
	HAL_RTC_GetTime(&hrtc, sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, sDate, RTC_FORMAT_BIN);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
