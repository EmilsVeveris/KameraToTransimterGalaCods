/*
 * Spirit1_function.c
 *
 *Functions taken from https://forum.digikey.com/t/getting-started-with-the-spirit1-transceiver/15624
 *
 *  Created on: May 1, 2024
 *      Author: emils
 */


#include <Spirit1_function.h>



/**
 * @brief Initcilize spirit1 radiotransciever using defined varaibles from Spirit1_function.h
 */
void Spirit1_init()
{
	/*
 	 *
 	 *
 	 *
 	 * Spirit 1 init
 	 *
 	 *
 	 *
 	 */

 	SpiritEnterShutdown();
 	SpiritExitShutdown();
 	SpiritManagementWaExtraCurrent();

 	do {
 		for (volatile uint8_t i = 0; i != 0xFF; i++)
 			; // delay for state transition
 		SpiritRefreshStatus(); // reads the MC_STATUS register
 		HAL_GPIO_WritePin(DIODE3_GPIO_Port, DIODE3_Pin, GPIO_PIN_SET);
 	} while (g_xStatus.MC_STATE != MC_STATE_READY);
 	HAL_GPIO_WritePin(DIODE3_GPIO_Port, DIODE3_Pin, GPIO_PIN_RESET);
 	SRadioInit xRadioInit;

 	// Initialize radio RF parameters
 	xRadioInit.nXtalOffsetPpm = XTAL_OFFSET_PPM;
 	xRadioInit.lFrequencyBase = BASE_FREQUENCY;
 	xRadioInit.nChannelSpace = CHANNEL_SPACE;
 	xRadioInit.cChannelNumber = CHANNEL_NUMBER;
 	xRadioInit.xModulationSelect = MODULATION_SELECT;
 	xRadioInit.lDatarate = DATARATE;
 	xRadioInit.lFreqDev = FREQ_DEVIATION;
 	xRadioInit.lBandwidth = BANDWIDTH;
 	SpiritRadioSetXtalFrequency(XTAL_FREQUENCY); // Must be called before SpiritRadioInit()
 	SpiritRadioInit(&xRadioInit);

 	// Set the transmitter power level
 	SpiritRadioSetPALeveldBm(POWER_INDEX, POWER_DBM);
 	SpiritRadioSetPALevelMaxIndex(POWER_INDEX);

 	PktBasicInit xBasicInit;
 	PktBasicAddressesInit xBasicAddress;

 	// Configure packet handler to use the Basic packet format
 	xBasicInit.xPreambleLength = PREAMBLE_LENGTH;
 	xBasicInit.xSyncLength = SYNC_LENGTH;
 	xBasicInit.lSyncWords = SYNC_WORD;
 	xBasicInit.xFixVarLength = LENGTH_TYPE;
 	xBasicInit.cPktLengthWidth = LENGTH_WIDTH;
 	xBasicInit.xCrcMode = CRC_MODE;
 	xBasicInit.xControlLength = CONTROL_LENGTH;
 	xBasicInit.xAddressField = EN_ADDRESS;
 	xBasicInit.xFec = EN_FEC;
 	xBasicInit.xDataWhitening = EN_WHITENING;
 	SpiritPktBasicInit(&xBasicInit);

 	// Configure destination address criteria for automatic packet filtering
 	xBasicAddress.xFilterOnMyAddress = EN_FILT_MY_ADDRESS;
 	xBasicAddress.cMyAddress = MY_ADDRESS;
 	xBasicAddress.xFilterOnMulticastAddress = EN_FILT_MULTICAST_ADDRESS;
 	xBasicAddress.cMulticastAddress = MULTICAST_ADDRESS;
 	xBasicAddress.xFilterOnBroadcastAddress = EN_FILT_BROADCAST_ADDRESS;
 	xBasicAddress.cBroadcastAddress = BROADCAST_ADDRESS;
 	SpiritPktBasicAddressesInit(&xBasicAddress);

 	SGpioInit xGpioInit;

 	// Configure GPIO3 as interrupt request pin (active low)
 	xGpioInit.xSpiritGpioPin = SPIRIT_GPIO_3;
 	xGpioInit.xSpiritGpioMode = SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP;
 	xGpioInit.xSpiritGpioIO = SPIRIT_GPIO_DIG_OUT_IRQ;
 	SpiritGpioInit(&xGpioInit);

 	// Generate an interrupt request for the following IRQs
 	SpiritIrqDeInit(NULL);
 	SpiritIrq(TX_DATA_SENT, S_ENABLE);
 	SpiritIrq(RX_DATA_READY, S_ENABLE);
 	SpiritIrq(RX_DATA_DISC, S_ENABLE);
 	SpiritIrq(RX_TIMEOUT, S_ENABLE);
 	SpiritIrqClearStatus();

 	// Enable the synchronization quality indicator check (perfect match required)
 	// NOTE: 9.10.4: "It is recommended to always enable the SQI check."
 	SpiritQiSetSqiThreshold(SQI_TH_0);
 	SpiritQiSqiCheck(S_ENABLE);

 	// Set the RSSI Threshold for Carrier Sense (9.10.2)
 	// NOTE: CS_MODE = 0 at reset
 	SpiritQiSetRssiThresholddBm(RSSI_THRESHOLD);

 	// Configure the RX timeout
	#ifdef RECEIVE_TIMEOUT
		 SpiritTimerSetRxTimeoutMs(2000.0);
	#else
		 SET_INFINITE_RX_TIMEOUT();
	#endif /* RECIEVE_TIMEOUT */
 	SpiritTimerSetRxTimeoutStopCondition(SQI_ABOVE_THRESHOLD);

 	SpiritPktBasicSetDestinationAddress(0x44);
}

/**
 * @brief Start data transmisson
 *
 * @param txBuff Pointer to buffer which will be transmited
 * @param txLen Buffer length
 */
void SPSGRF_StartTx(uint8_t *txBuff, uint8_t txLen) {
	// flush the TX FIFO
	SpiritCmdStrobeFlushTxFifo();

	// Avoid TX FIFO overflow
	//txLen = (txLen > MAX_BUFFER_LEN ? txLen : MAX_BUFFER_LEN);
	txLen = (txLen > MAX_BUFFER_LEN ? MAX_BUFFER_LEN : txLen);

	// start TX operation
	SpiritSpiWriteLinearFifo(txLen, txBuff);
	SpiritPktBasicSetPayloadLength(txLen);
	SpiritCmdStrobeTx();
}
/**
 * @brief Start data reciving
 */
void SPSGRF_StartRx(void)
{
  SpiritCmdStrobeRx();
}

/**
 * @brief Get recived data
 *
 * @param rxBuff Pointer to buffer where recived data wil lbe saved
 *
 * @retval recived data length
 */
uint8_t SPSGRF_GetRxData(uint8_t *rxBuff)
{
  uint8_t len;

  len = SpiritLinearFifoReadNumElementsRxFifo();
  SpiritSpiReadLinearFifo(len, rxBuff);

  return len;
}
