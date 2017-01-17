// LTC68041 library
#include "LTC68041.h"
#include "nodeMiscHelpers.h"

extern CRC_HandleTypeDef hcrc;
extern osSemaphoreId bmsTRxCompleteHandle;

// Writes a data[REG_LEN] to register at address
// All boards on the stack will be written with the same data to the address
int8_t ltc68041_writeRegGroup(bmsChainHandleTypeDef * hbms, uint16_t address, uint8_t * data){
	uint16_t tempPEC;

	// Assemble command frame
	(hbms->spiTxBuf)[0] = (address >> 8) & 0xFF;
	(hbms->spiTxBuf)[1] = address & 0xFF;
	//tempPEC = ltc68041_calculatePEC(hbms->spiTxBuf, CMD_LEN);
	tempPEC = pec15_calc(CMD_LEN, hbms->spiTxBuf);
	(hbms->spiTxBuf)[2] = (tempPEC >> 8) & 0xFF;
	(hbms->spiTxBuf)[3] = tempPEC & 0xFF;

	// Assemble the data for each board on the chain
	for(uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++){
		// Write the data into the buffer
		for(uint8_t current_byte = 0; current_byte < REG_LEN; current_byte++){
			(hbms->spiTxBuf)[current_byte + current_ic*TX_REG_LEN + TX_CMD_LEN] = data[current_byte];
		}

		// Write the corresponding PEC into the Tx buffer
		tempPEC = pec15_calc(REG_LEN, data);
		(hbms->spiTxBuf)[current_ic*TX_REG_LEN + TX_CMD_LEN + REG_LEN] = (tempPEC >> 8) & 0xFF;
		(hbms->spiTxBuf)[current_ic*TX_REG_LEN + TX_CMD_LEN + REG_LEN + 1] = tempPEC & 0xFF;;
	}

	wakeup_idle (); 	//This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.

	// Wait for the SPI peripheral to finish TXing if it's busy
	while(!((HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_READY) ||
	  (HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_BUSY_RX)))
	{
	  osDelay(1);
	}

	// Transmit the command via DMA
	HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_RESET);
	return HAL_SPI_Transmit_DMA(hbms->hspi, hbms->spiTxBuf, TX_CMD_LEN+(TX_REG_LEN*TOTAL_IC));
}

// Writes a single command frame
int8_t ltc68041_writeCommand(bmsChainHandleTypeDef * hbms, uint16_t cmd){
	uint16_t tempPEC;

	// Assemble command frame
	(hbms->spiTxBuf)[0] = (cmd >> 8) & 0xFF;
	(hbms->spiTxBuf)[1] = cmd & 0xFF;
	//tempPEC = ltc68041_calculatePEC(hbms->spiTxBuf, CMD_LEN);
	tempPEC = pec15_calc(CMD_LEN, hbms->spiTxBuf);
	(hbms->spiTxBuf)[2] = (tempPEC >> 8) & 0xFF;
	(hbms->spiTxBuf)[3] = tempPEC & 0xFF;

	wakeup_idle (); 	//This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.

	// Wait for the SPI peripheral to finish TXing if it's busy
	while(!((HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_READY) ||
	  (HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_BUSY_RX)))
	{
	  osDelay(1);
	}

	// Transmit the command via DMA
	HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_RESET);
	return HAL_SPI_Transmit_DMA(hbms->hspi, hbms->spiTxBuf, TX_CMD_LEN);
}

// Read a register group at address and have the data stored
int8_t ltc68041_readRegGroup(bmsChainHandleTypeDef * hbms, uint16_t address){
	uint16_t tempPEC;
	int8_t retCode = 0;

	// Clear Tx Buffer
	for(uint8_t i = 4 ; i < TX_CMD_LEN + TOTAL_IC * (TX_REG_LEN);i++){
		(hbms->spiTxBuf)[i] = 0;
	}

	// Assemble command frame
	(hbms->spiTxBuf)[0] = (address >> 8) & 0xFF;
	(hbms->spiTxBuf)[1] = address & 0xFF;
	tempPEC = pec15_calc(CMD_LEN, hbms->spiTxBuf);
	(hbms->spiTxBuf)[2] = (tempPEC >> 8) & 0xFF;
	(hbms->spiTxBuf)[3] = tempPEC & 0xFF;

	wakeup_idle (); 	//This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.

	// Wait for the SPI peripheral to finish TXing if it's busy
	while(!((HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_READY) ||
	  (HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_BUSY_RX)))
	{
	  osDelay(1);
	}

	// Flush spi Rx FIFO
	while(__HAL_SPI_GET_FLAG(hbms->hspi, SPI_FLAG_RXNE)){
		uint32_t garbage = hbms->hspi->Instance->DR;
	}

	// Transmit the command via DMA
	HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(hbms->hspi, hbms->spiTxBuf, hbms->spiRxBuf, TX_CMD_LEN+(TX_REG_LEN*TOTAL_IC));

	// Suspend until we get the semaphore that the transmission is
	xSemaphoreTake(bmsTRxCompleteHandle, portMAX_DELAY);

	// The data now sits in the spiRxBuf, with an offset of TX_CMD_LEN since the Rx FIFO is active during Tx
	// PEC check
	for(uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic ++){
		uint16_t RxPEC = ((hbms->spiRxBuf)[current_ic * TX_REG_LEN + TX_CMD_LEN + REG_LEN] << 8) +
				(hbms->spiRxBuf)[current_ic * TX_REG_LEN + TX_CMD_LEN + REG_LEN + 1];
		tempPEC = pec15_calc(REG_LEN, &((hbms->spiRxBuf)[TX_CMD_LEN + current_ic * TX_REG_LEN]));
		if(RxPEC != tempPEC){
			retCode--;
		}
	}
	return retCode;
}

/*!****************************************************
  \brief Wake the LTC6804 from the sleep state

 Generic wakeup commannd to wake the LTC6804 from sleep
 *****************************************************/
void wakeup_sleep()
{
  HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_RESET);
  osDelay(1); // Guarantees the LTC6804 will be in standby; soft delay
  HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_SET);
}

/*!****************************************************
  \brief Wake isoSPI up from idle state
 Generic wakeup commannd to wake isoSPI up out of idle
 *****************************************************/
void wakeup_idle()
{
  HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_RESET);
  delayUs(2); //Guarantees the isoSPI will be in ready mode
  HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_SET);
}




static const unsigned int crc15Table[256] = {0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  //!<precomputed CRC15 Table
						 0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
						 0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
						 0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
						 0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
						 0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
						 0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
						 0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
						 0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
						 0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
						 0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
						 0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
						 0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
						 0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
						 0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
						 0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
						 0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
						 0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
						 0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
						 0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
						 0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
						 0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
						 0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095};

uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
					uint8_t *data //Array of data that will be used to calculate  a PEC
					)
{
	uint16_t remainder,addr;

	remainder = 16;//initialize the PEC
	for(uint8_t i = 0; i<len;i++) // loops for each byte in data array
	{
		addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
		remainder = (remainder<<8)^crc15Table[addr];
	}
	return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

// PEC Calculator
uint16_t ltc68041_calculatePEC(uint8_t * data, uint8_t len){
	return HAL_CRC_Calculate(&hcrc , (data), len);	// Use STM32's CRC peripheral
}

// User-exposed API
// Initialization function with all the selfTests
uint8_t ltc68041_Initialize(bmsChainHandleTypeDef * hbms){

}

// Self-tests
uint8_t ltc68041_cvTest(bmsChainHandleTypeDef * hbms){

}

uint8_t ltc68041_auxTest(bmsChainHandleTypeDef * hbms){

}

uint8_t ltc68041_statTest(bmsChainHandleTypeDef * hbms){

}

uint8_t ltc68041_adstatTest(bmsChainHandleTypeDef * hbms){

}

// Clear register operations
uint8_t ltc68041_clearCell(bmsChainHandleTypeDef * hbms){

}

uint8_t ltc68041_clearAux(bmsChainHandleTypeDef * hbms){

}

uint8_t ltc68041_clearStat(bmsChainHandleTypeDef * hbms){

}

// Start conversion
uint8_t ltc68041_startCVConv(bmsChainHandleTypeDef * hbms){

}

uint8_t ltc68041_startAUXConv(bmsChainHandleTypeDef * hbms){

}

uint8_t ltc68041_startCVAUXConv(bmsChainHandleTypeDef * hbms){

}

// LTC6804 pass through SPI/I2C API
uint8_t ltc68041_writeCOMM(bmsChainHandleTypeDef * hbms){

}

uint8_t ltc68041_readOMM(bmsChainHandleTypeDef * hbms){

}

uint8_t ltc68041_startCOMM(bmsChainHandleTypeDef * hbms){

}

// Utilities
uint8_t ltc68041_muxTest(bmsChainHandleTypeDef * hbms){

}

uint8_t ltc68041_owTest(bmsChainHandleTypeDef * hbms){

}

uint8_t ltc68041_convStat(bmsChainHandleTypeDef * hbms){

}
