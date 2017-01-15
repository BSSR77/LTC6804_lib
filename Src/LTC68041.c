// LTC68041 library
#include "LTC68041.h"

extern CRC_HandleTypeDef hcrc;

// Writes a data[REG_LEN] to register at address
int8_t ltc68041_writeRegGroup(uint16_t address, uint8_t * data){

}

// Read a register group at address and have the data stored
int8_t ltc68041_readRegGroup(uint16_t address){

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

// PEC Calculator
inline uint16_t ltc68041_calculatePEC(uint8_t * data, uint8_t len){
	return HAL_CRC_Calculate(&hcrc , (uint32_t*)(data), len);	// Use STM32's CRC peripheral
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
