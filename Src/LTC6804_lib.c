/*
 * LTC6804_lib.c
 *
 *  Created on: Jan 12, 2017
 *      Author: frank
 */

#include "LTC6804_lib.h"
#include "cmsis_os.h"
#include "stm32l4xx_hal.h"
#include "nodeMiscHelpers.h"

extern CRC_HandleTypeDef hcrc;
extern osSemaphoreId bmsTRxCompleteHandle;


/*
 * To initialize:
 * set_adc(hbms, MD_NORMAL,DCP_DISABLED,CELL_CH_ALL,AUX_CH_ALL);
 *
 * Create a global ltc68041ChainHandle in main.c
 */

// CRC calculations are provided by the STM32-CRC peripheral
/*!**********************************************************
 \brief calaculates  and returns the CRC15

  @param[in] uint8_t len: the length of the data array being passed to the function

  @param[in] uint8_t data[] : the array of data that the PEC will be generated from


  @returns The calculated pec15 as an unsigned int
***********************************************************/
inline uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
					uint8_t *data //Array of data that will be used to calculate  a PEC
					)
{
	return HAL_CRC_Calculate(&hcrc , (uint32_t*)(data), len);	// Use STM32's CRC peripheral
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



/*!******************************************************
 \brief Reads configuration registers of a LTC6804 daisy chain

@param[in] uint8_t total_ic: number of ICs in the daisy chain

@param[out] uint8_t r_config[][8] is a two dimensional array that the function stores the read configuration data. The configuration data for each IC
is stored in blocks of 8 bytes with the configuration data of the lowest IC on the stack in the first 8 bytes
block of the array, the second IC in the second 8 byte etc. Below is an table illustrating the array organization:

|r_config[0][0]|r_config[0][1]|r_config[0][2]|r_config[0][3]|r_config[0][4]|r_config[0][5]|r_config[0][6]  |r_config[0][7] |r_config[1][0]|r_config[1][1]|  .....    |
|--------------|--------------|--------------|--------------|--------------|--------------|----------------|---------------|--------------|--------------|-----------|
|IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC1 PEC High    |IC1 PEC Low    |IC2 CFGR0     |IC2 CFGR1     |  .....    |


@return int8_t, PEC Status.

	0: Data read back has matching PEC

	-1: Data read back has incorrect PEC


Command Code:
-------------

|CMD[0:1]		|  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDCFG:	        |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   1   |   0   |
********************************************************/
int8_t LTC6804_rdcfg(ltc68041ChainHandle * hbms)
{
  int8_t pec_error = 0;
  uint16_t data_pec;
  uint16_t received_pec;

  //1
  // Assemble command bytes + pec15
  (hbms->spiTxBuf)[0] = 0x00;
  (hbms->spiTxBuf)[1] = 0x02;
  (hbms->spiTxBuf)[2] = 0x2b;
  (hbms->spiTxBuf)[3] = 0x0A;

  //2
  wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  //3
  // Wait for the SPI peripheral to finish TXing if it's busy
  while(!((HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_READY) ||
		  (HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_BUSY_RX))){
	  osDelay(1);
  }
  HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_DMA(hbms->hspi, hbms->spiTxBuf, hbms->spiRxBuf, CMD_LEN + BYTES_IN_REG * TOTAL_IC );
  //Read the configuration data of all ICs on the daisy chain into the handle's storage arrays

  // Suspend until we get the semaphore that the transmission is
  xSemaphoreTake(bmsTRxCompleteHandle, portMAX_DELAY);
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++) 			//executes for each LTC6804 in the daisy chain and packs the data
  { 																			//into the r_config array as well as check the received Config data
																				//for any bit errors
	//4.a
    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
      (hbms->boardConfigs)[current_ic][current_byte] = (hbms->spiRxBuf)[current_byte + (current_ic*BYTES_IN_REG)];
    }
    //4.b
    received_pec = ((hbms->boardConfigs)[current_ic][6]<<8) + (hbms->boardConfigs)[current_ic][7];	// Extract the last 2 bytes of the received data buffer
    data_pec = pec15_calc(6, &(hbms->boardConfigs)[current_ic][0]);
    if(received_pec != data_pec)
    {
      pec_error = -1 * current_ic;
      // Don't terminate on a bad PEC
    }
  }

  //5
  return(pec_error);
}
/*
	RDCFG Sequence:

	1. Load cmd array with the write configuration command and PEC
	2. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
	3. Send command and read back configuration data
	4. For each LTC6804 in the daisy chain
	  a. load configuration data into r_config array
	  b. calculate PEC of received data and compare against calculated PEC
	5. Return PEC Error

*/



/*****************************************************//**
 \brief Write the LTC6804 configuration register

 This command will write the configuration registers of the LTC6804-1s
 connected in a daisy chain stack. The configuration is written in descending
 order so the last device's configuration is written first.

 @param[in] uint8_t total_ic; The number of ICs being written to.

 @param[in] uint8_t config[][6] is a two dimensional array of the configuration data that will be written, the array should contain the 6 bytes for each
 IC in the daisy chain. The lowest IC in the daisy chain should be the first 6 byte block in the array. The array should
 have the following format:
 |  config[0][0]| config[0][1] |  config[0][2]|  config[0][3]|  config[0][4]|  config[0][5]| config[1][0] |  config[1][1]|  config[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |

 The function will calculate the needed PEC codes for the write data
 and then transmit data to the ICs on a daisy chain.


Command Code:
-------------
|               |							CMD[0]                              |                            CMD[1]                             |
|---------------|---------------------------------------------------------------|---------------------------------------------------------------|
|CMD[0:1]	    |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|WRCFG:	        |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |
********************************************************/
void LTC6804_wrcfg(ltc68041ChainHandle * hbms)
{
  uint16_t cfg_pec;
  uint8_t cmd_index; //command counter

  //1 - Assemble command and pec for the first board
  (hbms->spiTxBuf)[0] = 0x00;
  (hbms->spiTxBuf)[1] = 0x01;
  (hbms->spiTxBuf)[2] = 0x3d;
  (hbms->spiTxBuf)[3] = 0x6e;

  //2
  cmd_index = 4;
  for (uint8_t current_ic = TOTAL_IC; current_ic > 0; current_ic--) 			// executes for each LTC6804 in daisy chain, this loops starts with
  {																				// the last IC on the stack. The first configuration written is
																				// received by the last IC in the daisy chain

    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each of the 6 bytes in the CFGR register
    {																			// current_byte is the byte counter

    	(hbms->spiTxBuf)[cmd_index] = (hbms->boardConfigs)[current_ic-1][current_byte]; 						//adding the config data to the array to be sent
      cmd_index = cmd_index + 1;
    }
	//3
    cfg_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &((hbms->boardConfigs)[current_ic-1][0]));		// calculating the PEC for each ICs configuration register data
    (hbms->spiTxBuf)[cmd_index] = (uint8_t)(cfg_pec >> 8);
    (hbms->spiTxBuf)[cmd_index + 1] = (uint8_t)cfg_pec;
    cmd_index = cmd_index + 2;
  }

  //4
  wakeup_idle (); 															 	//This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.

  //5
  // Wait for the SPI peripheral to finish TXing if it's busy
  while(!((HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_READY) ||
	  (HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_BUSY_RX)))
  {
	  osDelay(1);
  }
  // Transmit the command via DMA
  HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit_DMA(hbms->hspi, hbms->spiTxBuf, CMD_LEN+(8*TOTAL_IC));
}
/*
	WRCFG Sequence:

	1. Load cmd array with the write configuration command and PEC
	2. Load the cmd with LTC6804 configuration data
	3. Calculate the pec for the LTC6804 configuration data being transmitted
	4. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
	5. Write configuration data to the LTC6804 daisy chain

*/

/***********************************************************//**
 \brief Clears the LTC6804 Auxiliary registers

 The command clears the Auxiliary registers and intiallizes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.


Command Code:
-------------

|CMD[0:1]	    |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|CLRAUX:	    |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |   0   |   0   |   1   |   0   |   0   |   2   |   0   |
***************************************************************/
void LTC6804_clraux(ltc68041ChainHandle * hbms)
{
  uint16_t cmd_pec;

  //1
  (hbms->spiTxBuf)[0] = 0x07;
  (hbms->spiTxBuf)[1] = 0x12;

  //2
  cmd_pec = pec15_calc(2, (hbms->spiTxBuf));
  (hbms->spiTxBuf)[2] = (uint8_t)(cmd_pec >> 8);
  (hbms->spiTxBuf)[3] = (uint8_t)(cmd_pec);

  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.

  //4
  // Wait for the SPI peripheral to finish TXing if it's busy
  while(!((HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_READY) ||
    (HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_BUSY_RX)))
  {
    osDelay(1);
  }
  // Transmit the command via DMA
  HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit_DMA(hbms->hspi, hbms->spiTxBuf, CMD_LEN);
}
/*
  LTC6804_clraux Function sequence:

  1. Load clraux command into cmd array
  2. Calculate clraux cmd PEC and load pec into cmd array
  3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  4. send broadcast clraux command to LTC6804 daisy chain
*/

/********************************************************//**
 \brief Clears the LTC6804 cell voltage registers

 The command clears the cell voltage registers and intiallizes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.


Command Code:
-------------

|CMD[0:1]	    |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|CLRCELL:	    |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |   0   |   0   |   1   |   0   |   0   |   0   |   1   |
************************************************************/
void LTC6804_clrcell(ltc68041ChainHandle * hbms)
{
  uint16_t cmd_pec;

  //1
  (hbms->spiTxBuf)[0] = 0x07;
  (hbms->spiTxBuf)[1] = 0x11;

  //2
  cmd_pec = pec15_calc(2, (hbms->spiTxBuf));
  (hbms->spiTxBuf)[2] = (uint8_t)(cmd_pec >> 8);
  (hbms->spiTxBuf)[3] = (uint8_t)(cmd_pec );

  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  //4
  // Wait for the SPI peripheral to finish TXing if it's busy
  while(!((HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_READY) ||
       (HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_BUSY_RX)))
  {
    osDelay(1);
  }
  // Transmit the command via DMA
  HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit_DMA(hbms->hspi, hbms->spiTxBuf, CMD_LEN);
}
/*
  LTC6804_clrcell Function sequence:

  1. Load clrcell command into cmd array
  2. Calculate clrcell cmd PEC and load pec into cmd array
  3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  4. send broadcast clrcell command to LTC6804 daisy chain
*/



/***********************************************//**
 \brief Read the raw data from the LTC6804 auxiliary register

 The function reads a single GPIO voltage register and stores thre read data
 in the *data point as a byte array. This function is rarely used outside of
 the LTC6804_rdaux() command.

 @param[in] uint8_t reg; This controls which GPIO voltage register is read back.

          1: Read back auxiliary group A

          2: Read back auxiliary group B


@param[in] uint8_t total_ic; This is the number of ICs in the daisy chain

@param[out] uint8_t *data; An array of the unparsed aux codes



Command Code:
-------------

|CMD[0:1]	    |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDAUXA:	    |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   0   |   0   |
|RDAUXB:	    |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |

 *************************************************/
void LTC6804_rdaux_reg(ltc68041ChainHandle * hbms, uint8_t reg)
{
  const uint8_t REG_LEN = 8; // number of bytes in the register + 2 bytes for the PEC
  uint16_t cmd_pec;

  //1
  if (reg == 1)			//Read back auxiliary group A
  {
	  (hbms->spiTxBuf)[1] = 0x0C;
	  (hbms->spiTxBuf)[0] = 0x00;
  }
  else if(reg == 2)		//Read back auxiliary group B
  {
	  (hbms->spiTxBuf)[1] = 0x0e;
	  (hbms->spiTxBuf)[0] = 0x00;
  }
  else					//Read back auxiliary group A
  {
	  (hbms->spiTxBuf)[1] = 0x0C;
	  (hbms->spiTxBuf)[0] = 0x00;
  }
  //2
  cmd_pec = pec15_calc(2, (hbms->spiTxBuf));
  (hbms->spiTxBuf)[2] = (uint8_t)(cmd_pec >> 8);
  (hbms->spiTxBuf)[3] = (uint8_t)(cmd_pec);

  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.

  //4
  // Wait for the SPI peripheral to finish TXing if it's busy
  while(!((HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_READY) ||
       (HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_BUSY_RX)))
  {
    osDelay(1);
  }
  // Transmit the command via DMA
  HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_DMA(hbms->hspi, hbms->spiTxBuf, hbms->spiRxBuf, CMD_LEN + (REG_LEN*TOTAL_IC));
}
/*
  LTC6804_rdaux_reg Function Process:
  1. Determine Command and initialize command array
  2. Calculate Command PEC
  3. Wake up isoSPI, this step is optional
  4. Send Global Command to LTC6804 daisy chain
*/


/***********************************************//**
 \brief Read the raw data from the LTC6804 cell voltage register

 The function reads a single cell voltage register and stores the read data
 in the *data point as a byte array. This function is rarely used outside of
 the LTC6804_rdcv() command.

 @param[in] uint8_t reg; This controls which cell voltage register is read back.

          1: Read back cell group A

          2: Read back cell group B

          3: Read back cell group C

          4: Read back cell group D

 @param[in] uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)

 @param[out] uint8_t *data; An array of the unparsed cell codes

Command Code:
-------------

|CMD[0:1]	|  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDCVA:	    |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |
|RDCVB:	    |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   0   |
|RDCVC:	    |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   0   |
|RDCVD:	    |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   1   |   0   |

 *************************************************/
void LTC6804_rdcv_reg(ltc68041ChainHandle * hbms, uint8_t reg) 	//Determines which cell voltage register is read back
{
  uint16_t cmd_pec;

  //1
  if (reg == 1)     //1: RDCVA
  {
    (hbms->spiTxBuf)[1] = 0x04;
    (hbms->spiTxBuf)[0] = 0x00;
  }
  else if(reg == 2) //2: RDCVB
  {
	  (hbms->spiTxBuf)[1] = 0x06;
	  (hbms->spiTxBuf)[0] = 0x00;
  }
  else if(reg == 3) //3: RDCVC
  {
	  (hbms->spiTxBuf)[1] = 0x08;
	  (hbms->spiTxBuf)[0] = 0x00;
  }
  else if(reg == 4) //4: RDCVD
  {
	  (hbms->spiTxBuf)[1] = 0x0A;
	  (hbms->spiTxBuf)[0] = 0x00;
  }

  //2
  cmd_pec = pec15_calc(2, (hbms->spiTxBuf));
  (hbms->spiTxBuf)[2] = (uint8_t)(cmd_pec >> 8);
  (hbms->spiTxBuf)[3] = (uint8_t)(cmd_pec);

  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  //4
  // Wait for the SPI peripheral to finish TXing if it's busy
  while(!((HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_READY) ||
    		(HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_BUSY_RX)))
  {
      osDelay(1);
  }
  // Transmit the command via DMA
  HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_DMA(hbms->hspi, hbms->spiTxBuf, hbms->spiRxBuf, CMD_LEN + (BYTES_IN_REG*TOTAL_IC));
}
/*
  LTC6804_rdcv_reg Function Process:
  1. Determine Command and initialize command array
  2. Calculate Command PEC
  3. Wake up isoSPI, this step is optional
  4. Send Global Command to LTC6804 daisy chain
*/

/***********************************************//**
 \brief Reads and parses the LTC6804 cell voltage registers.

 The function is used to read the cell codes of the LTC6804.
 This function will send the requested read commands parse the data
 and store the cell voltages in cell_codes variable.

 @param[in] uint8_t reg; This controls which cell voltage register is read back.

          0: Read back all Cell registers

          1: Read back cell group A

          2: Read back cell group B

          3: Read back cell group C

          4: Read back cell group D

 @param[in] uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)

 @param[out] uint16_t cell_codes[]; An array of the parsed cell codes from lowest to highest. The cell codes will
  be stored in the cell_codes[] array in the following format:
  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |

  @return int8_t, PEC Status.

		0: No PEC error detected

		-1: PEC error detected, retry read


 *************************************************/
uint8_t LTC6804_rdcv(ltc68041ChainHandle * hbms, uint8_t reg)
{
	uint8_t pec_error = 0;
	uint16_t parsed_cell;
	uint16_t received_pec;
	uint16_t data_pec;
	uint8_t data_counter = 0; //data counter
	//1.a
	if (reg == 0)	// Read back all registers
	{
		//a.i
		for(uint8_t cell_reg = 1; cell_reg<5; cell_reg++)         			 			//executes once for each of the LTC6804 cell voltage registers
		{
		  data_counter = 0;								// Reset the counter at every cycle
		  LTC6804_rdcv_reg(hbms, cell_reg);												// Reads a single Cell voltage register
		  xSemaphoreTake(bmsTRxCompleteHandle, portMAX_DELAY);							// Only proceeds when data is received

		  for (uint8_t current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) 			// executes for every LTC6804 in the daisy chain
		  {																 	  			// current_ic is used as the IC counter
			//a.ii
			for(uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++)	 	// This loop parses the read back data into cell voltages, it
			{														   		  			// loops once for each of the 3 cell voltage codes in the register
			  // The data offset is CMD_LEN to offset the data received during TX
			  parsed_cell = (hbms->spiRxBuf)[data_counter + CMD_LEN] + \
					  ((hbms->spiRxBuf)[data_counter + CMD_LEN + 1] << 8);				 // uint8_t combine into uint16_t

			  (hbms->cellVolts)[current_ic][current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
			  data_counter += 2;														 // Increment by 2 for uint16_t type
			}
			//a.iii
			// Last 2 bytes of the received data is the PEC
			received_pec = ((hbms->spiRxBuf)[data_counter + CMD_LEN] << 8) + (hbms->spiRxBuf)[data_counter + CMD_LEN + 1];
			data_pec = pec15_calc(REG_BYTES, &((hbms->spiRxBuf)[CMD_LEN + current_ic * BYTES_IN_REG]));

			if(received_pec != data_pec)
			{
			  pec_error = -1;															//The pec_error variable is simply set negative if any PEC errors
																						//are detected in the serial data
			}
			data_counter += 2;
		  }
		}
	}
	//1.b
	else
	{
		//b.i
		LTC6804_rdcv_reg(hbms, reg);
		xSemaphoreTake(bmsTRxCompleteHandle, portMAX_DELAY);							// Only proceeds when data is received

		for (uint8_t current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) 				// executes for every LTC6804 in the daisy chain
		{																 	  			// current_ic is used as the IC counter
			//b.ii
			for(uint8_t current_cell = 0; current_cell < CELL_IN_REG; current_cell++)   // This loop parses the read back data into cell voltages, it
			{														   		  			// loops once for each of the 3 cell voltage codes in the register
				 parsed_cell = (hbms->spiRxBuf)[data_counter + CMD_LEN] + \
								((hbms->spiRxBuf)[data_counter + CMD_LEN + 1] << 8);	// uint8_t combine into uint16_t

				(hbms->cellVolts)[current_ic][current_cell + ((reg - 1) * CELL_IN_REG)] = parsed_cell;
				data_counter += 2;
			}
			//b.iii
			received_pec = ((hbms->spiRxBuf)[data_counter + CMD_LEN] << 8) + (hbms->spiRxBuf)[data_counter + CMD_LEN + 1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
																						 //after the 6 cell voltage data bytes
			data_pec = pec15_calc(REG_BYTES, &((hbms->spiRxBuf)[current_ic * BYTES_IN_REG]));
			if(received_pec != data_pec)
			{
				pec_error = -1;															//The pec_error variable is simply set negative if any PEC errors
																						//are detected in the serial data
			}
			data_counter += 2;
		}
	}
	return(pec_error);
}
/*
	LTC6804_rdcv Sequence

	1. Switch Statement:
		a. Reg = 0
			i. Read cell voltage registers A-D for every IC in the daisy chain
			ii. Parse raw cell voltage data in cell_codes array
			iii. Check the PEC of the data read back vs the calculated PEC for each read register command
		b. Reg != 0
			i.Read single cell voltage register for all ICs in daisy chain
			ii. Parse raw cell voltage data in cell_codes array
			iii. Check the PEC of the data read back vs the calculated PEC for each read register command
	2. Return pec_error flag
*/



/*!*********************************************************************************************
  \brief Starts cell voltage conversion

  Starts ADC conversions of the LTC6804 Cpin inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CH     | Determines which cell channels are converted |
 | DCP    | Determines if Discharge is Permitted	     |

Command Code:
-------------

|CMD[0:1]	|  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADCV:	    |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |
***********************************************************************************************/
void LTC6804_adcv(ltc68041ChainHandle * hbms)
{
  uint16_t cmd_pec;

  //1
  (hbms->spiTxBuf)[0] = (hbms->ADCV)[0];
  (hbms->spiTxBuf)[1] = (hbms->ADCV)[1];

  //2
  cmd_pec = pec15_calc(2, hbms->ADCV);
  (hbms->spiTxBuf)[2] = (uint8_t)(cmd_pec >> 8);
  (hbms->spiTxBuf)[3] = (uint8_t)(cmd_pec);

  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  //4
  // Wait for the SPI peripheral to finish TXing if it's busy
   while(!((HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_READY) ||
     		(HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_BUSY_RX)))
   {
       osDelay(1);
   }
   // Transmit the command via DMA
   HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_RESET);
   HAL_SPI_Transmit_DMA(hbms->hspi, hbms->spiTxBuf, CMD_LEN);
}
/*
  LTC6804_adcv Function sequence:

  1. Load adcv command into cmd array
  2. Calculate adcv cmd PEC and load pec into cmd array
  3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  4. send broadcast adcv command to LTC6804 daisy chain
*/


/*!******************************************************************************************************
 \brief Start an GPIO Conversion

  Starts an ADC conversions of the LTC6804 GPIO inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CHG    | Determines which GPIO channels are converted |


Command Code:
-------------

|CMD[0:1]	|  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADAX:	    |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]|
*********************************************************************************************************/
void LTC6804_adax(ltc68041ChainHandle * hbms)
{
  uint16_t cmd_pec;

  (hbms->spiTxBuf)[0] = hbms->ADAX[0];
  (hbms->spiTxBuf)[1] = hbms->ADAX[1];
  cmd_pec = pec15_calc(2, hbms->ADAX);
  (hbms->spiTxBuf)[2] = (uint8_t)(cmd_pec >> 8);
  (hbms->spiTxBuf)[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  // Wait for the SPI peripheral to finish TXing if it's busy
   while(!((HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_READY) ||
     		(HAL_SPI_GetState(hbms->hspi) == HAL_SPI_STATE_BUSY_RX)))
   {
       osDelay(1);
   }
   // Transmit the command via DMA
   HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_RESET);
   HAL_SPI_Transmit_DMA(hbms->hspi, hbms->spiTxBuf, CMD_LEN);
}
/*
  LTC6804_adax Function sequence:

  1. Load adax command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  4. send broadcast adax command to LTC6804 daisy chain
*/


/*!*******************************************************************************************************************
 \brief Maps  global ADC control variables to the appropriate control bytes for each of the different ADC commands

@param[in] uint8_t MD The adc conversion mode
@param[in] uint8_t DCP Controls if Discharge is permitted during cell conversions
@param[in] uint8_t CH Determines which cells are measured during an ADC conversion command
@param[in] uint8_t CHG Determines which GPIO channels are measured during Auxiliary conversion command

Command Code:
-------------

|command	|  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADCV:	    |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |
|ADAX:	    |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]|
 ******************************************************************************************************************/
void set_adc(ltc68041ChainHandle * hbms,
			 uint8_t MD, //ADC Mode
			 uint8_t DCP, //Discharge Permit
			 uint8_t CH, //Cell Channels to be measured
			 uint8_t CHG //GPIO Channels to be measured
			 )
{
  uint8_t md_bits;

  // Setter functions; no SPI transmissions
  md_bits = (MD & 0x02) >> 1;
  (hbms->ADCV)[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  (hbms->ADCV)[1] =  md_bits + 0x60 + (DCP<<4) + CH;

  md_bits = (MD & 0x02) >> 1;
  (hbms->ADCV)[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  (hbms->ADCV)[1] = md_bits + 0x60 + CHG ;
}


/***********************************************************************************//**
 \brief Reads and parses the LTC6804 auxiliary registers.

 The function is used
 to read the  parsed GPIO codes of the LTC6804. This function will send the requested
 read commands parse the data and store the gpio voltages in aux_codes variable

@param[in] uint8_t reg; This controls which GPIO voltage register is read back.

          0: Read back all auxiliary registers

          1: Read back auxiliary group A

          2: Read back auxiliary group B


@param[in] uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)


 @param[out] uint16_t aux_codes[][6]; A two dimensional array of the gpio voltage codes. The GPIO codes will
 be stored in the aux_codes[][6] array in the following format:
 |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
 |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
 |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |

@return  int8_t, PEC Status

  0: No PEC error detected

 -1: PEC error detected, retry read
 *************************************************/
int8_t LTC6804_rdaux(ltc68041ChainHandle * hbms, uint8_t reg)
{
  uint8_t data_counter = 0;
  int8_t pec_error = 0;
  uint16_t parsed_aux;
  uint16_t received_pec;
  uint16_t data_pec;

  //1.a
  if (reg == 0)
  {
	//a.i
    for(uint8_t gpio_reg = 1; gpio_reg <= NUM_AUX_REG; gpio_reg++)		 	   		 			//executes once for each of the LTC6804 aux voltage registers
    {
      data_counter = 0;
      LTC6804_rdaux_reg(hbms, gpio_reg);											//Reads the raw auxiliary register data into the data[] array
      xSemaphoreTake(bmsTRxCompleteHandle, portMAX_DELAY);

      for (uint8_t current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) 			// executes for every LTC6804 in the daisy chain
      {																 	  			// current_ic is used as the IC counter
        //a.ii
		for(uint8_t current_gpio = 0; current_gpio< GPIO_IN_REG; current_gpio++)	// This loop parses the read back data into GPIO voltages, it
        {														   		  			// loops once for each of the 3 gpio voltage codes in the register
		  parsed_aux = (hbms->spiRxBuf)[data_counter + CMD_LEN] + ((hbms->spiRxBuf)[data_counter+CMD_LEN+1]<<8);
          (hbms->auxVolts)[current_ic][current_gpio +((gpio_reg-1)*GPIO_IN_REG)] = parsed_aux;
          data_counter += 2;
        }
		//a.iii
        received_pec = ((hbms->spiRxBuf)[data_counter + CMD_LEN]<<8) + (hbms->spiRxBuf)[data_counter+CMD_LEN+1];
        data_pec = pec15_calc(REG_BYTES, &(hbms->spiRxBuf)[current_ic*BYTES_IN_REG + CMD_LEN]);

        if(received_pec != data_pec)
        {
          pec_error = -1;
        }

        data_counter += 2;
      }
    }
  }
  else
  {
	//b.i
    LTC6804_rdaux_reg(hbms, reg);
    xSemaphoreTake(bmsTRxCompleteHandle, portMAX_DELAY);

    for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) 			  		// executes for every LTC6804 in the daisy chain
    {							   									          		// current_ic is used as an IC counter
		//b.ii
		for(int current_gpio = 0; current_gpio<GPIO_IN_REG; current_gpio++)  	 	// This loop parses the read back data. Loops
		{						 											  		// once for each aux voltage in the register
			parsed_aux = ((hbms->spiRxBuf)[data_counter+CMD_LEN] + ((hbms->spiRxBuf)[data_counter+CMD_LEN+1]<<8));
			(hbms->auxVolts)[current_ic][current_gpio +((reg-1)*GPIO_IN_REG)] = parsed_aux;
			data_counter += 2;
		}
		//b.iii
		received_pec = ((hbms->spiRxBuf)[data_counter+CMD_LEN]<<8) + (hbms->spiRxBuf)[data_counter+CMD_LEN+1];
        data_pec = pec15_calc(REG_BYTES, &((hbms->spiRxBuf)[current_ic*BYTES_IN_REG]));
        if(received_pec != data_pec)
        {
          pec_error = -1;
        }

		data_counter += 2;
    }
  }
  return (pec_error);
}
/*
	LTC6804_rdaux Sequence

	1. Switch Statement:
		a. Reg = 0
			i. Read GPIO voltage registers A-D for every IC in the daisy chain
			ii. Parse raw GPIO voltage data in cell_codes array
			iii. Check the PEC of the data read back vs the calculated PEC for each read register command
		b. Reg != 0
			i.Read single GPIO voltage register for all ICs in daisy chain
			ii. Parse raw GPIO voltage data in cell_codes array
			iii. Check the PEC of the data read back vs the calculated PEC for each read register command
	2. Return pec_error flag
*/

