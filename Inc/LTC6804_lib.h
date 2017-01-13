/************************************
REVISION HISTORY
$Revision: 1000 $
$Date: 2013-07-15 

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.

Copyright 2013 Linear Technology Corp. (LTC)
***********************************************************/

/*! @file
    @ingroup LTC68041
    Header for LTC6804-1 Multicell Battery Monitor
*/
 
#ifndef LTC68041_H
#define LTC68041_H

#include "stm32l4xx_hal.h"

#define TOTAL_IC	3	// Number of LTC6804-1s Stacked

#ifndef LTC6804_CS
#define LTC6804_CS QUIKEVAL_CS
#endif

 /*! 
 
  |MD| Dec  | ADC Conversion Model|
  |--|------|---------------------|
  |01| 1    | Fast 			   	  |
  |10| 2    | Normal 			  |
  |11| 3    | Filtered 		   	  |
*/
#define MD_FAST 1
#define MD_NORMAL 2
#define MD_FILTERED 3


 /*! 
 |CH | Dec  | Channels to convert |
 |---|------|---------------------|
 |000| 0    | All Cells  		  |
 |001| 1    | Cell 1 and Cell 7   |
 |010| 2    | Cell 2 and Cell 8   |
 |011| 3    | Cell 3 and Cell 9   |
 |100| 4    | Cell 4 and Cell 10  |
 |101| 5    | Cell 5 and Cell 11  |
 |110| 6    | Cell 6 and Cell 12  |
*/

#define CELL_CH_ALL 0
#define CELL_CH_1and7 1
#define CELL_CH_2and8 2
#define CELL_CH_3and9 3
#define CELL_CH_4and10 4
#define CELL_CH_5and11 5
#define CELL_CH_6and12 6


/*!

  |CHG | Dec  |Channels to convert   | 
  |----|------|----------------------|
  |000 | 0    | All GPIOS and 2nd Ref| 
  |001 | 1    | GPIO 1 			     |
  |010 | 2    | GPIO 2               |
  |011 | 3    | GPIO 3 			  	 |
  |100 | 4    | GPIO 4 			  	 |
  |101 | 5    | GPIO 5 			 	 |
  |110 | 6    | Vref2  			  	 |
*/

#define AUX_CH_ALL 0
#define AUX_CH_GPIO1 1
#define AUX_CH_GPIO2 2
#define AUX_CH_GPIO3 3
#define AUX_CH_GPIO4 4
#define AUX_CH_GPIO5 5
#define AUX_CH_VREF2 6

//uint8_t CHG = 0; //!< aux channels to be converted
 /*!****************************************************
  \brief Controls if Discharging transitors are enabled
  or disabled during Cell conversions.
  
 |DCP | Discharge Permitted During conversion |
 |----|----------------------------------------|
 |0   | No - discharge is not permitted         |
 |1   | Yes - discharge is permitted           |
 
********************************************************/
#define DCP_DISABLED	0
#define DCP_ENABLED 	1

#define BYTES_IN_REG 	8		// Bytes in each register + 2 bytes PEC
#define REG_BYTES		6		// Bytes in each register
#define CMD_LEN			4		// Command length
#define CELL_IN_REG		3		// Number of cell measurements per register group
#define GPIO_IN_REG		3		// Number of GPIO measurements per register group
#define NUM_AUX_REG		2		// Number of AUX register groups


typedef struct {
	SPI_HandleTypeDef * hspi;								// SPI Handle for this chain
	uint8_t		ADCV[2];									// Global ADCV register template
	uint8_t		ADAX[2];									// Global ADAX register template
	uint8_t 	spiRxBuf[TOTAL_IC * BYTES_IN_REG];			// SPI Receive Buffer
	uint8_t	 	spiTxBuf[TOTAL_IC * BYTES_IN_REG];			// SPI Transmit Buffer
	uint8_t		boardConfigs[TOTAL_IC][REG_BYTES];			// All the boards' configurations on the stack
	uint16_t 	auxVolts[TOTAL_IC][REG_BYTES];				// Stores the auxiliary GPIO measurement data
	uint16_t	cellVolts[TOTAL_IC][12];					// Stores the cell voltage measurement data
} ltc68041ChainHandle;

typedef struct {
	uint8_t 	refon;		// Reference on/off
	uint8_t		swtrd;		// Software discharge timer
	uint8_t 	adcMode;	// ADC mode group selection
	uint16_t	vov;		// Overvoltage set point
	uint16_t	vuv;		// Unvervoltage set point
	uint16_t	dcc;		// Cell discharge control
	uint8_t		dcto;		// Discharge timeout
}ltc68041ChainInitStruct;

int8_t LTC6804_rdaux(ltc68041ChainHandle * hbms, uint8_t reg);
void set_adc(ltc68041ChainHandle * hbms, uint8_t MD, uint8_t DCP, uint8_t CH, uint8_t CHG);
void LTC6804_adax(ltc68041ChainHandle * hbms);
void LTC6804_adcv(ltc68041ChainHandle * hbms);
uint8_t LTC6804_rdcv(ltc68041ChainHandle * hbms, uint8_t reg);
void LTC6804_rdcv_reg(ltc68041ChainHandle * hbms, uint8_t reg);
void LTC6804_rdaux_reg(ltc68041ChainHandle * hbms, uint8_t reg);
void LTC6804_clrcell(ltc68041ChainHandle * hbms);
void LTC6804_clraux(ltc68041ChainHandle * hbms);
void LTC6804_wrcfg(ltc68041ChainHandle * hbms);
int8_t LTC6804_rdcfg(ltc68041ChainHandle * hbms);
void wakeup_sleep();
uint16_t pec15_calc(uint8_t len, uint8_t *data);

#endif
