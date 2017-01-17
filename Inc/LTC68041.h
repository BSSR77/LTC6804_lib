

#ifndef LTC68041_H_
#define LTC68041_H_

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "../../CAN_ID.h"

// BMS Chain defines
#define TOTAL_IC      1   // Number of boards in the stack

// LTC6804-1 Dependent defines
// Register lengths and numbers
#define REG_LEN       6   // The byte length of each register settings
#define PEC_LEN       2   // Length of the PEC-byte
#define CMD_LEN       2   // Command length
#define TX_CMD_LEN    (CMD_LEN + PEC_LEN)   // Length of command transmission
#define TX_REG_LEN    (REG_LEN + PEC_LEN)   // Length of register data transmission
#define CV_PER_REG    3   // Cell voltages per CV register group
#define CV_REG_NUM    4   // Cell voltage register group numbers
#define AUX_PER_REG   3   // Auxiliary GPIO measurements per AUX register group
#define AUX_REG_NUM   2   // Auxiliary GPIO measurement register group number
#define STAT_PER_REG  3   // Status bytes per STAT register group
#define STAT_REG_NUM  2   // Status register group number

// Command codes / templates
#define WRCFG         0x0001
#define RDCFG         0x0002
#define RDCVA         0x0004
#define RDCVB         0x0006
#define RDCVC         0x0008
#define RDCVD         0x000A
#define RDAUXA        0x000C
#define RDAUXB        0x000E
#define RDSTATA       0x0010
#define RDSTATB       0x0012
#define ADCV_T        0x0260  // Template
#define ADOW_T        0x0228  // Template
#define CVST_T        0x0207  // Template
#define ADAX_T        0x0460  // Template
#define AXST_T        0x0407  // Template
#define ADSTAT_T      0x0468  // Template
#define STATST_T      0x040F  // Template
#define ADCVAX_T      0x046F  // Template
#define CLRCELL       0x0711
#define CLRAUX        0x0712
#define CLRST         0x0713
#define PLAD          0x0714
#define DIAGN         0x0715
#define WRCOMM        0x0721
#define RDCOMM        0x0722
#define STCOMM        0x0723

// Self test (ST) configuration bits
#define SELF_TEST_1   0x1
#define SELF_TEST_2   0x2
#define ST_OFFSET     0x5   // Bit offset

// Discharge permitted
#define DCP_ON        0x1
#define DCP_OFF       0x0
#define DCP_OFFSET    0x4



typedef struct {
  uint8_t     CFGR[REG_LEN];                        // Configuration register
  uint16_t    CVR[CV_REG_NUM * CV_PER_REG];         // Cell voltage register group
  uint16_t    AUXR[AUX_REG_NUM * AUX_PER_REG];      // Auxiliary voltage register group
  uint16_t    STATR[STAT_REG_NUM * STAT_PER_REG];   // Status register group
  uint8_t     COMMR[3];         // Communication register - 3 bytes of data length
} LTC68041HandleTypeDef;

// spiRxBuf sized according to the size of reading a register group
// spiTxBuf sized according to size of cmd + register length + pec
typedef struct {
  SPI_HandleTypeDef * hspi;   // SPI Handle
  uint8_t   spiRxBuf[TX_CMD_LEN + TOTAL_IC * (TX_REG_LEN)];       // SPI receive buffer
  uint8_t   spiTxBuf[TX_CMD_LEN + TOTAL_IC * (TX_REG_LEN)];       // SPI transmit buffer
  LTC68041HandleTypeDef   board[TOTAL_IC];          // Handle buffer to store all the board info for each board
} bmsChainHandleTypeDef;


void wakeup_idle();
void wakeup_sleep();

// Writes a data[REG_LEN] to register at address
int8_t ltc68041_writeRegGroup(bmsChainHandleTypeDef * hbms, uint16_t address, uint8_t * data);

// Writes a single command frame
int8_t ltc68041_writeCommand(bmsChainHandleTypeDef * hbms, uint16_t cmd);

// Read a register group at address and have the data stored
int8_t ltc68041_readRegGroup(bmsChainHandleTypeDef * hbms, uint16_t address);

// PEC Calculator
uint16_t ltc68041_calculatePEC(uint8_t * data, uint8_t len);
uint16_t pec15_calc(uint8_t len, uint8_t *data);		// CPU CRC Calculator

// User-exposed API
// Initialization function with all the selfTests
uint8_t ltc68041_Initialize(bmsChainHandleTypeDef * hbms);

// Self-tests
uint8_t ltc68041_cvTest(bmsChainHandleTypeDef * hbms);
uint8_t ltc68041_auxTest(bmsChainHandleTypeDef * hbms);
uint8_t ltc68041_statTest(bmsChainHandleTypeDef * hbms);
uint8_t ltc68041_adstatTest(bmsChainHandleTypeDef * hbms);

// Clear register operations
uint8_t ltc68041_clearCell(bmsChainHandleTypeDef * hbms);
uint8_t ltc68041_clearAux(bmsChainHandleTypeDef * hbms);
uint8_t ltc68041_clearStat(bmsChainHandleTypeDef * hbms);

// Start conversion
uint8_t ltc68041_startCVConv(bmsChainHandleTypeDef * hbms);     // Cell voltages
uint8_t ltc68041_startAUXConv(bmsChainHandleTypeDef * hbms);    // GPIO voltages
uint8_t ltc68041_startCVAUXConv(bmsChainHandleTypeDef * hbms);  // GPIO1, GPIO2, and cell voltage measurements

// LTC6804 pass through SPI/I2C API
uint8_t ltc68041_writeCOMM(bmsChainHandleTypeDef * hbms);   // Write to COMM register
uint8_t ltc68041_readOMM(bmsChainHandleTypeDef * hbms);     // Read from COMM register
uint8_t ltc68041_startCOMM(bmsChainHandleTypeDef * hbms);   // Start communication

// Utilities
uint8_t ltc68041_muxTest(bmsChainHandleTypeDef * hbms);     // Checks the multiplexer (DIAGN)
uint8_t ltc68041_owTest(bmsChainHandleTypeDef * hbms);      // Open wire test (ADOW)
uint8_t ltc68041_convStat(bmsChainHandleTypeDef * hbms);    // ADC conversion Status (PLADC)

#endif
