/*
 * can.c
 *
 *  Created on: Dec 4, 2016
 *      Author: jamesliu
 */

#include "can.h"

/*
 * This is how HAL handles the filter values:
 *
 * 16 bit filters:
 * FR1 = ((0x0000FFFF & FilterMaskIdLow) << 16) | (0x0000FFFF & FilterIdLow);
 * FR2 = ((0x0000FFFF & FilterMaskIdHigh) << 16) | (0x0000FFFF & FilterIdHigh);
 *
 * 32 bit filters:
 * FR1 = ((0x0000FFFF & FilterIdHigh) << 16) | (0x0000FFFF & FilterIdLow);
 * FR2 = ((0x0000FFFF & FilterMaskIdHigh) << 16) | (0x0000FFFF & FilterMaskIdLow);
 *
 * A mask is also always more significant than its corresponding ID.
 */

/*
 * This is the queue-buffered version of STM32 bxCAN driver for use with FreeRTOS.
 * Users must take care to use only the following exposed functions, or behavior of bxCAN may be unstable!
 *
 * CAN SETUP:
 * bxCan_begin(A,B,C);
 * A - HAL bxCAN Handle
 * B - bxCAN Receive FreeRTOS Queue
 * C - bxCAN Transmit FreeRTOS Queue
 *
 * FRAME TRANSMISSION:
 *
 * Method 1:
 * Prepare a Can_frame_t type with the desired data
 * Pass frame into bxCan_sendFrame and wait for return
 * Users should check the return value of bxCan_sendFrame (should be 0) to ensure that the frame is properly sent
 *
 * Method 2:
 * Users can directly enqueue items to the TxQ (FreeRTOS Queue)
 * Using this method, it is the user's responsibility to MANUALLY call bxCanDoTx(0) to actually attempt frame transmission
 * bxCanDoTx will return the CAN error state code
 *
 * FRAME RECEPTION:
 *
 */

/*
 * This is the special no-buffer version of the Can driver, for use with rtos.
 * Because there's no buffer, there are no read functions,
 * so can rx data must be sent to a queue or something in HAL_CAN_RxCpltCallback.
 */

static CAN_FilterConfTypeDef Can_filters[CAN_BANKS];
static uint8_t Can_filterCapacity[CAN_BANKS];
static uint8_t Can_filterUsage[CAN_BANKS];

static uint8_t not_in_use = 1;

static CanTxMsgTypeDef txFrameBuf;		// Transmission frame handle from HAL
static CanRxMsgTypeDef rxFrameBuf;		// Receive frame handle from HAL

static CAN_HandleTypeDef *hcan_handle;	// CAN Handle object passed in from HAL
static osMessageQId *rxQ;				// Receive message queue
static osMessageQId *txQ;				// Transmit message queue

static void (*bxCan_Txcb)(); //don't touch. user callbacks.
static void (*bxCan_Rxcb)();
static void (*bxCan_Ercb)(uint32_t);

static void empty(){}		// Empty function

/* CHECKED
 * CAN library initialization
 * Set all data object pointers and being the interrupt-based receive service
 */
void bxCan_begin(CAN_HandleTypeDef *hcan, osMessageQId *rx, osMessageQId *tx){
	bxCan_Txcb = empty;
	bxCan_Rxcb = empty;
	bxCan_Ercb = empty;
	hcan_handle = hcan;
	rxQ = rx;
	txQ = tx;
	hcan_handle->pRxMsg = &rxFrameBuf;
	hcan_handle->pTxMsg = &txFrameBuf;
	HAL_CAN_Receive_IT(hcan_handle, 0);
}

int bxCan_addMaskedFilterStd(uint16_t id, uint16_t mask, int isRemote){ //2 slots per bank
	uint8_t rtr = (isRemote==0)?0:1;
	uint8_t rtrm = (isRemote<0)?0:1; //0 = don't care, 1 = must match
	for(int i=0; i<CAN_BANKS; i++){ //add to existing available bank
		uint8_t *usage = &Can_filterUsage[i];
		uint8_t *capacity = &Can_filterCapacity[i];
		if(*capacity==2 && *usage>0 && *usage<0x03 &&
				Can_filters[i].FilterScale==0){ //if in use but unfilled
			if((*usage&0x02) == 0){ //if slot 2 unused, so 3 must be in use
				Can_filters[i].FilterIdLow = id<<5 | rtr<<4 | 0<<3; //id|rtr|ide
				Can_filters[i].FilterMaskIdLow = mask<<5 | rtrm << 4 | 1<<3;
				*usage |= 0x02;
				HAL_CAN_ConfigFilter(hcan_handle, &Can_filters[i]);
				return 4*i + 2; //"usage" index = xxxx0123
			}else{ //slot 2 in use, so slot 3 must be empty
				Can_filters[i].FilterIdHigh = id<<5 | rtr<<4 | 0<<3; //id|rtr|ide
				Can_filters[i].FilterMaskIdHigh = mask<<5 | rtrm << 4 | 1<<3;
				*usage |= 0x01;
				HAL_CAN_ConfigFilter(hcan_handle, &Can_filters[i]);
				return 4*i + 3; //"usage" index = xxxx0123
			}
		}
	}
	for(int i=0; i<CAN_BANKS; i++){ //open up a fresh bank
		if(Can_filterUsage[i]==0){ //find unused bank
			Can_filterUsage[i] |= 0x02; //use highest slot first
			Can_filterCapacity[i] = 2;
			Can_filters[i].FilterIdLow = id<<5 | rtr<<4 | 0<<3; //id|rtr|ide
			Can_filters[i].FilterMaskIdLow = mask<<5 | rtrm << 4 | 1<<3;
			Can_filters[i].FilterIdHigh = 0; //0 should not be a valid addr
			Can_filters[i].FilterMaskIdHigh = ~0;
			Can_filters[i].FilterActivation = ENABLE;
			Can_filters[i].FilterFIFOAssignment = 0;
			Can_filters[i].BankNumber = i+1; //This one isn't even used us HAL...
			Can_filters[i].FilterNumber = i;
			Can_filters[i].FilterMode = 0; //0 for mask, 1 for list
			Can_filters[i].FilterScale = 0; //0 for 16, 1 for 32
			HAL_CAN_ConfigFilter(hcan_handle, &Can_filters[i]);
			return 4*i + 2; //"usage" index = xxxx0123
		}
	}
	return -1;
}

int bxCan_addMaskedFilterExt(uint32_t id, uint32_t mask, int isRemote){ //1 slot per bank
	uint8_t rtr = (isRemote==0)?0:1;
	uint8_t rtrm = (isRemote<0)?0:1; //0 = don't care, 1 = must match
	for(int i=0; i<CAN_BANKS; i++){ //open up a fresh bank
		if(Can_filterUsage[i]==0){ //find unused bank
			Can_filterUsage[i] |= 0x01; //use highest slot first
			Can_filterCapacity[i] = 1;
			Can_filters[i].FilterIdHigh = id>>13; //id[28:13]
			Can_filters[i].FilterIdLow = ((id<<3)&0xffff) | 1<<2 | rtr<<1; //id[12:0]|ide|rtr|0
			Can_filters[i].FilterMaskIdHigh = mask>>13; //mask[28:13]
			Can_filters[i].FilterMaskIdLow = ((mask<<3)&0xffff) | 1<<2 | rtrm<<1; //mask[12:0]|1|rtrm|0
			Can_filters[i].FilterActivation = ENABLE;
			Can_filters[i].FilterFIFOAssignment = 0;
			Can_filters[i].BankNumber = i+1; //This one isn't even used us HAL...
			Can_filters[i].FilterNumber = i;
			Can_filters[i].FilterMode = 0; //0 for mask, 1 for list
			Can_filters[i].FilterScale = 1; //0 for 16, 1 for 32
			HAL_CAN_ConfigFilter(hcan_handle, &Can_filters[i]);
			return 4*i + 3; //"usage" index = xxxx0123
		}
	}
	return -1;
}

int bxCan_addFilterStd(uint16_t id, uint8_t isRemote){ //4 slots per bank
	isRemote = (isRemote==0)?0:1;
	for(int i=0; i<CAN_BANKS; i++){ //add to existing available bank
		uint8_t *usage = &Can_filterUsage[i];
		uint8_t *capacity = &Can_filterCapacity[i];
		if(*capacity==4 && *usage>0 && *usage<0x0f){ //if in use but unfilled
			uint8_t openSlot;
			if((*usage&0x08)==0){
				openSlot = 0;
				Can_filters[i].FilterMaskIdLow = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			}else if((*usage&0x04)==0){
				openSlot = 1;
				Can_filters[i].FilterIdLow = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			}else if((*usage&0x02)==0){
				openSlot = 2;
				Can_filters[i].FilterMaskIdHigh = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			}else{
				openSlot = 3;
				Can_filters[i].FilterIdHigh = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			}
			*usage |= 0x08>>openSlot;
			HAL_CAN_ConfigFilter(hcan_handle, &Can_filters[i]);
			return 4*i + openSlot;
		}
	}
	for(int i=0; i<CAN_BANKS; i++){ //open up a fresh bank
		if(Can_filterUsage[i]==0){ //find unused bank
			Can_filterUsage[i] |= 0x08; //use highest slot first
			Can_filterCapacity[i] = 4;
			Can_filters[i].FilterMaskIdLow = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			Can_filters[i].FilterIdLow = 0; //0 should not be a valid addr
			Can_filters[i].FilterMaskIdHigh = 0;
			Can_filters[i].FilterIdHigh = 0;
			Can_filters[i].FilterActivation = ENABLE;
			Can_filters[i].FilterFIFOAssignment = 0;
			Can_filters[i].BankNumber = i+1; //This one isn't even used us HAL...
			Can_filters[i].FilterNumber = i;
			Can_filters[i].FilterMode = 1; //0 for mask, 1 for list
			Can_filters[i].FilterScale = 0; //0 for 16, 1 for 32
			HAL_CAN_ConfigFilter(hcan_handle, &Can_filters[i]);
			return 4*i + 0; //"usage" index = xxxx0123
		}
	}
	return -1;
}

int bxCan_addFilterExt(uint32_t id, uint8_t isRemote){ //2 slots per bank
	isRemote = (isRemote==0)?0:1;
	for(int i=0; i<CAN_BANKS; i++){ //add to existing available bank
		uint8_t *usage = &Can_filterUsage[i];
		uint8_t *capacity = &Can_filterCapacity[i];
		if(*capacity==2 && *usage>0 && *usage<0x03 &&
				Can_filters[i].FilterScale==1){ //if in use but unfilled
			if((*usage&0x02) == 0){ //if slot 2 unused, so 3 must be in use
				Can_filters[i].FilterIdHigh = id>>13; //id[28:13]
				Can_filters[i].FilterIdLow = ((id<<3)&0xffff) | 1<<2 | isRemote << 1; //id[12:0]|ide|rtr|0
				*usage |= 0x02;
				HAL_CAN_ConfigFilter(hcan_handle, &Can_filters[i]);
				return 4*i + 2; //"usage" index = xxxx0123
			}else{ //slot 2 in use, so slot 3 must be empty
				Can_filters[i].FilterMaskIdHigh = id>>13; //id[28:13]
				Can_filters[i].FilterMaskIdLow = ((id<<3)&0xffff) | 1<<2 | isRemote << 1; //id[12:0]|ide|rtr|0
				*usage |= 0x01;
				HAL_CAN_ConfigFilter(hcan_handle, &Can_filters[i]);
				return 4*i + 3; //"usage" index = xxxx0123
			}
		}
	}
	for(int i=0; i<CAN_BANKS; i++){ //open up a fresh bank
		if(Can_filterUsage[i]==0){ //find unused bank
			Can_filterUsage[i] |= 0x02; //use highest slot first
			Can_filterCapacity[i] = 2;
			Can_filters[i].FilterIdHigh = id>>13; //id[28:13]
			Can_filters[i].FilterIdLow = ((id<<3)&0xffff) | 1<<2 | isRemote << 1; //id[12:0]|ide|rtr|0
			Can_filters[i].FilterMaskIdHigh = 0; //0 should not be a valid addr
			Can_filters[i].FilterMaskIdLow = 0;
			Can_filters[i].FilterActivation = ENABLE;
			Can_filters[i].FilterFIFOAssignment = 0;
			Can_filters[i].BankNumber = i+1; //This one isn't even used us HAL...
			Can_filters[i].FilterNumber = i;
			Can_filters[i].FilterMode = 1; //0 for mask, 1 for list
			Can_filters[i].FilterScale = 1; //0 for 16, 1 for 32
			HAL_CAN_ConfigFilter(hcan_handle, &Can_filters[i]);
			return 4*i + 2; //"usage" index = xxxx0123
		}
	}
	return -1;
}

int bxCan_getFilter(Can_filter_t *target, int filterNum){
	CAN_FilterConfTypeDef *bank = &Can_filters[filterNum/4];
	if(bank->FilterActivation == ENABLE && (Can_filterUsage[filterNum/4] & (1<<(3-(filterNum%4))))){ //optimizing this by hand is just not worth it
		target->filterNum = filterNum;
		filterNum %= 4;
		if(bank->FilterScale == 0x00){ //16 bit
			target->isExt = 0;
			if(bank->FilterMode == 0x00){ //mask
				target->isMasked = 1;
				if(filterNum == 2){ //slot 2
					target->id = bank->FilterIdLow>>5;
					target->mask = bank->FilterMaskIdLow>>5;
					target->isRemote = (bank->FilterIdLow>>4)&1;
					target->maskRemote = (bank->FilterMaskIdLow>>4)&1;
				}else{ //slot 3
					target->id = bank->FilterIdHigh>>5;
					target->mask = bank->FilterMaskIdHigh>>5;
					target->isRemote = (bank->FilterIdHigh>>4)&1;
					target->maskRemote = (bank->FilterMaskIdHigh>>4)&1;
				}
			}else{ //list
				target->isMasked = 0;
				switch(filterNum){
				case 0:
					target->id = bank->FilterMaskIdLow>>5;
					target->isRemote = (bank->FilterMaskIdLow>>4)&1;
					break;
				case 1:
					target->id = bank->FilterIdLow>>5;
					target->isRemote = (bank->FilterIdLow>>4)&1;
					break;
				case 2:
					target->id = bank->FilterMaskIdHigh>>5;
					target->isRemote = (bank->FilterMaskIdHigh>>4)&1;
					break;
				case 3:
					target->id = bank->FilterIdHigh>>5;
					target->isRemote = (bank->FilterIdHigh>>4)&1;
					break;
				}
			}
		}else{ //32 bit
			target->isExt = 1;
			if(bank->FilterMode == 0x00){ //mask
				target->isMasked = 1;
				target->id = (bank->FilterIdLow>>3) | (bank->FilterIdHigh<<13);
				target->mask = (bank->FilterMaskIdLow>>3) | (bank->FilterMaskIdHigh<<13);
				target->isRemote = (bank->FilterIdLow>>1)&1;
				target->maskRemote = (bank->FilterMaskIdLow>>1)&1;
			}else{ //list
				target->isMasked = 0;
				if(filterNum == 2){ //slot 2
					target->id = (bank->FilterIdLow>>3) | (bank->FilterIdHigh<<13);
					target->isRemote = (bank->FilterIdLow>>1)&1;
				}else{ //slot 3
					target->id = (bank->FilterMaskIdLow>>3) | (bank->FilterMaskIdHigh<<13);
					target->isRemote = (bank->FilterMaskIdLow>>1)&1;
				}
			}
		}
		return 0;
	}
	return -1;
}

int bxCan_removeFilter(int filterNum){
	int bankNum = filterNum/4;
	CAN_FilterConfTypeDef *bank = &Can_filters[bankNum];
	filterNum %= 4;
	if(bank->FilterActivation == ENABLE){
		Can_filterUsage[bankNum] &= ~(1<<(3-filterNum));
		if(Can_filterUsage[bankNum] == 0x00){
			bank->FilterActivation = DISABLE;
			Can_filterCapacity[bankNum] = 0;
		}
		return 0;
	}
	return -1;
}

int bxCan_getFilterNum(uint32_t fmi){
	int result = 0; //fmi is 0 indexed
	for(int i=0; i<CAN_BANKS; i++){
		if(Can_filterCapacity[i] > fmi){ //if target is in ith bank
			result += 4-Can_filterCapacity[i]+fmi;
			return result;
		}
		result += 4;
		fmi -= Can_filterCapacity[i];
	}
	return -1;
}

/* CHECKED
 * Check if bxCAN is ready for transmission
 */
int Can_availableForTx(){
	if((hcan_handle->State == HAL_CAN_STATE_READY) || (hcan_handle->State == HAL_CAN_STATE_BUSY_RX)){
		return 1;
	}
	return 0;
}

/* CHECKED
 * May be called from an ISR or non-ISR context
 * If there are pending frames in the Tx Q
 * Assembles data frame for the bxCAN peripheral
 * Execute the interrupt-based CAN transmit function
 *
 * Return: HAL bxCAN Error code (refer to stm32l4xx_hal_can.h)
 */
uint32_t bxCanDoTx(uint8_t fromISR){
	if(Can_availableForTx() && not_in_use && (fromISR ? \
			uxQueueMessagesWaitingFromISR(*txQ) : \
			uxQueueMessagesWaiting(*txQ))){
		not_in_use = 0;		// bxCAN is now in use
		static Can_frame_t toSend;
		// Use the appropriate calls from different contexts to dequeue object
		if(fromISR){
		   xQueueReceiveFromISR(*txQ, &toSend, NULL);
		} else {
		   xQueueReceive(*txQ, &toSend, portMAX_DELAY);
		}

		// Assemble the new transmission frame
		if(toSend.isExt){
			txFrameBuf.IDE = CAN_ID_EXT;
			txFrameBuf.ExtId = toSend.id;
		} else {
			txFrameBuf.IDE = CAN_ID_STD;
			txFrameBuf.StdId = toSend.id;
		}

		if(toSend.isRemote){
			txFrameBuf.RTR = CAN_RTR_REMOTE;
		} else {
			txFrameBuf.RTR = CAN_RTR_DATA;
		}

		txFrameBuf.DLC = toSend.dlc;
		for(uint8_t i = 0; i<toSend.dlc; i++){
			txFrameBuf.Data[i] = toSend.Data[i];
		}
		HAL_CAN_Transmit_IT(hcan_handle);
		return HAL_CAN_ERROR_NONE;	// Successful transmission
	} else {
		// When CAN is not available for Tx -> return the error code in case there are errors
		return HAL_CAN_GetError(hcan_handle);	// HAL Error encountered
	}
}

/* CHECKED
 * bxCAN Tx function to be called from non-ISR context
 * Function to enqueue a frame into the TxQ and trigger sending
 *
 * Return: -1 -  enqueue failed
 * 			0 -  success!
 * 			>0 - CAN error
 */
int bxCan_sendFrame(Can_frame_t *frame){
	UBaseType_t fail = xQueueSend(*txQ, frame, 0);
	if(fail == pdPASS){
		// Only try to transmit if message successfully placed onto Tx Q
		fail = bxCanDoTx(0);		// Call bxCAN Tx function from non-ISR context
	}
	return fail;
}

/* CHECKED
 * HAL bxCAN transmission complete callback
 * Will call the interrupt CAN Tx function again
 * Ultimately try to clear the Tx Q
 */
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	bxCan_Txcb();		// Any user-defined bxCAN txCallback goes here
	not_in_use = 1;		// Transmission is complete, bxCAN not in use
	bxCanDoTx(1);		// Execute the transmission function again; try to clear the Tx Q
}

/* CHEKCED
 * HAL bxCAN Rx callback function
 * Immediately activates interrupt-based receive routine again to ensure no frames are dropped
 * Extracts frame information and enqueues into Rx Q
 * Executes user-defined callback
 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	static Can_frame_t received;
	received.isRemote = (rxFrameBuf.RTR) ? 1 : 0;
	received.isExt    = (rxFrameBuf.IDE) ? 1 : 0;
	if(received.isExt) {
		received.id = rxFrameBuf.ExtId;
	} else {
		received.id = rxFrameBuf.StdId;
	}
	received.filterNum = bxCan_getFilterNum(rxFrameBuf.FMI);
	received.dlc = rxFrameBuf.DLC;
	for(uint8_t i = 0; i < 8 ;i++){
		received.Data[i] = rxFrameBuf.Data[i];
	}
	// Enqueue the received frame to the Rx Q; no higher priority task running after ISR
	xQueueSendFromISR(*rxQ, &received, NULL);

	bxCan_Rxcb();	// User-defined receive callback
	HAL_CAN_Receive_IT(hcan_handle, 0);
}

/* CHECKED
 * If CAN runs into any error, will execute user defined error handler
 * Passes the CAN Error code as argument
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	bxCan_Ercb(HAL_CAN_GetError(hcan));
}

void bxCan_setTxCallback(void(*pt)()){
	bxCan_Txcb = pt;
}

void bxCan_setRxCallback(void(*pt)()){
	bxCan_Rxcb = pt;
}

void bxCan_setErrCallback(void(*pt)(uint32_t)){
	bxCan_Ercb = pt;
}
