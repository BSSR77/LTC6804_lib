/*
 * Can_Processor.c
 *
 *  Created on: Jan 8, 2017
 *      Author: frank
 */
#include "Can_Processor.h"

extern osMessageQId mainCanRxQHandle;
extern const uint8_t selfNodeID;

inline void Can_Processor(){
static Can_frame_t newFrame;
		xQueueReceive(mainCanRxQHandle, &newFrame, portMAX_DELAY);
		uint32_t canID = newFrame.id;	// canID container
#ifdef __JAMES__
		static uint8_t rxmsg[] = "Got a frame.";
		Serial2_writeBuf(rxmsg);
#endif
		if(canID == p2pOffset || canID == selfNodeID + p2pOffset){
			// Multicast or unicast command received!
			taskENTER_CRITICAL();
			executeCommand(newFrame.Data[0]);
			taskEXIT_CRITICAL();
		} else {
			// XXX 1: Additional application-level message parsing
			// Note: Any application-level messages should be either mutex protected or passed via queue!
		}
}
