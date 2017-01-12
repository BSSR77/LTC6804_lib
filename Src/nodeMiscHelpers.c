/*
 * nodeMiscHelpers.c
 *
 *  Created on: Dec 31, 2016
 *      Author: frank
 */
#include "nodeMiscHelpers.h"

extern uint32_t 	selfStatusWord;
extern osMutexId 	swMtxHandle;
extern osMessageQId mainCanTxQHandle;
extern osMessageQId mainCanRxQHandle;
extern osTimerId 	HBTmrHandle;

/*
 * Command executer for implementing node command responses
 */
void executeCommand(uint8_t cmd){
	switch(cmd){
	// Hard reset
	case NODE_HRESET:
		NVIC_SystemReset();					// CMSIS System reset function
		break;

	// Soft Reset
	case NODE_RESET:
		node_shutdown();					// Soft shutdown
		NVIC_SystemReset();					// CMSIS System reset function
		break;

	// Clean shutdown
	// NOTE: CAN command processor is still active!
	case NODE_SHUTDOWN:
		if((selfState == ACTIVE) || (selfState == INIT)){
			node_shutdown();						// Soft shutdown if node is active
			xTimerStop(HBTmrHandle, portMAX_DELAY);	// Stop the heartbeat timer

			// XXX 4: User must suspend any additional application tasks
			// xTaskSuspend(ApplicationHandle);		// Suspend any active non-CAN tasks
		}
		break;

	// Node start command from shutdown state
	case NODE_START:
		if(selfState == SHUTDOWN){
			setState(INIT);
			// Flush the Rx queue for fresh state on start-up
			xQueueReset(mainCanRxQHandle);
			xQueueReset(mainCanTxQHandle);
			// XXX 2: Flush the application queues!
			// xQueueReset();

			xTimerReset(HBTmrHandle,portMAX_DELAY);	// Start the heartbeat timer

			// XXX 3: User must resume additional application tasks
			// xTaskResume(ApplicationHandle);			// Resume any application tasks
		}
		break;

	// CC Acknowledgement of node addition attempt
	case CC_ACK:
		if(selfState == INIT){
			setState(ACTIVE);
		}
		break;

	// CC Negation of node addition attempt
	case CC_NACK:
		if(selfState == INIT){
			setState(SHUTDOWN);
		}
		break;

	default:
		// Do nothing if the command is invalid
		break;
	}
}

/* CHECKED
 * Thread-safe node state accessor
 */
nodeState inline getSelfState(){
	xSemaphoreTake(swMtxHandle, portMAX_DELAY);
	nodeState ret = (nodeState)(selfStatusWord & 0x07);
	xSemaphoreGive(swMtxHandle);
	return ret;
}

/* CHECKED
 * Thread-safe node state mutator
 */
inline void setSelfState(nodeState newState){
	xSemaphoreTake(swMtxHandle, portMAX_DELAY);
	selfStatusWord &= 0xfffffff8;	// Clear the current status word
	selfStatusWord |= newState;
	xSemaphoreGive(swMtxHandle);
}

/*
 * Soft shutdown routine that will complete any critical cleanups via callback
 * Assembles node SHUTDOWN statusword CAN frame
 * Flushes CanTx queue via broadcast on bxCAN
 */
inline void soft_shutdown(void(*usr_clbk)()){
	// Don't care about locking the statusWord here since we are in Critical Area
	setState(SHUTDOWN);

	// User defined shutdown routine
	usr_clbk();

	// Broadcast node shutdown state to main CAN
	Can_frame_t newFrame;
	newFrame.id = radio_SW;
	newFrame.dlc = CAN_HB_DLC;
	for(int i=0; i<4; i++){
		newFrame.Data[3-i] = (selfStatusWord >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
	}
	bxCan_sendFrame(&newFrame);
	// TODO: Test if bxCan_sendFrame can successfully send the new frame and flush the queue
}

