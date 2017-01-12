/*
 * nodeMiscHelpers.h
 *
 *  Created on: Dec 31, 2016
 *      Author: frank
 *
 *  Misc. functions that groups specific set of actions in a friendlier way
 */

#ifndef NODEMISCHELPERS_H_
#define NODEMISCHELPERS_H_

#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#include "can.h"
#include "serial.h"
#include "../../CAN_ID.h"

// Microsecond delay
// Multiply by 20 for O2 and O3
// Multiply by 16 for O0, O1, and Os
#ifdef DEBUG
#define delayUs(US) 	_delayUS_ASM(US * 16)
#else
#define delayUs(US) 	_delayUS_ASM(US * 20)
#endif

#define _delayUS_ASM(X) \
	asm volatile (	"MOV R0,#" #X  "\n\t"\
			"1: \n\t"\
			"SUB R0, #1\n\t"\
			"CMP R0, #0\n\t"\
			"BNE 1b \n\t"\
		      );\

#define node_shutdown()		soft_shutdown(NULL)			// shutdown wrapper

// Following macro expansions are NOT THREAD SAFE!!!
#define selfState 		(nodeState)(selfStatusWord & 0x07)		// Get the state of the node
#define setState(x)		selfStatusWord &= 0xfffffff8; \
						selfStatusWord |= x;


void executeCommand(uint8_t cmd);
void setSelfState(nodeState newState);
nodeState getSelfState();
void soft_shutdown(void(*usr_clbk)());

#endif /* NODEMISCHELPERS_H_ */
