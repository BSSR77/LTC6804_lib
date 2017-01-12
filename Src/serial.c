/*
 * serial.c
 *
 *  Created on: Nov 28, 2016
 *      Author: jamesliu
 */

#include "serial.h"

extern UART_HandleTypeDef huart2;

static uint8_t *Serial2_tail = Serial2_buffer;
static uint8_t *Serial2_max = Serial2_buffer + SERIAL2_BUFFER_SIZE; //points just outside the bounds
static uint8_t Serial2_Ovf = 0;

void Serial2_begin(){
	HAL_UART_Receive_DMA(&huart2, Serial2_buffer, SERIAL2_BUFFER_SIZE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(Serial2_Ovf < 3) Serial2_Ovf++;
}

static uint8_t *Serial2_getHead(){ //Volatile! Avoid use as much as possible!
	return Serial2_buffer + SERIAL2_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1,LL_DMA_CHANNEL_6);
}

int Serial2_available(){
	uint8_t *head = Serial2_getHead();
	if(Serial2_Ovf==0){
		return head - Serial2_tail;
	}else if((Serial2_Ovf==1) && (head <= Serial2_tail)){
		return SERIAL2_BUFFER_SIZE - (Serial2_tail - head);
	}else{
		Serial2_tail = head;
		Serial2_Ovf = 1;
		return SERIAL2_BUFFER_SIZE;
	}
}

uint8_t Serial2_peek(){
	if(Serial2_available()){
		return *Serial2_tail;
	}else{
		return 0; //null is appropriate return value for nothing buffered
	}
}

uint8_t Serial2_read(){
	if(Serial2_available()){
		uint8_t data = *Serial2_tail;
		Serial2_tail++; //1 byte version of dequeue()
		if(Serial2_tail >= Serial2_max){
			Serial2_tail = Serial2_buffer;
			Serial2_Ovf--;
		}
		return data;
	}else{
		return 0; //null is appropriate return value for nothing buffered
	}
}

int Serial2_readBytes(uint8_t *buffer, int length){
//	BUFFER OVERFLOW WARNING!!! DO NOT REQUEST MORE THAN YOU CAN TAKE!
	if(Serial2_available()>=length){
		if(Serial2_tail+length >= Serial2_max){
			int half = Serial2_max - Serial2_tail;
			memcpy(buffer, Serial2_tail, half);
			memcpy(buffer+half, Serial2_buffer, length-half);
			Serial2_tail = Serial2_buffer + length - half;
			Serial2_Ovf--;
		}else{
			memcpy(buffer, Serial2_tail, length);
			Serial2_tail += length;
		}
		return 0;
	}
	return -1;
}

int Serial2_find(uint8_t data){
	//different from Arduino: this returns index of char of interest!
	for(int i=0; i<Serial2_available(); i++){
		if(*((Serial2_tail+i >= Serial2_max)?
				Serial2_tail+i-SERIAL2_BUFFER_SIZE:
				Serial2_tail+i) ==data) return i;
	}
	return -1;
}

int Serial2_findAny(uint8_t *match, int length){
	for(int i=0; i<Serial2_available(); i++){
		uint8_t input = *((Serial2_tail+i >= Serial2_max)?
				Serial2_tail+i-SERIAL2_BUFFER_SIZE:
				Serial2_tail+i);
		for(int j=0; j<length; j++){
			if(input == *(match+j)) return i;
		}
	}
	return -1;
}

int Serial2_readUntil(uint8_t *buffer, uint8_t data){
	if(Serial2_available()){
		int found = Serial2_find(data);
		if(found > -1){
			Serial2_readBytes(buffer, found);
			return 0;
		}
	}
	return -1;
}

int Serial2_readCommand(uint8_t *buffer){ //returns length of command
	while(Serial2_peek() == 0x0A || Serial2_peek() == 0x0D){ //NL(LF) or CR, respectively
		Serial2_read(); //clear leading line breaks
	}
	uint8_t delimiters[2] = {0x0A, 0x0D};
	int nextDelim = Serial2_findAny(delimiters, 2);
	if(nextDelim==-1){
		return -1;
	}else{
		Serial2_readBytes(buffer, nextDelim);
		return nextDelim;
	}
}

int Serial2_availableForWrite(){
	HAL_UART_StateTypeDef state = HAL_UART_GetState(&huart2);
	if(state == HAL_UART_STATE_BUSY_TX || state == HAL_UART_STATE_BUSY_TX_RX){
		return 0;
	}else{
		return 1;
	}
}

/*
 * Below this point be all the write functionality
 */

static uint8_t Serial2_charToWrite;
static uint8_t Serial2_Buffer_tx[SERIAL2_BUFFER_SIZE_TX];
static uint8_t *Serial2_tail_tx = Serial2_Buffer_tx;
static uint8_t *Serial2_head_tx = Serial2_Buffer_tx;
static uint8_t *Serial2_max_tx = Serial2_Buffer_tx + SERIAL2_BUFFER_SIZE_TX;
static uint8_t Serial2_ovf_tx = 0;
static uint16_t currentWrite = 0; //length of ongoing dma transaction
static uint8_t txWillTrigger = 0;

static int Serial2_available_tx(){
	if(Serial2_ovf_tx==0){
		return Serial2_head_tx - Serial2_tail_tx;
	}else if((Serial2_ovf_tx==1) && (Serial2_head_tx <= Serial2_tail_tx)){
		return SERIAL2_BUFFER_SIZE_TX - (Serial2_tail_tx - Serial2_head_tx);
	}else{
		Serial2_tail_tx = Serial2_head_tx;
		Serial2_ovf_tx = 1;
		return SERIAL2_BUFFER_SIZE_TX;
	}
}

static void Serial2_dequeue_tx(int length){
	Serial2_tail_tx+=length;
	if(Serial2_tail_tx >= Serial2_max_tx){
		Serial2_tail_tx -= SERIAL2_BUFFER_SIZE_TX;
		Serial2_ovf_tx--;
	}
}

static void doTx(uint8_t fromISR){
	static int txavail;
	txavail = Serial2_available_tx();
	if(txavail){
		if(Serial2_tail_tx + txavail > Serial2_max_tx){
			currentWrite = Serial2_max_tx - Serial2_tail_tx;
		}else{
			currentWrite = txavail;
		}
		HAL_UART_Transmit_DMA(&huart2, Serial2_tail_tx, currentWrite);
		Serial2_dequeue_tx(currentWrite);
		txWillTrigger = 0;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(txWillTrigger == 0) doTx(1);
}

//IF YOU ARE USING RTOS, PLEASE USE MUTEXES, OR WRAP THE BELOW IN CRITICAL SECTIONS.
void Serial2_writeBytes(uint8_t *data, uint16_t length){
	txWillTrigger = 1;
	if(Serial2_head_tx + length >= Serial2_max_tx){
		uint16_t half = Serial2_max_tx-Serial2_head_tx;
		memcpy(Serial2_head_tx, data, half);
		memcpy(Serial2_Buffer_tx, data+half, length-half);
		Serial2_head_tx = Serial2_head_tx + length - SERIAL2_BUFFER_SIZE_TX;
		Serial2_ovf_tx++;
	}else{
		memcpy(Serial2_head_tx, data, length);
		Serial2_head_tx += length;
	}
	if(Serial2_availableForWrite()){
		doTx(0);
	}else{
		txWillTrigger = 0;
	}
}

void Serial2_write(uint8_t data){
	Serial2_charToWrite = data;
	Serial2_writeBytes(&Serial2_charToWrite, 1);
}
