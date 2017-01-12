/*
 * serial.h
 *
 *  Created on: Nov 28, 2016
 *      Author: jamesliu
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "main.h"
#include <String.h>

#ifndef SERIAL2_BUFFER_SIZE
#define SERIAL2_BUFFER_SIZE 64
#endif

#ifndef SERIAL2_BUFFER_SIZE_TX
#define SERIAL2_BUFFER_SIZE_TX 128
#endif

//this is for writing an existing, assigned buffer:
#define Serial2_writeBuf(str) Serial2_writeBytes((str), sizeof((str))-1)

//this is for writing a string literal:
//#define Serial2_writeStr(str) Serial2_writeStr_Buf=(str); Serial2_writeBytes(Serial2_writeStr_Buf, sizeof((str))-1)

uint8_t Serial2_writeStr_Buf[SERIAL2_BUFFER_SIZE_TX];

uint8_t Serial2_buffer[SERIAL2_BUFFER_SIZE];

void Serial2_begin();
int Serial2_available();
uint8_t Serial2_peek();
uint8_t Serial2_read();
int Serial2_readBytes(uint8_t *buffer, int length);
int Serial2_find(uint8_t data);
int Serial2_findAny(uint8_t *match, int length);
int Serial2_readUntil(uint8_t *buffer, uint8_t data);
int Serial2_readCommand(uint8_t *buffer);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
int Serial2_availableForWrite();
void Serial2_write(uint8_t data);
void Serial2_writeBytes(uint8_t *data, uint16_t length);

#endif
