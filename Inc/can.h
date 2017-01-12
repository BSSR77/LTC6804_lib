/*
 * can.h
 *
 * Created on: Dec 4, 2016
 *     Author: jamesliu
 *       Note: the HAL CAN driver is a complete friggin hack job. No respect. Pisses me off.
 */

#ifndef CAN_H_
#define CAN_H_

#include "main.h"
#include "cmsis_os.h"

#define CAN_BANKS 14

typedef struct
{
  uint32_t id;
  uint8_t dlc;
  uint8_t Data[8];
  uint8_t isExt; //1 or 0
  uint8_t isRemote;
  int filterNum;
}Can_frame_t;

typedef struct
{
  uint32_t id;
  uint32_t mask;
  uint8_t isRemote;
  uint8_t maskRemote;
  uint8_t isExt;
  uint8_t isMasked;
  int filterNum;
}Can_filter_t; //only used for getFilter()

void bxCan_begin(CAN_HandleTypeDef *hcan, osMessageQId *rx, osMessageQId *tx);

int bxCan_addMaskedFilterStd(uint16_t id, uint16_t mask, int isRemote/*-1 = don't care*/);
int bxCan_addMaskedFilterExt(uint32_t id, uint32_t mask, int isRemote/*-1 = don't care*/);
int bxCan_addFilterStd(uint16_t id, uint8_t isRemote);
int bxCan_addFilterExt(uint32_t id, uint8_t isRemote);
int bxCan_getFilter(Can_filter_t *target, int filterNum);
int bxCan_removeFilter(int filterNum);
int bxCan_getFilterNum(uint32_t fmi);

int bxCan_availableForTx();
uint32_t bxCanDoTx(uint8_t fromISR);
int bxCan_sendFrame(Can_frame_t *frame);

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

void bxCan_setTxCallback(void(*pt)());
void bxCan_setRxCallback(void(*pt)());
void bxCan_setErrCallback(void(*pt)(uint32_t));

#endif /* CAN_H_ */
