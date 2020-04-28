//
// Created by kevinlad on 2020/4/28.
//

#ifndef SRC_HUAXUN_RADAR_HUAXUN_RADAR_SRC_CAN_CANPROCESS_H_
#define SRC_HUAXUN_RADAR_HUAXUN_RADAR_SRC_CAN_CANPROCESS_H_

#include <cstdint>
#include <canlib.h>


int CanInit();
int CanClose();
int CanWrite(uint32_t nSendId, uint8_t* nData, uint8_t nDataLen);
int CanPutData(uint32_t nId, uint64_t nTime, uint8_t* pData);


#endif //SRC_HUAXUN_RADAR_HUAXUN_RADAR_SRC_CAN_CANPROCESS_H_
