#ifndef IMU_H
#define IMU_H
#include <Arduino.h>
#include "config.h"

void sendCmd(uint8_t addr, uint8_t c);
void printSensorUARTPacket();
bool readBNOStatus();
bool receiveSensorData(uint8_t* dataBuf, uint8_t* dataLen, uint16_t maxLen);

#endif