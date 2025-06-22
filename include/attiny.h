#ifndef IMU_H
#define IMU_H
#include <Arduino.h>
#include "config.h"

extern HardwareSerial Uart1;

void sendCmd(uint8_t addr, uint8_t c);
void checkForZeros(const uint8_t* dataBuf, uint8_t dataLen, uint8_t addr);
void printSensorUARTPacket();
bool readBNOStatus();
bool receiveSensorData(uint8_t* dataBuf, uint8_t* dataLen, uint16_t maxLen);
void startAttiny(int idx);
void sendZeroSensorJson(uint8_t sensorIdx);
void sendSensorPacketAsJson(const uint8_t* dataBuf, uint8_t dataLen, uint8_t sensorIdx);

#endif