#ifndef ATTINY_H
#define ATTINY_H
#include <Arduino.h>
#include "config.h"
#include "DataTypes.h"
#include "BluetoothManager.h"

extern HardwareSerial Uart1;

void sendCmd(uint8_t addr, uint8_t c);
void checkForZeros(const SensorData& data, uint8_t addr);
void printSensorUARTPacket();
void startAttiny(int idx);
void sendZeroSensorJson(uint8_t sensorIdx);
void sendSensorPacketAsJson(const SensorData& data, uint8_t sensorIdx);
void handleAttiny(uint8_t sensorIdx, SensorData& data);
void sendAllQuaternionsBLE(const SensorData sensorData[], BluetoothManager& bleManager);

#endif