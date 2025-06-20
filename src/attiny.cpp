#include <HardwareSerial.h>
#include <SPI.h>
#include <Arduino.h>
#include "attiny.h"

void sendCmd(uint8_t addr, uint8_t c);
bool receiveUARTPacket(uint8_t* addr, uint8_t* buf, uint8_t* len, uint16_t bufsize, uint16_t timeout = UART_TIMEOUT);
void printRawUARTPacket();
void printSensorUARTPacket();
bool readBNOStatus();
bool receiveSensorData(uint8_t* dataBuf, uint8_t* dataLen, uint16_t maxLen);
void checkForZeros(const uint8_t* dataBuf, uint8_t dataLen, uint8_t addr);

