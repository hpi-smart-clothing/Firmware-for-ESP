#ifndef ATTINYMANAGER_H
#define ATTINYMANAGER_H

#include <Arduino.h>
#include "config.h"
#include "DataTypes.h"

class AttinyManager {
public:
    AttinyManager(HardwareSerial& uart);

    void begin();
    bool AttinyManager::startAttiny(uint8_t idx);
    bool receiveSensorData(uint8_t idx, SensorData& data);
    void handleAttiny(uint8_t idx, SensorData& data);
    void startAllAttinys();

private:
    HardwareSerial& uart_;
    uint8_t attinyAddresses_[NUM_ATTINYS];

    UARTPacketStatus receiveUARTPacket(uint8_t* addr, uint8_t* buf, uint8_t* len, uint16_t bufsize, uint16_t timeout = UART_TIMEOUT);
    bool readBNOStatus(uint8_t idx);
    void sendCmd(uint8_t addr, uint8_t cmd);

};

enum UARTPacketStatus {
    UART_OK = 0,
    UART_TIMEOUT_STARTBYTE,
    UART_TIMEOUT_HEADER,
    UART_BUFFER_OVERFLOW,
    UART_TIMEOUT_DATABYTE,
    UART_TIMEOUT_ENDBYTE,
    UART_WRONG_ENDBYTE
};

#endif // ATTINYMANAGER_H
