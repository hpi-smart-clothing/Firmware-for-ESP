#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <Arduino.h>
#include "DataTypes.h"
#include "config.h"

class DataManager {
public:
    // Daten ausgeben
    void SerialPrintSensorUARTPacket(const SensorData& data, uint8_t senderAddr) const;
    void SerialPrintRawUARTPacket(const uint8_t* dataBuf, uint8_t dataLen, uint8_t senderAddr) const;

    // Daten als JSON an Serial ausgeben
    void sendSensorPacketAsJson(const SensorData& data, uint8_t sensorIdx) const;
    void sendZeroSensorJson(uint8_t sensorIdx) const;

    // Null-Quaternions prüfen (für Fehlerhandling)
    bool checkForZeros(const SensorData& data) const;

    // Für BLE: Rohquaternions extrahieren
    void extractQuatsForBLE(const SensorData sensorData[], uint8_t quatData[][8], size_t numAttinys) const;
};

#endif // DATAMANAGER_H
