#include "DataManager.h"
#include <ArduinoJson.h>

void DataManager::SerialPrintRawUARTPacket(const uint8_t* dataBuf, uint8_t dataLen, uint8_t senderAddr) const {
    Serial.printf("Rohdatenpaket von 0x%02X, %d Bytes:\n", senderAddr, dataLen);
    for (int i = 0; i < dataLen; i++) {
        Serial.printf("Byte %2d: 0x%02X\n", i, dataBuf[i]);
    }
}

void DataManager::SerialPrintSensorUARTPacket(const SensorData& data, uint8_t senderAddr) const {
    if (data.dataLen < 40) {
        Serial.println("Zu kurzes Sensordatenpaket.");
        return;
    }
    int idx = 0;
    int16_t accX  = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t accY  = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t accZ  = data.buffer[idx++] | (data.buffer[idx++] << 8);

    int16_t gyrX  = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t gyrY  = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t gyrZ  = data.buffer[idx++] | (data.buffer[idx++] << 8);

    int16_t magX  = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t magY  = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t magZ  = data.buffer[idx++] | (data.buffer[idx++] << 8);

    int16_t quatW = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t quatX = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t quatY = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t quatZ = data.buffer[idx++] | (data.buffer[idx++] << 8);

    int16_t linAccX = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t linAccY = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t linAccZ = data.buffer[idx++] | (data.buffer[idx++] << 8);

    int16_t gravX = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t gravY = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t gravZ = data.buffer[idx++] | (data.buffer[idx++] << 8);

    uint16_t readingTime = data.buffer[idx++] | (data.buffer[idx++] << 8);

    Serial.printf("Von Attiny 0x%02X:\n", senderAddr);
    Serial.printf("Acc:     %.2f, %.2f, %.2f m/s²\n", accX * 0.00981f, accY * 0.00981f, accZ * 0.00981f);
    Serial.printf("Gyro:    %.2f, %.2f, %.2f °/s\n", gyrX / 16.0f, gyrY / 16.0f, gyrZ / 16.0f);
    Serial.printf("Mag:     %.2f, %.2f, %.2f uT\n", magX / 16.0f, magY / 16.0f, magZ / 16.0f);
    Serial.printf("Quat:    %.4f, %.4f, %.4f, %.4f\n", quatW / 16384.0f, quatX / 16384.0f, quatY / 16384.0f, quatZ / 16384.0f);
    Serial.printf("LinAcc:  %.2f, %.2f, %.2f m/s²\n", linAccX * 0.00981f, linAccY * 0.00981f, linAccZ * 0.00981f);
    Serial.printf("Gravity: %.2f, %.2f, %.2f m/s²\n", gravX * 0.00981f, gravY * 0.00981f, gravZ * 0.00981f);
    Serial.printf("Reading Time: %d ms\n", readingTime);
}

void DataManager::sendSensorPacketAsJson(const SensorData& data, uint8_t sensorIdx) const {
    if (data.dataLen < 40) {
        Serial.println("Zu kurzes Sensordatenpaket für JSON.");
        return;
    }

    int idx = 0;
    int16_t accX  = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t accY  = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t accZ  = data.buffer[idx++] | (data.buffer[idx++] << 8);

    int16_t gyrX  = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t gyrY  = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t gyrZ  = data.buffer[idx++] | (data.buffer[idx++] << 8);

    int16_t magX  = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t magY  = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t magZ  = data.buffer[idx++] | (data.buffer[idx++] << 8);

    int16_t quatW = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t quatX = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t quatY = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t quatZ = data.buffer[idx++] | (data.buffer[idx++] << 8);

    int16_t linAccX = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t linAccY = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t linAccZ = data.buffer[idx++] | (data.buffer[idx++] << 8);

    int16_t gravX = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t gravY = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t gravZ = data.buffer[idx++] | (data.buffer[idx++] << 8);

    StaticJsonDocument<256> doc;
    doc["i"] = sensorIdx;
    JsonArray arr = doc.createNestedArray("m");
    arr.add(accX * 0.00981f);
    arr.add(accY * 0.00981f);
    arr.add(accZ * 0.00981f);

    arr.add(linAccX * 0.00981f);
    arr.add(linAccY * 0.00981f);
    arr.add(linAccZ * 0.00981f);

    arr.add(gravX * 0.00981f);
    arr.add(gravY * 0.00981f);
    arr.add(gravZ * 0.00981f);

    arr.add(magX / 16.0f);
    arr.add(magY / 16.0f);
    arr.add(magZ / 16.0f);

    arr.add(gyrX / 16.0f);
    arr.add(gyrY / 16.0f);
    arr.add(gyrZ / 16.0f);

    arr.add(quatW / 16384.0f);
    arr.add(quatX / 16384.0f);
    arr.add(quatY / 16384.0f);
    arr.add(quatZ / 16384.0f);

    serializeJson(doc, Serial);
    Serial.println();
}

void DataManager::sendZeroSensorJson(uint8_t sensorIdx) const {
    StaticJsonDocument<256> doc;
    doc["i"] = sensorIdx;
    JsonArray data = doc.createNestedArray("m");
    for (int j = 0; j < 19; ++j) data.add(0.0);
    serializeJson(doc, Serial);
    Serial.println();
}

bool DataManager::checkForZeros(const SensorData& data) const {
    if (data.dataLen < 26) return false;
    int idx = 18;
    int16_t quatW = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t quatX = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t quatY = data.buffer[idx++] | (data.buffer[idx++] << 8);
    int16_t quatZ = data.buffer[idx++] | (data.buffer[idx++] << 8);
    return (quatW == 0 && quatX == 0 && quatY == 0 && quatZ == 0);
}

void DataManager::extractQuatsForBLE(const SensorData sensorData[], uint8_t quatData[][8], size_t numAttinys) const {
    for (size_t i = 0; i < numAttinys; ++i) {
        sensorData[i].getQuaternionsRaw(quatData[i]);
    }
}
