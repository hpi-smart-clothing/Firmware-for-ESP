#include "DataManager.h"
#include <ArduinoJson.h>

void DataManager::SerialPrintRawUARTPacket(const uint8_t* dataBuf, uint8_t dataLen, uint8_t senderAddr) const {
    Serial.printf("Raw Data from 0x%02X, %d Bytes:\n", senderAddr, dataLen);
    for (int i = 0; i < dataLen; i++) {
        Serial.printf("Byte %2d: 0x%02X\n", i, dataBuf[i]);
    }
}

void DataManager::SerialPrintSensorUARTPacket(const SensorData& data, uint8_t senderAddr) const {
    if (data.dataLen < 40) {
        Serial.println("Sensor packet to short.");
        return;
    }
    uint8_t accRaw[6], gyroRaw[6], magRaw[6], quatRaw[8], linAccRaw[6], gravRaw[6];

    data.getAccelerationRaw(accRaw);
    data.getGyroRaw(gyroRaw);
    data.getMagRaw(magRaw);
    data.getQuaternionsRaw(quatRaw);
    data.getLinearAccelerationRaw(linAccRaw);
    data.getGravityRaw(gravRaw);

    auto get16 = [](const uint8_t* arr, int idx) { return int16_t(arr[idx] | (arr[idx+1] << 8)); };

    Serial.printf("From attiny 0x%02X:\n", senderAddr);
    Serial.printf("Acc:     %.2f, %.2f, %.2f m/s²\n", get16(accRaw,0) * 0.00981f, get16(accRaw,2) * 0.00981f, get16(accRaw,4) * 0.00981f);
    Serial.printf("Gyro:    %.2f, %.2f, %.2f °/s\n", get16(gyroRaw,0) / 16.0f, get16(gyroRaw,2) / 16.0f, get16(gyroRaw,4) / 16.0f);
    Serial.printf("Mag:     %.2f, %.2f, %.2f uT\n", get16(magRaw,0) / 16.0f, get16(magRaw,2) / 16.0f, get16(magRaw,4) / 16.0f);
    Serial.printf("Quat:    %.4f, %.4f, %.4f, %.4f\n", get16(quatRaw,0) / 16384.0f, get16(quatRaw,2) / 16384.0f, get16(quatRaw,4) / 16384.0f, get16(quatRaw,6) / 16384.0f);
    Serial.printf("LinAcc:  %.2f, %.2f, %.2f m/s²\n", get16(linAccRaw,0) * 0.00981f, get16(linAccRaw,2) * 0.00981f, get16(linAccRaw,4) * 0.00981f);
    Serial.printf("Gravity: %.2f, %.2f, %.2f m/s²\n", get16(gravRaw,0) * 0.00981f, get16(gravRaw,2) * 0.00981f, get16(gravRaw,4) * 0.00981f);
}


void DataManager::sendSensorPacketAsJson(const SensorData& data, uint8_t sensorIdx) const {
    if (data.dataLen < 40) {
        Serial.println("Sensor packet to short.");
        return;
    }
    uint8_t accRaw[6], gyroRaw[6], magRaw[6], quatRaw[8], linAccRaw[6], gravRaw[6];
    data.getAccelerationRaw(accRaw);
    data.getGyroRaw(gyroRaw);
    data.getMagRaw(magRaw);
    data.getQuaternionsRaw(quatRaw);
    data.getLinearAccelerationRaw(linAccRaw);
    data.getGravityRaw(gravRaw);

    auto get16 = [](const uint8_t* arr, int idx) { return int16_t(arr[idx] | (arr[idx+1] << 8)); };

    StaticJsonDocument<256> doc;
    doc["i"] = sensorIdx;
    JsonArray arr = doc.createNestedArray("m");
    arr.add(get16(accRaw,0) * 0.00981f);
    arr.add(get16(accRaw,2) * 0.00981f);
    arr.add(get16(accRaw,4) * 0.00981f);

    arr.add(get16(linAccRaw,0) * 0.00981f);
    arr.add(get16(linAccRaw,2) * 0.00981f);
    arr.add(get16(linAccRaw,4) * 0.00981f);

    arr.add(get16(gravRaw,0) * 0.00981f);
    arr.add(get16(gravRaw,2) * 0.00981f);
    arr.add(get16(gravRaw,4) * 0.00981f);

    arr.add(get16(magRaw,0) / 16.0f);
    arr.add(get16(magRaw,2) / 16.0f);
    arr.add(get16(magRaw,4) / 16.0f);

    arr.add(get16(gyroRaw,0) / 16.0f);
    arr.add(get16(gyroRaw,2) / 16.0f);
    arr.add(get16(gyroRaw,4) / 16.0f);

    arr.add(get16(quatRaw,0) / 16384.0f);
    arr.add(get16(quatRaw,2) / 16384.0f);
    arr.add(get16(quatRaw,4) / 16384.0f);
    arr.add(get16(quatRaw,6) / 16384.0f);

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
    uint8_t quatRaw[8];
    data.getQuaternionsRaw(quatRaw);
    auto get16 = [](const uint8_t* arr, int idx) { return int16_t(arr[idx] | (arr[idx+1] << 8)); };
    return (get16(quatRaw,0) == 0 && get16(quatRaw,2) == 0 && get16(quatRaw,4) == 0 && get16(quatRaw,6) == 0);
}

void DataManager::extractQuatsForBLE(const SensorData sensorData[], uint8_t quatData[][8], size_t numAttinys) const {
    for (size_t i = 0; i < numAttinys; ++i) {
        sensorData[i].getQuaternionsRaw(quatData[i]);
    }
}
