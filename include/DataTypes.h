//
// Created by Tebbe on 23.04.2025.
//

#ifndef DATATYPES_H
#define DATATYPES_H

#include <Arduino.h>
#include "config.h"

typedef struct VibrationInterval
{
    uint16_t duration;
    uint8_t leftIntensity;
    uint8_t rightIntensity;
} VibrationInterval_t;

struct SensorData {
    uint8_t buffer[MAX_PACKET_SIZE];
    uint8_t dataLen;
    bool valid;

    // Acceleration (X, Y, Z) - 6 Bytes ab Offset 0
    void getAccelerationRaw(uint8_t dest[6]) const {
        if (valid && dataLen >= 6) {
            memcpy(dest, &buffer[0], 6);
        } else {
            memset(dest, 0, 6);
        }
    }

    void getGyroRaw(uint8_t dest[6]) const {
        if (valid && dataLen >= 12) {
            memcpy(dest, &buffer[6], 6);
        } else {
            memset(dest, 0, 6);
        }
    }

    void getMagRaw(uint8_t dest[6]) const {
        if (valid && dataLen >= 18) {
            memcpy(dest, &buffer[12], 6);
        } else {
            memset(dest, 0, 6);
        }
    }

    void getQuaternionsRaw(uint8_t dest[8]) const {
        if (valid && dataLen >= 26) {
            memcpy(dest, &buffer[18], 8);
        } else {
            memset(dest, 0, 8);
        }
    }

    void getLinearAccelerationRaw(uint8_t dest[6]) const {
        if (valid && dataLen >= 32) {
            memcpy(dest, &buffer[26], 6);
        } else {
            memset(dest, 0, 6);
        }
    }

    void getGravityRaw(uint8_t dest[6]) const {
        if (valid && dataLen >= 38) {
            memcpy(dest, &buffer[32], 6);
        } else {
            memset(dest, 0, 6);
        }
    }
};

#endif //DATATYPES_H
