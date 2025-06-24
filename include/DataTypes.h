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
};

#endif //DATATYPES_H
