//
// Created by Tebbe on 23.04.2025.
//

#ifndef VIBRATIONMANAGER_H
#define VIBRATIONMANAGER_H
#include "DataTypes.h"

class VibrationManager
{
public:
    VibrationManager();
    bool submitVibrationPattern(size_t size, const VibrationInterval_t* vibrationIntervals) const;

private:
    QueueHandle_t vibrationQueue;
    TaskHandle_t vibrationTask;

    static void taskTrampoline(void* pvParameters);
    void taskEntryPoint() const;
};


#endif //VIBRATIONMANAGER_H
