#include <Arduino.h>
#include "config.h"
#include "VibrationManager.h"


VibrationManager::VibrationManager()
{
    ledcSetup(MOTOR_LEFT_LEDC_CHANNEL, MOTOR_PWM_FREQUENCY, 8);
    ledcAttachPin(MOTOR_LEFT_PIN, MOTOR_LEFT_LEDC_CHANNEL);
    ledcWrite(MOTOR_LEFT_LEDC_CHANNEL, 0);

    ledcSetup(MOTOR_RIGHT_LEDC_CHANNEL, MOTOR_PWM_FREQUENCY, 8);
    ledcAttachPin(MOTOR_RIGHT_PIN, MOTOR_RIGHT_LEDC_CHANNEL);
    ledcWrite(MOTOR_RIGHT_LEDC_CHANNEL, 0);

    vibrationQueue = xQueueCreate(16, sizeof(VibrationInterval_t));
    xTaskCreate(taskTrampoline, "Vibration", 2048, this, tskIDLE_PRIORITY + 1, &vibrationTask);
}

bool VibrationManager::submitVibrationPattern(const size_t size, const VibrationInterval_t* vibrationIntervals) const
{
    for (int i = 0; i < size; i++)
    {
        //Try to add all the items to the queue. Skip once it's full.
        if (xQueueSend(vibrationQueue, &vibrationIntervals[i], 0) != pdTRUE)
        {
            return false;
        }
    }
    return true;
}

void VibrationManager::taskTrampoline(void* pvParameters)
{
    const auto instance = static_cast<VibrationManager*>(pvParameters);
    instance->taskEntryPoint();
}

void VibrationManager::taskEntryPoint() const
{
    VibrationInterval_t receivedInterval;
    while (true)
    {
        //Ensure motors are turned off when the queue is empty.
        if (xQueuePeek(vibrationQueue, &receivedInterval, 0) != pdTRUE)
        {
            ledcWrite(MOTOR_LEFT_LEDC_CHANNEL, 0);
            ledcWrite(MOTOR_RIGHT_LEDC_CHANNEL, 0);
        }

        //Wait for arrival of new commands. Put the task in waiting state to avoid consuming CPU time.
        if (xQueueReceive(vibrationQueue, &receivedInterval, portMAX_DELAY) == pdFALSE) continue;

        ledcWrite(MOTOR_LEFT_LEDC_CHANNEL, receivedInterval.leftIntensity);
        ledcWrite(MOTOR_RIGHT_LEDC_CHANNEL, receivedInterval.rightIntensity);

        //Put the task in waiting state again for the provided duration.
        //The accuracy is subject to scheduling behavior and CONFIG_FREERTOS_HZ, which defaults to 1000 Hz / 1ms on Arduino platform.
        vTaskDelay(pdMS_TO_TICKS(receivedInterval.duration));
    }
}
