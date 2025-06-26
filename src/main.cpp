#include "AttinyManager.h"
#include "BluetoothManager.h"
#include "VibrationManager.h"
#include "DataManager.h"

uint32_t lastQuery = 0;
HardwareSerial Uart1(1);

BluetoothManager* bluetoothManager;
VibrationManager* vibrationManager;
AttinyManager* attinyManager;
DataManager* dataManager;

void vibrationCallback(size_t size, const VibrationInterval_t *intervals);

void setup() {  
  Serial.begin(115200);
  
  vibrationManager = new VibrationManager();
  dataManager = new DataManager();
  bluetoothManager = new BluetoothManager(&vibrationCallback);
  attinyManager = new AttinyManager(Uart1);

  attinyManager->begin();
  delay(50);
  attinyManager->startAllAttinys();
  
}

void loop(){
  uint32_t now = millis();

  if (now - lastQuery >= DATA_INTERVAL) {
    lastQuery += DATA_INTERVAL;
    
    SensorData sensorData[NUM_ATTINYS];

    for (int i = 0; i < NUM_ATTINYS; i++) {
      attinyManager->handleAttiny(i, sensorData[i]);
    }

    for (int i = 0; i < NUM_ATTINYS; i++) {
      if (sensorData[i].valid) {
        if(!dataManager->checkForZeros(sensorData[i])) 
          dataManager->sendSensorPacketAsJson(sensorData[i], i);
        else {
          dataManager->sendZeroSensorJson(i);
          attinyManager->startAttiny(i);
        }
      } else {
        dataManager->sendZeroSensorJson(i);
      }
    }
    bluetoothManager->streamIMUFullPacket(sensorData);
    Serial.println("Time for all Attinys: " + String(millis() - now) + " ms");
  }
}

void vibrationCallback(const size_t size, const VibrationInterval_t* intervals)
{
    vibrationManager->submitVibrationPattern(size, intervals);
}
