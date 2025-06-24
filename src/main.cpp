#include "attiny.h"

uint32_t lastQuery = 0;

BluetoothManager* bluetoothManager;

void vibrationCallback(size_t size, const VibrationInterval_t *intervals);

void setup() {  
  Serial.begin(115200);
  Uart1.begin(UARTBAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  bluetoothManager = new BluetoothManager(&vibrationCallback);

  delay(1000);
  
  sendCmd(BROADCAST_ADDR, CMD_RESTART_BNO);
  
  delay(1000);
  
  for(int i = 0; i < NUM_ATTINYS; i++) {
    startAttiny(i);
  }
  sendCmd(BROADCAST_ADDR, CMD_UPDATE_DATA);
  while (Uart1.available()) Uart1.read();
}

void loop(){
  uint32_t now = millis();

  if (now - lastQuery >= DATA_INTERVAL) {
    lastQuery += DATA_INTERVAL;

    while (Uart1.available()) Uart1.read();
    
    SensorData sensorData[NUM_ATTINYS];

    for (int i = 0; i < NUM_ATTINYS; i++) {
      handleAttiny(i, sensorData[i]);
    }

    for (int i = 0; i < NUM_ATTINYS; i++) {
      if (sensorData[i].valid) {
        sendSensorPacketAsJson(sensorData[i], i);
      } else {
        sendZeroSensorJson(i);
      }
    }
    bluetoothManager->streamIMUFullPacket(sensorData);

    Serial.println("Dauer alle Attinys abzufragen: " + String(millis() - now) + " ms");
  }
}

void vibrationCallback(size_t size, const VibrationInterval_t *intervals)  {

}