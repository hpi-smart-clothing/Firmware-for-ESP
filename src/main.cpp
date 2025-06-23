#include "attiny.h"

uint32_t lastQuery = 0;

void setup() {  
  Serial.begin(115200);
  Uart1.begin(UARTBAUD, SERIAL_8N1, RX_PIN, TX_PIN);
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
    
    for (int i = 0; i < NUM_ATTINYS; i++) {
      sendCmd(attinyAddresses[i], CMD_SEND_THEN_UPDATE);
      Serial.println("Anfrage an Attiny 0x" + String(attinyAddresses[i], HEX) + " gesendet.");
      uint8_t buffer[MAX_PACKET_SIZE];
      uint8_t len = 0;

      if (receiveSensorData(buffer, &len, sizeof(buffer))) {
        checkForZeros(buffer, len, attinyAddresses[i]);
        sendSensorPacketAsJson(buffer, len, i);
        Serial.println();
      } else {
        Serial.println("Fehler beim Empfangen der Sensordaten von Attiny 0x" + String(attinyAddresses[i], HEX));
        sendZeroSensorJson(i);
      }
    }
    Serial.println("Dauer alle Attinys abzufragen: " + String(millis() - now) + " ms");
  }
}