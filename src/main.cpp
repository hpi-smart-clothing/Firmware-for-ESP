#include <HardwareSerial.h>
#include <SPI.h>
#include <Arduino.h>
#include <array>
#include <ArduinoJson.h>
#include "attiny.h"

#define DATA_INTERVAL 200
#define UARTBAUD 38400
#define NUM_ATTINYS 6
#define UART_TIMEOUT 100
#define UART_PACKET_SIZE 43 // 1 Startbyte + 1 Addr + 40 Data + 1 Endbyte
#define BROADCAST_ADDR 0xFF
#define START_BYTE 0xAA
#define ENDBYTE 0x55
#define MAX_PACKET_SIZE 64
#define TX_PIN 9 // GPIO9 / D9
#define RX_PIN 10 // GPIO10 / D10

#define CMD_TEST 0x01
#define CMD_CHIPID 0x02
#define CMD_RESTART_BNO 0x03
#define CMD_CALIBRATION 0x04
#define CMD_UPDATE_THEN_SEND 0x05
#define CMD_SEND_THEN_UPDATE 0x06
#define CMD_UPDATE_DATA 0x07
#define CMD_SEND_TIME 0x08
#define CMD_SEND_BNO_STATUS 0x09

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