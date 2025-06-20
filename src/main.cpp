#include <HardwareSerial.h>
#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <array>
#include <ArduinoJson.h>

#define DATA_INTERVAL 200
#define UARTBAUD 38400
#define NUM_ATTINYS 6
#define UART_TIMEOUT 100
#define UART_PACKET_SIZE 43 // 1 Startbyte + 1 Addr + 40 Data + 1 Endbyte
#define BROADCAST_ADDR 0xFF
#define START_BYTE 0xAA
#define ENDBYTE 0x55
#define MAX_PACKET_SIZE 64

#define CMD_TEST 0x01
#define CMD_CHIPID 0x02
#define CMD_RESTART_BNO 0x03
#define CMD_CALIBRATION 0x04
#define CMD_UPDATE_THEN_SEND 0x05
#define CMD_SEND_THEN_UPDATE 0x06
#define CMD_UPDATE_DATA 0x07
#define CMD_SEND_TIME 0x08
#define CMD_SEND_BNO_STATUS 0x09

uint8_t attinyAddresses[NUM_ATTINYS] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
uint32_t lastQuery = 0;


HardwareSerial Uart1(1);                   // UART1

void sendCmd(uint8_t addr, uint8_t c);
bool receiveUARTPacket(uint8_t* addr, uint8_t* buf, uint8_t* len, uint16_t bufsize, uint16_t timeout = UART_TIMEOUT);
void printRawUARTPacket();
void printSensorUARTPacket();
bool readBNOStatus();
bool receiveSensorData(uint8_t* dataBuf, uint8_t* dataLen, uint16_t maxLen);
void sendSensorPacketAsJson(const uint8_t* dataBuf, uint8_t dataLen, uint8_t sensorIdx);
void checkForZeros(const uint8_t* dataBuf, uint8_t dataLen, uint8_t addr);
void sendZeroSensorJson(uint8_t sensorIdx);

void setup() {  
  Serial.begin(115200);                    // USB-Console
  Uart1.begin(UARTBAUD, SERIAL_8N1, 20, 21); // RX = GPIO20 / D7
  lastQuery = millis();
  StaticJsonDocument<128> doc;
  
  delay(1000);
  sendCmd(BROADCAST_ADDR, CMD_RESTART_BNO); // Testbefehl an alle Attinys
  delay(1000);
  for(int i = 0; i < NUM_ATTINYS; i++) {
    sendCmd(attinyAddresses[i], CMD_SEND_BNO_STATUS); // Chip-ID an alle Attinys
    Serial.println("Chip-ID Anfrage an Attiny 0x" + String(attinyAddresses[i], HEX) + " gesendet.");
    doc["i"] = i;
    doc["m"] = String("S: Init: ") + "0x" + String(attinyAddresses[i], HEX);
    serializeJson(doc, Serial);
    Serial.println();

    if(!readBNOStatus())  
      Serial.println("BNO konnte nicht gestartet werden: 0x" + String(attinyAddresses[i], HEX));
    delay(5);
  }
  sendCmd(BROADCAST_ADDR, CMD_UPDATE_DATA); // Datenabfrage an alle Attinys
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

void sendCmd(uint8_t addr, uint8_t c){
  while (Uart1.available()) Uart1.read();
  Uart1.write(addr);
  Uart1.write(c);
  delay(5);
}

bool receiveUARTPacket(uint8_t* addr, uint8_t* buf, uint8_t* len, uint16_t bufsize, uint16_t timeout) {
  unsigned long startTime = millis();

  // 1. Warte auf Startbyte
  while (true) {
    if (millis() - startTime > timeout) {
      Serial.println("UART Timeout beim Startbyte-Empfang!");
      return false;
    }
    if (Uart1.available() && Uart1.read() == START_BYTE) {      
      break;
    }
  }

  // 2. Länge und Adresse empfangen
  while (Uart1.available() < 2) {
    if (millis() - startTime > timeout) return false;
  }
  uint8_t packetLen = Uart1.read();
  uint8_t packetAddr = Uart1.read();

  if (packetLen > bufsize)  {
    Serial.println("Paketlänge größer als Puffergröße!");
    return false; // Schutz vor Bufferüberlauf
  }
  // 3. Datenbytes lesen
  for (int i = 0; i < packetLen; i++) {
    unsigned long bstart = millis();
    while (!Uart1.available()) {
      if (millis() - bstart > timeout)  {
        Serial.println("UART Timeout beim Datenbyte-Empfang!"); 
        return false;
      }
    }
    buf[i] = Uart1.read();
  }

  // 4. Endbyte prüfen
  unsigned long endStart = millis();
  while (!Uart1.available()) {
    if (millis() - endStart > timeout) return false;
  }
  uint8_t end = Uart1.read();
  if (end != ENDBYTE) return false;

  // 5. Übergabe der Adress- und Längeninfo
  if (addr) *addr = packetAddr;
  if (len)  *len  = packetLen;

  return true;
}

void printRawUARTPacket() {
  uint8_t senderAddr;
  uint8_t dataBuf[MAX_PACKET_SIZE];
  uint8_t dataLen;

  if (receiveUARTPacket(&senderAddr, dataBuf, &dataLen, sizeof(dataBuf))) {
    Serial.printf("Rohdatenpaket von 0x%02X, %d Bytes:\n", senderAddr, dataLen);
    for (int i = 0; i < dataLen; i++) {
      Serial.printf("Byte %2d: 0x%02X\n", i, dataBuf[i]);
    }
  } else {
    Serial.println("Kein vollständiges Paket empfangen (Rohdaten).");
  }
}

void printSensorUARTPacket() {
  uint8_t senderAddr;
  uint8_t dataBuf[MAX_PACKET_SIZE];
  uint8_t dataLen;

  if (receiveUARTPacket(&senderAddr, dataBuf, &dataLen, sizeof(dataBuf))) {
    if (dataLen < 40) {
      Serial.println("Zu kurzes Sensordatenpaket.");
      return;
    }
    int idx = 0;
    int16_t accX  = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t accY  = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t accZ  = dataBuf[idx++] | (dataBuf[idx++] << 8);

    int16_t gyrX  = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t gyrY  = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t gyrZ  = dataBuf[idx++] | (dataBuf[idx++] << 8);

    int16_t magX  = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t magY  = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t magZ  = dataBuf[idx++] | (dataBuf[idx++] << 8);

    int16_t quatW = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t quatX = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t quatY = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t quatZ = dataBuf[idx++] | (dataBuf[idx++] << 8);

    int16_t linAccX = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t linAccY = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t linAccZ = dataBuf[idx++] | (dataBuf[idx++] << 8);

    int16_t gravX = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t gravY = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t gravZ = dataBuf[idx++] | (dataBuf[idx++] << 8);

    uint16_t readingTime = dataBuf[idx++] | (dataBuf[idx++] << 8);

    Serial.printf("Von Attiny 0x%02X:\n", senderAddr);
    Serial.printf("Acc:     %.2f, %.2f, %.2f m/s²\n", accX * 0.00981f, accY * 0.00981f, accZ * 0.00981f);
    Serial.printf("Gyro:    %.2f, %.2f, %.2f °/s\n", gyrX / 16.0f, gyrY / 16.0f, gyrZ / 16.0f);
    Serial.printf("Mag:     %.2f, %.2f, %.2f uT\n", magX / 16.0f, magY / 16.0f, magZ / 16.0f);
    Serial.printf("Quat:    %.4f, %.4f, %.4f, %.4f\n", quatW / 16384.0f, quatX / 16384.0f, quatY / 16384.0f, quatZ / 16384.0f);
    Serial.printf("LinAcc:  %.2f, %.2f, %.2f m/s²\n", linAccX * 0.00981f, linAccY * 0.00981f, linAccZ * 0.00981f);
    Serial.printf("Gravity: %.2f, %.2f, %.2f m/s²\n", gravX * 0.00981f, gravY * 0.00981f, gravZ * 0.00981f);
    Serial.printf("Reading Time: %d ms\n", readingTime);
  } else {
    Serial.println("Kein vollständiges Paket empfangen (Sensorwerte).");
  }
}

bool readBNOStatus() {
    uint8_t dataBuf[MAX_PACKET_SIZE];
    uint8_t dataLen;

    if (receiveUARTPacket(nullptr, dataBuf, &dataLen, sizeof(dataBuf))) {
        if (dataLen < 1) {
            Serial.println("Zu kurzes BNO-Statuspaket.");
            return false;
        }
        uint8_t status = dataBuf[1];
        Serial.printf("BNO-Status: %u\n", status);
        return (status == 1);
    } else {
        Serial.println("Kein gültiges Statuspaket empfangen.");
        return false;
    }
}

bool receiveSensorData(uint8_t* dataBuf, uint8_t* dataLen, uint16_t maxLen) {
  if (receiveUARTPacket(nullptr, dataBuf, dataLen, maxLen)) {
    if (*dataLen < 40) {
      Serial.println("Sensorpaket zu kurz.");
      return false;
    }
    return true;
  } else {
    Serial.println("Kein Sensorpaket empfangen.");
    return false;
  }
}

void sendSensorPacketAsJson(const uint8_t* dataBuf, uint8_t dataLen, uint8_t sensorIdx) {
    if (dataLen < 40) {
        Serial.println("Zu kurzes Sensordatenpaket für JSON.");
        return;
    }

    int idx = 0;
    int16_t accX  = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t accY  = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t accZ  = dataBuf[idx++] | (dataBuf[idx++] << 8);

    int16_t gyrX  = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t gyrY  = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t gyrZ  = dataBuf[idx++] | (dataBuf[idx++] << 8);

    int16_t magX  = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t magY  = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t magZ  = dataBuf[idx++] | (dataBuf[idx++] << 8);

    int16_t quatW = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t quatX = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t quatY = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t quatZ = dataBuf[idx++] | (dataBuf[idx++] << 8);

    int16_t linAccX = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t linAccY = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t linAccZ = dataBuf[idx++] | (dataBuf[idx++] << 8);

    int16_t gravX = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t gravY = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t gravZ = dataBuf[idx++] | (dataBuf[idx++] << 8);

    // JSON bauen (wie in deinem anderen Code)
    StaticJsonDocument<256> doc;
    doc["i"] = sensorIdx; // Sensor-Index, kannst du auch aus Adresse nehmen
    JsonArray data = doc.createNestedArray("m");
    // Reihenfolge und Skalierung wie in deinem bisherigen Code:
    data.add(accX * 0.00981f);
    data.add(accY * 0.00981f);
    data.add(accZ * 0.00981f);

    data.add(linAccX * 0.00981f);
    data.add(linAccY * 0.00981f);
    data.add(linAccZ * 0.00981f);

    data.add(gravX * 0.00981f);
    data.add(gravY * 0.00981f);
    data.add(gravZ * 0.00981f);

    data.add(magX / 16.0f);
    data.add(magY / 16.0f);
    data.add(magZ / 16.0f);

    data.add(gyrX / 16.0f);
    data.add(gyrY / 16.0f);
    data.add(gyrZ / 16.0f);

    data.add(quatW / 16384.0f);
    data.add(quatX / 16384.0f);
    data.add(quatY / 16384.0f);
    data.add(quatZ / 16384.0f);

    serializeJson(doc, Serial); // oder auf einen anderen Stream/File
    Serial.println();
}

void sendZeroSensorJson(uint8_t sensorIdx) {
    StaticJsonDocument<256> doc;
    doc["i"] = sensorIdx;
    JsonArray data = doc.createNestedArray("m");

    // 18 Messwerte (wie in deinem Format): Acc (3), LinAcc (3), Grav (3), Mag (3), Gyro (3), Quat (4)
    for (int j = 0; j < 19; ++j) data.add(0.0);

    serializeJson(doc, Serial);
    Serial.println();
}

void checkForZeros(const uint8_t* dataBuf, uint8_t dataLen, uint8_t addr)  {
  if (dataLen < 26) return; // 18+8 Bytes nötig!

    int idx = 18; // Offset für Quaternionen
    int16_t quatW = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t quatX = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t quatY = dataBuf[idx++] | (dataBuf[idx++] << 8);
    int16_t quatZ = dataBuf[idx++] | (dataBuf[idx++] << 8);

    if (quatW == 0 && quatX == 0 && quatY == 0 && quatZ == 0) {
      Serial.printf("Alle Quaternion-Komponenten sind 0 für Attiny 0x%02X! BNO wird neu gestartet...\n", addr);
      sendCmd(addr, CMD_RESTART_BNO);
      delay(1000);
      if(!readBNOStatus())  
        Serial.println("BNO konnte nicht gestartet werden: 0x" + String(addr, HEX));
      else
        sendCmd(addr, CMD_UPDATE_DATA);
    }
}