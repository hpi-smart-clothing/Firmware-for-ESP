#include <HardwareSerial.h>

#define DATA_INTERVAL 1000
#define UARTBAUD 57600
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
uint16_t getChar();
String readSerialLine();
void checkForCommand(String input);
void readSensorPacket();
bool receiveUARTPacket(uint8_t* addr, uint8_t* buf, uint8_t* len, uint16_t bufsize, uint16_t timeout = UART_TIMEOUT);
void readUART(uint8_t cmd);
void printRawUARTPacket();
void printSensorUARTPacket();

void setup() {
  Serial.begin(115200);                    // USB-Console
  Uart1.begin(UARTBAUD, SERIAL_8N1, 20, 21); // RX = GPIO20 / D7
  lastQuery = millis();
  delay(1000);
  for(int i = 0; i < NUM_ATTINYS; i++) {
    sendCmd(attinyAddresses[i], CMD_UPDATE_DATA);
    sendCmd(attinyAddresses[i], CMD_TEST);
    Serial.println("Update Data for Attiny 0x" + String(attinyAddresses[i], HEX));
    delay(5);
  }
  while (Uart1.available()) Uart1.read();
  delay(1000);
}

void loop(){
  uint32_t now = millis();
  if (now - lastQuery >= DATA_INTERVAL) {

    lastQuery += DATA_INTERVAL;
    while (Uart1.available()) Uart1.read();
    for (int i = 0; i < NUM_ATTINYS; i++) {
      sendCmd(attinyAddresses[i], CMD_UPDATE_THEN_SEND);
      Serial.println("Anfrage an Attiny 0x" + String(attinyAddresses[i], HEX) + " gesendet.");
      readUART(CMD_UPDATE_THEN_SEND);
      delay(20);
    }
  }
}

void sendCmd(uint8_t addr, uint8_t c){
  while (Uart1.available()) Uart1.read();
  Uart1.write(addr);
  Uart1.write(c);
  delay(50);
}

void readUART(uint8_t cmd) {
  switch(cmd) {
  case CMD_UPDATE_THEN_SEND:
  case CMD_SEND_THEN_UPDATE:
    printSensorUARTPacket();
    break;

  case CMD_UPDATE_DATA:
  case CMD_RESTART_BNO:
    break;
    
  default:
    printRawUARTPacket();
    break;
  }
}


uint16_t getChar() {
  if (Serial.available()) {
    uint16_t c = Serial.read();
    if (!(c == '\r' || c == '\n')){
      return c; // erstes nutzbares Zeichen
    } 
  }
  return -1;                                // nichts Neues
}

String getTinyLine() {
  Uart1.setTimeout(100);                  // Timeout für die Zeile
  static String buf;
  while (Uart1.available()) {
    char ch = Uart1.read();
    if (ch == '\n') {                       // Zeile fertig
      String out = buf;
      buf = "";
      return out;
    }
    if (ch != '\r') buf += ch;              // CR wegfiltern
  }
  return String();                          // noch unvollständig
}

String readSerialLine() {
  static String input = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;   // CR ignorieren (nur für Windows-Kompatibilität)
    if (c == '\n') {           // ENTER gedrückt: Zeile fertig!
      String line = input;
      input = "";              // Buffer zurücksetzen
      return line;             // Gibt die komplette Zeile zurück
    }
    input += c;                // Zeichen an Buffer anhängen
  }
  return "";                   // Noch keine Zeile fertig
}

void checkForCommand(String input)  {
  if(input.length() != 3) {
    Serial.println("No Command detected");
    return;
  }
  String addrStr = input.substring(0, 2);
  uint8_t addr = (uint8_t) strtoul(addrStr.c_str(), NULL, 16);
  char cmd = input.charAt(2);

  Serial.print("Addr: 0x");
  Serial.print(addr, HEX);
  Serial.print(", Command: ");
  Serial.println(cmd);
  sendCmd(addr, cmd);
}

void readSensorPacket(uint8_t addr) {
  // 1. Auf Startbyte synchronisieren
  unsigned long startWait = millis();

  while (true) {
    // Timeout abfragen:
    if (millis() - startWait > UART_TIMEOUT) {
      Serial.println("UART Timeout beim Startbyte-Empfang!");
      return;
    }
    if (Uart1.available() && Uart1.read() == START_BYTE) break;
  }

  uint8_t paket[UART_PACKET_SIZE - 1];

  // Paket byteweise einlesen
  for (int i = 0; i < UART_PACKET_SIZE - 1; i++) {
    unsigned long start = millis();
    while (!Uart1.available()) {
      // Timeout nach 100 ms
      if (millis() - start > 100) {
        Serial.println("UART Timeout beim Paketempfang!");
        return;
      }
    }
    paket[i] = Uart1.read();
  }

  int idx = 0;
  // 3. Adresse lesen
  uint8_t senderAddr = paket[idx++];
  // Acc
  int16_t accX  = paket[idx++] | (paket[idx++] << 8);
  int16_t accY  = paket[idx++] | (paket[idx++] << 8);
  int16_t accZ  = paket[idx++] | (paket[idx++] << 8);

  // Gyro
  int16_t gyrX  = paket[idx++] | (paket[idx++] << 8);
  int16_t gyrY  = paket[idx++] | (paket[idx++] << 8);
  int16_t gyrZ  = paket[idx++] | (paket[idx++] << 8);

  // Mag
  int16_t magX  = paket[idx++] | (paket[idx++] << 8);
  int16_t magY  = paket[idx++] | (paket[idx++] << 8);
  int16_t magZ  = paket[idx++] | (paket[idx++] << 8);

  // Quat
  int16_t quatW = paket[idx++] | (paket[idx++] << 8);
  int16_t quatX = paket[idx++] | (paket[idx++] << 8);
  int16_t quatY = paket[idx++] | (paket[idx++] << 8);
  int16_t quatZ = paket[idx++] | (paket[idx++] << 8);

  // Linear Acceleration
  int16_t linAccX = paket[idx++] | (paket[idx++] << 8);
  int16_t linAccY = paket[idx++] | (paket[idx++] << 8);
  int16_t linAccZ = paket[idx++] | (paket[idx++] << 8);

  // Gravity
  int16_t gravX = paket[idx++] | (paket[idx++] << 8);
  int16_t gravY = paket[idx++] | (paket[idx++] << 8);
  int16_t gravZ = paket[idx++] | (paket[idx++] << 8);

  // Reading Time
  uint16_t readingTime = paket[idx++] | (paket[idx++] << 8);
  
  // 5. Endbyte prüfen
  uint8_t endByte = paket[idx++];
  if (endByte != ENDBYTE) {
    Serial.println("Paketende stimmt nicht! Paket wird verworfen.");
  }

  // 6. Ausgabe (inkl. Absender)
  Serial.printf("Von Attiny 0x%02X:\n", senderAddr);
  Serial.printf("Acc:     %.2f, %.2f, %.2f m/s²\n", accX * 0.00981f, accY * 0.00981f, accZ * 0.00981f);
  Serial.printf("Gyro:    %.2f, %.2f, %.2f °/s\n", gyrX / 16.0f, gyrY / 16.0f, gyrZ / 16.0f);
  Serial.printf("Mag:     %.2f, %.2f, %.2f uT\n", magX / 16.0f, magY / 16.0f, magZ / 16.0f);
  Serial.printf("Quat:    %.4f, %.4f, %.4f, %.4f\n", quatW / 16384.0f, quatX / 16384.0f, quatY / 16384.0f, quatZ / 16384.0f);
  Serial.printf("LinAcc:  %.2f, %.2f, %.2f m/s²\n", linAccX * 0.00981f, linAccY * 0.00981f, linAccZ * 0.00981f);
  Serial.printf("Gravity: %.2f, %.2f, %.2f m/s²\n", gravX * 0.00981f, gravY * 0.00981f, gravZ * 0.00981f);
  Serial.printf("Reading Time: %d ms\n", readingTime);

  if(quatW == 0 && quatX == 0 && quatY == 0 && quatZ == 0 &&
     accX == 0 && accY == 0 && accZ == 0 &&
     gyrX == 0 && gyrY == 0 && gyrZ == 0 &&
     magX == 0 && magY == 0 && magZ == 0 &&
     linAccX == 0 && linAccY == 0 && linAccZ == 0 &&
     gravX == 0 && gravY == 0 && gravZ == 0) {
    Serial.println("only zeros received, restarting BNO.");
    sendCmd(addr, CMD_RESTART_BNO);
    delay(1000);
    sendCmd(addr, CMD_UPDATE_DATA);
    delay(10);
    sendCmd(addr, CMD_UPDATE_DATA);
  }
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