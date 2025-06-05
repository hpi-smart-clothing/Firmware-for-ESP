#include <Wire.h>
#include <Arduino.h>

#define ATTINY_I2C_ADDR 0x08  // I²C-Adresse des ATtiny85
#define MAX_BYTES_FOR_I2C 16  // maximale Anzahl an bytes die per I2C in einer Übertragung gesendet werden können
#define MAX_RESPONSE_TIME 25  // maximale Wartezeit für die Antwort des ATtiny in Millisekunden

String getSerialInput();
void handleInput(String cmd);
void handleCharInput(char input);
void sendCommandToAttiny(uint8_t cmd);
String requestDataFromAttiny(uint8_t maxLength = MAX_BYTES_FOR_I2C);
String safeRequestFromAttiny(uint8_t addr = ATTINY_I2C_ADDR, uint8_t len = MAX_BYTES_FOR_I2C);
uint8_t request1ByteFromAttiny();

void setup() {
  Serial.begin(115200);
  Wire.begin();  // SDA/SCL default
  Serial.println("ESP32-C3 bereit. Tippe 1, um vom ATtiny zu lesen.");
}

void loop() {
  String temp = getSerialInput();
  if(temp.length() > 0) {
    Serial.print("Input: ");
    Serial.println(temp);
    handleInput(temp);
  }
}

String getSerialInput() {
  String inputStr = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {  // Eingabe abgeschlossen
      return inputStr;
    } else {
      inputStr += c;
    }
  }
  return "";
}

void handleInput(String cmd) {
  cmd.trim();  // falls z. B. "\r\n" mitkommt
  int cmdInt = cmd.toInt();
  if (cmdInt > 0 && cmdInt <= 255) {
    sendCommandToAttiny((uint8_t)cmdInt);
    delay(30);
    String response = safeRequestFromAttiny();
    Serial.print("Antwort vom ATtiny: ");
    Serial.println(response);
  } else {
    Serial.println("Ungültiger Befehl.");
  }
}

String requestDataFromAttiny(uint8_t maxLength) {
  String result = "";
  char c = '\0';

  for (int i = 0; i < 32; i++) {
    Wire.requestFrom(ATTINY_I2C_ADDR, 1);
    if (Wire.available()) {
      c = Wire.read();
      if (c == '\n') break;
      result += c;
    }
    delay(1);  // Mini-Wartezeit für ATtiny
  }

  return result;
}

String safeRequestFromAttiny(uint8_t addr, uint8_t len) {
  String result = "";
  char c = '\0';

  uint8_t bytesReceived = Wire.requestFrom(addr, len);

  if (!bytesReceived) {
    return "NO RESPONSE";
  }

  while (Wire.available()) {
    c = Wire.read();
    if (c == '\n') break;
    result += c;
    delay(1);  // Mini-Wartezeit für ATtiny
    if (result.length() >= len) {
      break;  // Maximale Länge erreicht
    }
  }

  return result;
}

uint8_t request1ByteFromAttiny()  {
  Wire.requestFrom(ATTINY_I2C_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  } else {
    return 0;
  }
}

void sendCommandToAttiny(uint8_t cmd) {
  Wire.beginTransmission(ATTINY_I2C_ADDR);
  Wire.write(cmd);  // z. B. 1–10
  uint8_t status = Wire.endTransmission();
  if (status != 0) {
    Serial.print("I2C-Fehler bei EndTransmission: ");
    Serial.println(status);
  }
}