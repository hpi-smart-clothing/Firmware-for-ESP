#include <Wire.h>

#define ATTINY_I2C_ADDR 0x08  // I²C-Adresse des ATtiny85
#define MAX_BYTES_FOR_I2C 16  // maximale Anzahl an bytes die per I2C in einer Übertragung gesendet werden können

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
  if(cmd.length() == 1) {
    handleCharInput(cmd.charAt(0));
  }
}

String requestDataFromAttiny(uint8_t maxLength = MAX_BYTES_FOR_I2C) {
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

void handleCharInput(char input)  {
  if (input >= '1' && input <= '10') {
    uint8_t cmd = input - '0';
    sendCommandToAttiny(cmd);
    delay(10);  // dem ATtiny etwas Zeit geben
    String response = requestDataFromAttiny();
    Serial.print("Antwort vom ATtiny: ");
    Serial.println(response);
  } else {
    Serial.println("Unbekannter Befehl.");
  }
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
  Wire.endTransmission();
}