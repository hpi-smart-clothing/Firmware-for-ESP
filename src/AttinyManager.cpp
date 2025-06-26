#include "AttinyManager.h"
#include <ArduinoJson.h>

AttinyManager::AttinyManager(HardwareSerial &uart) : uart_(uart)
{
  memcpy(attinyAddresses_, attinyAddresses, sizeof(attinyAddresses_));
}

void AttinyManager::begin()
{
  uart_.begin(UARTBAUD, SERIAL_8N1, RX_PIN, TX_PIN);
}

void AttinyManager::sendCmd(uint8_t addr, uint8_t cmd)
{
  while (uart_.available())
    uart_.read();
  uart_.write(addr);
  uart_.write(cmd);
  delay(5);
}

UARTPacketStatus AttinyManager::receiveUARTPacket(uint8_t *addr, uint8_t *buf, uint8_t *len, uint16_t bufsize, uint16_t timeout)
{
  unsigned long startTime = millis();

  while (true)
  {
    if (millis() - startTime > timeout)
    {
      return UART_TIMEOUT_STARTBYTE;
    }
    if (uart_.available() && uart_.read() == START_BYTE)
    {
      break;
    }
  }

  while (uart_.available() < 2)
  {
    if (millis() - startTime > timeout)
      return UART_TIMEOUT_HEADER;
  }
  uint8_t packetLen = uart_.read();
  uint8_t packetAddr = uart_.read();

  if (packetLen > bufsize)
  {
    return UART_BUFFER_OVERFLOW;
  }

  for (int i = 0; i < packetLen; i++)
  {
    unsigned long bstart = millis();
    while (!uart_.available())
    {
      if (millis() - bstart > timeout)
      {
        return UART_TIMEOUT_DATABYTE;
      }
    }
    buf[i] = uart_.read();
  }

  unsigned long endStart = millis();
  while (!uart_.available())
  {
    if (millis() - endStart > timeout)
      return UART_TIMEOUT_ENDBYTE;
  }
  uint8_t end = uart_.read();
  if (end != ENDBYTE)
    return UART_WRONG_ENDBYTE;

  if (addr)
    *addr = packetAddr;
  if (len)
    *len = packetLen;

  return UART_OK;
}

bool AttinyManager::receiveSensorData(uint8_t idx, SensorData &data)
{
  if (receiveUARTPacket(nullptr, data.buffer, &data.dataLen, MAX_PACKET_SIZE) == UART_OK)
  {
    data.valid = true;
    if (data.dataLen < MIN_SENSOR_PACKET_SIZE)
    {
      data.valid = false;
      return false;
    }
    return true;
  }
  else
  {
    data.valid = false;
    data.dataLen = 0;
    return false;
  }
}

bool AttinyManager::startAttiny(uint8_t idx)
{
  sendCmd(attinyAddresses_[idx], CMD_SEND_BNO_STATUS);
  delay(100);
  if (!readBNOStatus(idx))
  {
    return false;
  }
  else
  {
    sendCmd(attinyAddresses_[idx], CMD_UPDATE_DATA);
    return true;
  }
}

void AttinyManager::startAllAttinys()
{
  sendCmd(BROADCAST_ADDR, CMD_RESTART_BNO);
  delay(1000);
  for (uint8_t i = 0; i < NUM_ATTINYS; i++)
  {
    startAttiny(i);
  }
  sendCmd(BROADCAST_ADDR, CMD_UPDATE_DATA);
  while (uart_.available())
    uart_.read();
  delay(50);
}

bool AttinyManager::readBNOStatus(uint8_t idx)
{
  uint8_t dataBuf[MAX_PACKET_SIZE];
  uint8_t dataLen;
  if (receiveUARTPacket(nullptr, dataBuf, &dataLen, sizeof(dataBuf)))
  {
    if (dataLen < 1)
      return false;
    uint8_t status = dataBuf[1];
    return (status == 1);
  }
  return false;
}

void AttinyManager::handleAttiny(uint8_t idx, SensorData &data)
{
  while (uart_.available())
    uart_.read();
  sendCmd(attinyAddresses_[idx], CMD_SEND_THEN_UPDATE);
  receiveSensorData(idx, data);
}
