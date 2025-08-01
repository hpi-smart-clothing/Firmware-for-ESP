#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define DATA_INTERVAL 200
#define UARTBAUD 38400
#define UART_TIMEOUT 100
#define UART_PACKET_SIZE 43
#define BROADCAST_ADDR 0xFF
#define START_BYTE 0xAA
#define ENDBYTE 0x55
#define MAX_PACKET_SIZE 64
#define TX_PIN 9 // GPIO9 / D9
#define RX_PIN 10 // GPIO10 / D10

constexpr uint8_t attinyAddresses[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
constexpr size_t NUM_ATTINYS = sizeof(attinyAddresses) / sizeof(attinyAddresses[0]);

#define CMD_TEST 0x01
#define CMD_CHIPID 0x02
#define CMD_RESTART_BNO 0x03
#define CMD_CALIBRATION 0x04
#define CMD_UPDATE_THEN_SEND 0x05
#define CMD_SEND_THEN_UPDATE 0x06
#define CMD_UPDATE_DATA 0x07
#define CMD_SEND_TIME 0x08
#define CMD_SEND_BNO_STATUS 0x09

#define MIN_SENSOR_PACKET_SIZE 38

#define MTU_RATE 195

#define MOTOR_LEFT_PIN D8
#define MOTOR_RIGHT_PIN D7
#define MOTOR_LEFT_LEDC_CHANNEL 0
#define MOTOR_RIGHT_LEDC_CHANNEL 0
#define MOTOR_PWM_FREQUENCY 5000

#define BLE_DEVICE_NAME "BackUp"

#endif