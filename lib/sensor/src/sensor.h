#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

// === UART Configuration ===
// Use Serial2 for communication with STM32
#define RXD2 16  // Connect to STM32 TX (PA2)
#define TXD2 17  // Connect to STM32 RX (PA3)

// === Data structure matching STM32 ===
typedef struct __attribute__((packed)) {
  uint8_t start;      // 0xAA
  uint8_t temperature;
  uint8_t humidity;
  float ldrPercent;
  float waterPercent;
  uint8_t checksum;
} SensorPacket;

// API
void sensorBegin(unsigned long baud = 115200);
bool readSensorData(SensorPacket *pkt);
uint8_t calculateChecksum(SensorPacket *pkt);

#endif // SENSOR_H
