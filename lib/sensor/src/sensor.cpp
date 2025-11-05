#include "sensor.h"
#include <HardwareSerial.h>
#include "debug.h"

void sensorBegin(unsigned long baud) {
  Serial2.begin(baud, SERIAL_8N1, RXD2, TXD2);
}

bool readSensorData(SensorPacket *pkt) {
  // Wait for start byte 0xAA with timeout
  unsigned long startTime = millis();
  bool foundStart = false;

  while (millis() - startTime < 100) { // 100 ms timeout
    if (Serial2.available()) {
      uint8_t byte = Serial2.read();
      if (byte == 0xAA) {
        pkt->start = byte;
        foundStart = true;
        break;
      }
    }
    delay(1);
  }

  if (!foundStart) {
    DBG_PRINT("Timeout waiting for start byte\n");
    return false;
  }

  // Read the rest of the packet
  uint8_t *pData = (uint8_t *)pkt;
  int bytesRead = 1; // Already read start byte

  startTime = millis();
  while (bytesRead < sizeof(SensorPacket) && millis() - startTime < 500) {
    if (Serial2.available()) {
      pData[bytesRead] = Serial2.read();
      bytesRead++;
    }
    delay(1);
  }

  if (bytesRead != sizeof(SensorPacket)) {
    DBG_PRINT("Incomplete packet: got %d bytes, expected %d\n", bytesRead, (int)sizeof(SensorPacket));
    return false;
  }

  return true;
}

uint8_t calculateChecksum(SensorPacket *pkt) {
  uint8_t *data = (uint8_t *)pkt;
  uint8_t sum = 0;
  for (int i = 0; i < sizeof(SensorPacket) - 1; i++) {
    sum += data[i];
  }
  return sum;
}
