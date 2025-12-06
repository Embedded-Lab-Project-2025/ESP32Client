#include <Arduino.h>

#define DEBUG 1
#define SENSOR_READ_INTERVAL 5000

#include "debug.h"
#include "sensor.h"
#include "secret.h"
#include "wifi_mqtt.h"
#include "servo.h"

// Task handles
TaskHandle_t sensorTaskHandle;
TaskHandle_t mqttTaskHandle;
TaskHandle_t servoTaskHandle;

// Flag for servo trigger
volatile bool servoFlag = false;

// Servo task function
void servoTask(void *pvParameters) {
  servoFlag = false;
  while (1) {
    if (servoFlag) {
      servoTrigger(servoFlag);
      servoFlag = false;
    }
    delay(10); // Small delay to yield
  }
}

// Sensor task function
void sensorTask(void *pvParameters) {
  while (1) {
    DBG_PRINT("--- Requesting Sensor Data ---\n");
    
    // Send 'R' command to STM32 to wake and read sensors
    Serial2.write('R');
    Serial2.flush();
    
    // Read sensor data
    SensorPacket packet;
    if (readSensorData(&packet)) {
      // Verify checksum
      uint8_t calculatedChecksum = calculateChecksum(&packet);
      
      if (packet.checksum == calculatedChecksum) {
        // Display results
        DBG_PRINT("Data received successfully\n");
        DBG_PRINT("=== Sensor Data ===\n");
        
        if (packet.temperature == 0xFF && packet.humidity == 0xFF) {
          DBG_PRINT("DHT11 Error\n");
        } else {
          DBG_PRINT("Temperature: %d°C\n", packet.temperature);
          DBG_PRINT("Humidity: %d%%\n", packet.humidity);
        }
        
        DBG_PRINT("Light Level: %.1f%%\n", packet.ldrPercent);
        DBG_PRINT("Water Level: %.1f mm\n", packet.waterPercent * 0.8);
        
        // Raw data for debugging
        DBG_PRINT("Raw: ");
        uint8_t *pData = (uint8_t *)&packet;
        for (int i = 0; i < sizeof(packet); i++) {
          DBG_PRINT("%02X ", pData[i]);
        }
        DBG_PRINT("\n");
        // Publish CSV: temperature,humidity,ldrPercent,waterLevel
        if (!publishSensorCSV(&packet)) {
          
          DBG_PRINT("MQTT publish failed or not connected\n");
        }
      } else {
        DBG_PRINT("Checksum error!\n");
        DBG_PRINT("Expected: 0x%02X, Got: 0x%02X\n", 
                      calculatedChecksum, packet.checksum);
      }
    } else {
      DBG_PRINT("Failed to read data - timeout or invalid packet\n");
    }
    
    // Clear any remaining data on Serial2
    while (Serial2.available()) {
      Serial2.read();
    }
    
    DBG_PRINT("\n");
    vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL));
  }
}

// MQTT task function
void mqttTask(void *pvParameters) {
  while (1) {
    mqttLoop();
    vTaskDelay(10); // Small delay to yield
  }
}

void setup() {
  Serial.begin(115200);
  delay(10);
  DBG_PRINT("\n\n=== ESP32 UART Master Starting ===\n");

  
  // Initialize UART2 for STM32 communication
  sensorBegin(115200);

  // Setup WiFi and MQTT (non-blocking with timeout)
  if (!wifiMqttBegin(String(WIFI_SSID), String(WIFI_PASSWORD), 15000)) {
    DBG_PRINT("WiFi not available, continuing without MQTT\n");
  }

  // Initialize servo
  servoBegin();
  DBG_PRINT("\n\n=== ESP32 UART Master Ready ===\n");
  DBG_PRINT("Connected to STM32 via UART\n");
  DBG_PRINT("Requesting sensor data every %d seconds...\n\n", SENSOR_READ_INTERVAL / 1000);
  DBG_PRINT("RX: GPIO%d, TX: GPIO%d\n", RXD2, TXD2);

  // Create tasks
  xTaskCreate(sensorTask, "SensorTask", 4096, NULL, 1, &sensorTaskHandle);
  xTaskCreate(mqttTask, "MQTTTask", 4096, NULL, 1, &mqttTaskHandle);
  xTaskCreate(servoTask, "ServoTask", 2048, NULL, 1, &servoTaskHandle);
}

void loop() {
  // Main loop can be empty or handle other tasks
  vTaskDelay(1000); // Yield to other tasks
}
