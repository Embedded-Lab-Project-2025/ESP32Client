#include <Arduino.h>
#include <HardwareSerial.h>
#include "secret.h"
#include <WiFi.h>
#include <PubSubClient.h>

#if DEBUG
#define DBG_PRINT(...)    Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#endif

// === UART Configuration ===
// Use Serial2 for communication with STM32
// ESP32 GPIO mapping:
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

SensorPacket packet;

// --- MQTT / WiFi ---
const char* mqtt_server = "ubunsin.krissada.com";
const uint16_t mqtt_port = 1883;
const char* mqtt_user = "board"; // requested username
const char* mqtt_topic = "sensor/data";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// === Function prototypes ===
bool readSensorData();
uint8_t calculateChecksum(SensorPacket *pkt);
void setupWiFi();
bool mqttReconnect();
void publishSensorCSV();

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize UART2 for STM32 communication
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Setup WiFi and MQTT
  setupWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);

  DBG_PRINT("\n\n=== ESP32 UART Master Ready ===\n");
  DBG_PRINT("Connected to STM32 via UART\n");
  DBG_PRINT("Requesting sensor data every 3 seconds...\n\n");
  DBG_PRINT("RX: GPIO%d, TX: GPIO%d\n", RXD2, TXD2);

  delay(1000);
}

void loop() {
  // Keep MQTT alive
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  DBG_PRINT("--- Requesting Sensor Data ---\n");
  
  // Send 'R' command to STM32 to wake and read sensors
  Serial2.write('R');
  Serial2.flush();
  
  // Read sensor data
  if (readSensorData()) {
    // Verify checksum
    uint8_t calculatedChecksum = calculateChecksum(&packet);
    
    if (packet.checksum == calculatedChecksum) {
      // Display results
      DBG_PRINT("✓ Data received successfully\n");
      DBG_PRINT("=== Sensor Data ===\n");
      
      if (packet.temperature == 0xFF && packet.humidity == 0xFF) {
        DBG_PRINT("⚠️  DHT11 Error\n");
      } else {
        DBG_PRINT("🌡️  Temperature: %d°C\n", packet.temperature);
        DBG_PRINT("💧 Humidity: %d%%\n", packet.humidity);
      }
      
      DBG_PRINT("☀️  Light Level: %.1f%%\n", packet.ldrPercent);
      DBG_PRINT("💦 Water Level: %.1f%%\n", packet.waterPercent);
      
      // Raw data for debugging
      DBG_PRINT("Raw: ");
      uint8_t *pData = (uint8_t *)&packet;
      for (int i = 0; i < sizeof(packet); i++) {
        DBG_PRINT("%02X ", pData[i]);
      }
      DBG_PRINT("\n");
      // Publish CSV: temperature,humidity,ldrPercent,waterPercent
      if (mqttClient.connected()) {
        publishSensorCSV();
      } else {
        DBG_PRINT("MQTT not connected, skipping publish\n");
      }
    } else {
      DBG_PRINT("✗ Checksum error!\n");
      DBG_PRINT("Expected: 0x%02X, Got: 0x%02X\n", 
                    calculatedChecksum, packet.checksum);
    }
  } else {
    DBG_PRINT("✗ Failed to read data - timeout or invalid packet\n");
  }
  
  // Clear any remaining data
  while (Serial2.available()) {
    Serial2.read();
  }
  
  DBG_PRINT("\n");
  delay(1000); // Wait 1 second before next reading
}

bool readSensorData() {
  // Wait for start byte 0xAA with timeout
  unsigned long startTime = millis();
  bool foundStart = false;
  
  while (millis() - startTime < 100) { // 1 second timeout
    if (Serial2.available()) {
      uint8_t byte = Serial2.read();
      if (byte == 0xAA) {
        packet.start = byte;
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
  uint8_t *pData = (uint8_t *)&packet;
  int bytesRead = 1; // Already read start byte
  int bytesToRead = sizeof(SensorPacket) - 1;
  
  startTime = millis();
  while (bytesRead < sizeof(SensorPacket) && millis() - startTime < 500) {
    if (Serial2.available()) {
      pData[bytesRead] = Serial2.read();
      bytesRead++;
    }
    delay(1);
  }
  
  if (bytesRead != sizeof(SensorPacket)) {
    DBG_PRINT("Incomplete packet: got %d bytes, expected %d\n", 
                  bytesRead, sizeof(SensorPacket));
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

// --- WiFi / MQTT helper implementations ---
void setupWiFi() {
  DBG_PRINT("Connecting to WiFi '%s'...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > 15000) {
      DBG_PRINT("WiFi connect timeout, retrying...\n");
      start = millis();
    }
    DBG_PRINT(".");
    delay(500);
  }
  DBG_PRINT("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
}

bool mqttReconnect() {
  if (mqttClient.connected()) return true;

  DBG_PRINT("Attempting MQTT connection to %s:%d ...\n", mqtt_server, mqtt_port);
  // Create a client id based on chip ID
  String clientId = "ESP32-" + String((uint32_t)ESP.getEfuseMac());
  if (mqttClient.connect(clientId.c_str(), mqtt_user, "")) {
    DBG_PRINT("MQTT connected\n");
    return true;
  } else {
    DBG_PRINT("MQTT connect failed, rc=%d - retrying in 5s\n", mqttClient.state());
    delay(5000);
    return false;
  }
}

void publishSensorCSV() {
  char payload[128];
  // Use -1 for missing values (0xFF => sensor error)
  int temp = (packet.temperature == 0xFF) ? -1 : (int)packet.temperature;
  int hum = (packet.humidity == 0xFF) ? -1 : (int)packet.humidity;
  snprintf(payload, sizeof(payload), "%d,%d,%.1f,%.1f", temp, hum, packet.ldrPercent, packet.waterPercent);
  if (mqttClient.publish(mqtt_topic, payload)) {
    DBG_PRINT("Published to %s: %s\n", mqtt_topic, payload);
  } else {
    DBG_PRINT("Failed to publish to %s\n", mqtt_topic);
  }
}