#include "wifi_mqtt.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "debug.h"
#include <Arduino.h>
#include <String.h>

static const char* mqtt_server = "192.168.1.168"; 
static const uint16_t mqtt_port = 1883;
static const char* mqtt_user = "board"; // requested username
static const char* mqtt_topic_data = "sensor/data";
static const char* mqtt_topic_action = "sensor/action";

static WiFiClient espClient;
static PubSubClient mqttClient(espClient);
static void (*servoCallback)() = nullptr;

void setServoTrigger(void (*callback)()) {
  servoCallback = callback;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  DBG_PRINT("MQTT message received on topic: %s\n", topic);
  if (strcmp(topic, mqtt_topic_action) == 0 && servoCallback) {
    DBG_PRINT("Triggering servo\n");
    servoCallback();
  }
}

bool wifiMqttBegin(const String& wifi_ssid, const String& wifi_password, unsigned long timeoutMs) {
  DBG_PRINT("Connecting to WiFi '%s'...\n", wifi_ssid.c_str());
  WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    DBG_PRINT(".");
    delay(500);
  }
  if (WiFi.status() != WL_CONNECTED) {
    DBG_PRINT("\nWiFi not connected after %lu ms\n", timeoutMs);
    return false;
  }
  DBG_PRINT("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  return true;
}

bool mqttReconnect() {
  if (mqttClient.connected()) return true;

  DBG_PRINT("Attempting MQTT connection to %s:%d ...\n", mqtt_server, mqtt_port);
  // Create a client id based on chip ID
  String clientId = "ESP32-" + String((uint32_t)ESP.getEfuseMac());
  if (mqttClient.connect(clientId.c_str(), mqtt_user, "")) {
    DBG_PRINT("MQTT connected\n");
    mqttClient.subscribe(mqtt_topic_action);
    DBG_PRINT("Subscribed to %s\n", mqtt_topic_action);
    return true;
  } else {
    DBG_PRINT("MQTT connect failed, rc=%d - retrying in 5s\n", mqttClient.state());
    delay(5000);
    return false;
  }
}

void mqttLoop() {
  if (!mqttClient.connected()) mqttReconnect();
  mqttClient.loop();
}

bool publishSensorCSV(const SensorPacket *pkt) {
  char payload[32];
  // Use -1 for missing values (0xFF => sensor error)
  int temp = (pkt->temperature == 0xFF) ? -1 : (int)pkt->temperature;
  int hum = (pkt->humidity == 0xFF) ? -1 : (int)pkt->humidity;
  snprintf(payload, sizeof(payload), "%d,%d,%.2f,%.2f", temp, hum, pkt->ldrPercent, pkt->waterPercent);
  if (mqttClient.publish(mqtt_topic_data, payload)) {
    DBG_PRINT("Published to %s: %s\n", mqtt_topic_data, payload);
    return true;
  } else {
    DBG_PRINT("Failed to publish to %s\n", mqtt_topic_data);
    return false;
  }
}
