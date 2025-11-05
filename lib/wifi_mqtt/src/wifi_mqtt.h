#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include "sensor.h"

// Initialize WiFi and MQTT client (connect to WiFi and set MQTT server)
// Returns true if WiFi connected within timeout, false otherwise.
bool wifiMqttBegin(const String& wifi_ssid, const String& wifi_password, unsigned long timeoutMs = 15000);
// Call regularly to keep MQTT alive
void mqttLoop();
// Ensure connected (blocking retry behavior similar to previous impl)
bool mqttReconnect();
// Publish sensor packet as CSV: temp,hum,ldr,water
bool publishSensorCSV(const SensorPacket *pkt);

#endif // WIFI_MQTT_H
