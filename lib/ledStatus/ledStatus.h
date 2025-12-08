#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// NeoPixel LED configuration
#define NEOPIXEL_PIN 48  // Adjust based on your board
#define NUM_PIXELS 1     // Single RGB LED

// LED status enumeration
enum LedStatus {
    STATUS_NORMAL,           // Green solid
    STATUS_WIFI_DISCONNECTED, // Red solid
    STATUS_MQTT_DISCONNECTED, // Blue solid
    STATUS_SENSOR_ERROR      // Yellow solid
};

// Initialize LED status system
void ledStatusBegin();

// Set the current LED status
void setLedStatus(LedStatus status);

#endif // LED_STATUS_H