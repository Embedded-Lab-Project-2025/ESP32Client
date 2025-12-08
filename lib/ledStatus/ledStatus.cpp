#include "ledStatus.h"

// NeoPixel object
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Current LED status
static LedStatus currentStatus = STATUS_NORMAL;

// RGB values for each status (R, G, B) - 0-255
static const uint32_t STATUS_COLORS[] = {
    pixels.Color(0, 255, 0),   // STATUS_NORMAL: Green
    pixels.Color(255, 0, 0),   // STATUS_WIFI_DISCONNECTED: Red
    pixels.Color(0, 0, 255),   // STATUS_MQTT_DISCONNECTED: Blue
    pixels.Color(255, 255, 0)  // STATUS_SENSOR_ERROR: Yellow
};

void ledStatusBegin() {
    pixels.begin();
    pixels.setBrightness(10); // Set brightness (0-255)
    pixels.clear();
    pixels.show();
    
    currentStatus = STATUS_NORMAL;
    // Set initial color
    uint32_t color = STATUS_COLORS[currentStatus];
    pixels.setPixelColor(0, color);
    pixels.show();
}

void setLedStatus(LedStatus status) {
    currentStatus = status;
    // Set color based on current status
    uint32_t color = STATUS_COLORS[currentStatus];
    pixels.setPixelColor(0, color);
    pixels.show();
}