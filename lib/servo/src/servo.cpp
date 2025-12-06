#include <s3servo.h>
#include "servo.h"
#include "debug.h"

s3servo servo1;
int servoPin = 14;
int restAngle = 35;
int pourAngle = 150;

void servoBegin() {
    servo1.attach(servoPin);
    DBG_PRINT("Servo attached on pin %d\n", servoPin);
}

void servoTrigger() {
    DBG_PRINT("Resetting rain gauge\n");
    servo1.write(pourAngle);
    delay(1000);  // Hold to pour
    int shakeMin = pourAngle - 15;
    int shakeMax = pourAngle + 15;
    for(int shake = 0; shake < 50; shake++) {
        servo1.write(shakeMin);
        delay(25);
        servo1.write(shakeMax);
        delay(25);
    }
    // Return to rest position
    servo1.write(restAngle);
    delay(100);
    DBG_PRINT("Rain gauge reset\n");
}