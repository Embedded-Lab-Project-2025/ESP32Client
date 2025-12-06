#include <s3servo.h>
#include "servo.h"
#include "debug.h"

s3servo servo1;
int servoPin = 14;   //use the GPIO numbers!

void servoBegin() {
    servo1.attach(servoPin);
    DBG_PRINT("Servo attached on pin %d\n", servoPin);
}

void servoTrigger() {
    DBG_PRINT("Servo trigger: sweeping 30->150->30\n");
    for(int i = 30; i <= 150; i++) {
        servo1.write(i);
        delay(22);
    }
    for(int i = 150; i >= 30; i--) {
        servo1.write(i);
        delay(22);
    }
    DBG_PRINT("Servo sweep complete\n");
}