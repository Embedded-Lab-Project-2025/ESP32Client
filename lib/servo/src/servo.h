#ifndef SERVO_H
#define SERVO_H

// Initialize servo on pin 18
void servoBegin();
// Trigger servo sweep: 0 -> 90 -> 0 degrees
void servoTrigger();

#endif // SERVO_H