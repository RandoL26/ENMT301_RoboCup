#ifndef BIGSERVO_H
#define BIGSERVO_H

#include <Servo.h>
#define bigServoPin 27

void bigServo_setup();

void bigServo_move(int angle);


#endif