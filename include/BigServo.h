#ifndef BIGSERVO_H
#define BIGSERVO_H

#include <Servo.h>
#define bigServoPin 27
#define max_angle 169
#define min_angle 120
#define rest_angle 150

void bigServo_setup();

void bigServo_move(int angle);


#endif