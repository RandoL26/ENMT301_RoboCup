#include "BigServo.h"

Servo bigServo;

void bigServo_setup()
{
    bigServo.attach(bigServoPin, 500, 2500);
    //bigServo.writeMicroseconds(2400);
}

void bigServo_move(int angle)
{
    bigServo.write(angle);
}