#include "BigServo.h"

Servo bigServo;

void bigServo_setup()
{
    bigServo.attach(bigServoPin, 500, 2500);
    bigServo.write(rest_angle);
    //bigServo.writeMicroseconds(2400);
}

void bigServo_move(int angle)
{
    if (angle >= min_angle && angle <= max_angle) {
        bigServo.write(angle);
    }
}