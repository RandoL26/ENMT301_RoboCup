#include <Servo.h>

Servo myservoA,myservoB,myservoC,myservoD;      // create servo object to control a servo

void setup()
{   
  myservoA.attach(2);  // attaches the servo  to the servo object useing pin 3
  myservoB.attach(3);  // attaches the servo  to the servo object useing pin 3
  myservoC.attach(4);  // attaches the servo  to the servo object useing pin 3
  myservoD.attach(5);  // attaches the servo  to the servo object useing pin 3
}

void loop() 
{ 
  myservoA.writeMicroseconds(1000);      // sets the servo position full speed backward
  myservoB.writeMicroseconds(1000);      // sets the servo position full speed backward
  myservoC.writeMicroseconds(1000);      // sets the servo position full speed backward
  myservoD.writeMicroseconds(1000);      // sets the servo position full speed backward
  delay(1500);                           // waits for the servo to get there 
  myservoA.writeMicroseconds(2000);      // sets the servo position full speed forward
  myservoB.writeMicroseconds(2000);      // sets the servo position full speed forward
  myservoC.writeMicroseconds(2000);      // sets the servo position full speed forward
  myservoD.writeMicroseconds(2000);      // sets the servo position full speed forward
  delay(1500);                           // waits for the servo to get there 
} 
