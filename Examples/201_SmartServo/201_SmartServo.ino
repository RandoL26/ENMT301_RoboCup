#include "Herkulex.h"
int n=0x04; //motor ID - verify your ID !!!!
/*
0xfd or 253 is the default
0xfe or 254 is broadcast, ie all motors
*/
 
void setup() 
{
  
  Herkulex.beginSerial2(115200); //open serial port 2 to talk to the motors
  Herkulex.reboot(n);            //reboot motor
  delay(500);
  Herkulex.initialize();         //initialize motors
  delay(200); 
}
 
void loop()
{
  Herkulex.moveOneAngle(n, -100, 1000, LED_BLUE); //move motor backward
  delay(1200);
  Herkulex.moveOneAngle(n, 100, 1000, LED_GREEN); //move motor forward
  delay(1200);   
}
