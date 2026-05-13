/*************************************************************
digitalRead.ino
SparkFun SX1509 I/O Expander Example: digital in (digitalRead)
Jim Lindblom @ SparkFun Electronics
Original Creation Date: September 21, 2015
https://github.com/sparkfun/SparkFun_SX1509_Arduino_Library

This example demonstrates the SX1509's digitalRead
functionality. A pin can either be set as an INPUT or
INPUT_PULLUP. We'll attach an active-low button to an 
INPUT_PULLUP input, then whenever the button read's LOW, we'll
read the state of another INPUT pin.

After uploading the sketch, open your serial monitor and set 
it to 9600 baud.

Hardware Hookup:
	SX1509 Breakout ------ Arduino -------- Breadboard
	      GND -------------- GND
	      3V3 -------------- 3.3V
		  SDA ------------ SDA (A4)
		  SCL ------------ SCL (A5)
		  0 ---------------------------------]BTN[----GND
		  8 -----------------------------Jumper (GND or 3.3V)

Development environment specifics:
	IDE: Arduino 1.6.5
	Hardware Platform: Arduino Uno
	SX1509 Breakout Version: v2.0

This code is beerware; if you see me (or any other SparkFun 
employee) at the local, and you've found our code helpful, 
please buy us a round!

Distributed as-is; no warranty is given.
*************************************************************/

#include <Wire.h> // Include the I2C library (required)
#include <SparkFunSX1509.h> // Include SX1509 library

// SX1509 I2C address (set by ADDR1 and ADDR0 (00 by default):
const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address
SX1509 io; // Create an SX1509 object to be used throughout

// SX1509 Pins:
const byte SX1509_AIO0 = 0; 
const byte SX1509_AIO1 = 1; 
const byte SX1509_AIO2 = 2; 
const byte SX1509_AIO3 = 3; 
const byte SX1509_AIO4 = 4; 
const byte SX1509_AIO5 = 5; 
const byte SX1509_AIO6 = 6; 
const byte SX1509_AIO7 = 7; 
const byte SX1509_AIO8 = 8; 
const byte SX1509_AIO9 = 9; 
const byte SX1509_AIO10 = 10; 
const byte SX1509_AIO11 = 11; 
const byte SX1509_AIO12 = 12; 
const byte SX1509_AIO13 = 13; 
const byte SX1509_AIO14 = 14; 
const byte SX1509_AIO15 = 15; 


void setup() 
{
  // Serial is used in this example to display the input value
  // of the SX1509_INPUT_PIN input:
  Serial.begin(9600);
  // Call io.begin(<address>) to initialize the SX1509. If it
  // successfully communicates, it'll return 1.
  if (!io.begin(SX1509_ADDRESS))
  {
    Serial.println("Failed to communicate.");
    while (1) ;
  }

  // use io.pinMode(<pin>, <mode>) to set input pins as either 
  // INPUT or INPUT_PULLUP. Set up a floating (or jumpered to 
  // either GND or 3.3V) pin to an INPUT:
  io.pinMode(SX1509_AIO0, INPUT);
  io.pinMode(SX1509_AIO1, INPUT);
  io.pinMode(SX1509_AIO2, INPUT);
  io.pinMode(SX1509_AIO3, INPUT);
  io.pinMode(SX1509_AIO4, INPUT);
  io.pinMode(SX1509_AIO5, INPUT);
  io.pinMode(SX1509_AIO6, INPUT);
  io.pinMode(SX1509_AIO7, INPUT);
  io.pinMode(SX1509_AIO8, INPUT);
  io.pinMode(SX1509_AIO9, INPUT);
  io.pinMode(SX1509_AIO10, INPUT);
  io.pinMode(SX1509_AIO11, INPUT);
  io.pinMode(SX1509_AIO12, INPUT);
  io.pinMode(SX1509_AIO13, INPUT);
  io.pinMode(SX1509_AIO14, INPUT);
  io.pinMode(SX1509_AIO15, INPUT);

}

void loop() 
{
  bool  bAIO0,bAIO1,bAIO2,bAIO3,bAIO4,bAIO5,bAIO6,bAIO7,
        bAIO8,bAIO9,bAIO10,bAIO11,bAIO12,bAIO13,bAIO14,bAIO15;
  char  sOut[100];

  bAIO0=io.digitalRead(SX1509_AIO0);
  bAIO1=io.digitalRead(SX1509_AIO1);
  bAIO2=io.digitalRead(SX1509_AIO2);
  bAIO3=io.digitalRead(SX1509_AIO3);
  bAIO4=io.digitalRead(SX1509_AIO4);
  bAIO5=io.digitalRead(SX1509_AIO5);
  bAIO6=io.digitalRead(SX1509_AIO6);
  bAIO7=io.digitalRead(SX1509_AIO7);
  bAIO8=io.digitalRead(SX1509_AIO8);
  bAIO9=io.digitalRead(SX1509_AIO9);
  bAIO10=io.digitalRead(SX1509_AIO10);
  bAIO11=io.digitalRead(SX1509_AIO11);
  bAIO12=io.digitalRead(SX1509_AIO12);
  bAIO13=io.digitalRead(SX1509_AIO13);
  bAIO14=io.digitalRead(SX1509_AIO14);
  bAIO15=io.digitalRead(SX1509_AIO15);
  
  sprintf(sOut,"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",bAIO0,bAIO1,bAIO2,bAIO3,bAIO4,bAIO5,bAIO6,bAIO7,bAIO8,bAIO9,bAIO10,bAIO11,bAIO12,bAIO13,bAIO14,bAIO15);
  
  Serial.println(sOut);
  delay(50);
}
