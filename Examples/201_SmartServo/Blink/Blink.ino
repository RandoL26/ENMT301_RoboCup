#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HerkulexServo.h>

#define PIN_SW_RX 0
#define PIN_SW_TX 1
#define SERVO_ID  4

SoftwareSerial   servo_serial(PIN_SW_RX, PIN_SW_TX);
HerkulexServoBus herkulex_bus(servo_serial);
HerkulexServo    my_servo(herkulex_bus, SERVO_ID);


void setup() {
  Serial.begin(115200);
  servo_serial.begin(115200);
}

void loop() {
  herkulex_bus.update();

  my_servo.setLedColor(HerkulexLed::White);
  delay(1000);
  my_servo.setLedColor(HerkulexLed::Off);
  delay(1000);
}
