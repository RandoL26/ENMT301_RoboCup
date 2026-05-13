#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HerkulexServo.h>

#define PIN_SW_RX 0
#define PIN_SW_TX 1

#define SERVO_ID_A 4
#define SERVO_ID_B 1

SoftwareSerial   servo_serial(PIN_SW_RX, PIN_SW_TX);
HerkulexServoBus herkulex_bus(servo_serial);
HerkulexServo    servo_a(herkulex_bus, SERVO_ID_A);
HerkulexServo    servo_b(herkulex_bus, SERVO_ID_B);


void setup() {
  Serial.begin(115200);
  servo_serial.begin(115200);
  delay(500);

  // turn power on
  servo_a.setTorqueOn();
  servo_b.setTorqueOn();

  herkulex_bus.prepareSynchronizedMove(100);
  servo_a.setPosition(0);
  servo_b.setPosition(256);
  herkulex_bus.executeMove();

  delay(100 * 11.2f);

  // turn power off
  servo_a.setTorqueOff();
  servo_b.setTorqueOff();
}

void loop() {
  herkulex_bus.update();
}
