/*
Test out the low cost ultrasound sensor
*/
const int AtrigPin = 3;
const int AechoPin = 2;

const int BtrigPin = 5;
const int BechoPin = 4;
 
void setup() 
{  
  pinMode(AtrigPin, OUTPUT);            //Setup ultrasound pins
  pinMode(AechoPin, INPUT);

  pinMode(BtrigPin, OUTPUT);            //Setup ultrasound pins
  pinMode(BechoPin, INPUT);
  
  Serial.begin(9600);                  // initialize serial communication:
}
 
void loop()
{
  long durationA,durationB, Acm,Bcm;
 
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(AtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(AtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(AtrigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  durationA = pulseIn(AechoPin, HIGH);

  digitalWrite(BtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(BtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(BtrigPin, LOW);
  durationB = pulseIn(BechoPin, HIGH);
  
 
  // convert the time into a distance
  Acm = microsecondsToCentimeters(durationA);
  Bcm = microsecondsToCentimeters(durationB);
  Serial.print(Acm);
  Serial.print(" ");
  Serial.print(Bcm);
  Serial.println();
  delay(100);
}
 
 
long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
} 
