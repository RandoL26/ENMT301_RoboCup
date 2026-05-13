/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogReadSerial
*/

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() 
{
  int sensorValue1,sensorValue2,sensorValue3,sensorValue4,sensorValue5,
      sensorValue6,sensorValue7,sensorValue8,sensorValue9,sensorValue10;
  
  // read the input on analog pin 0:
  sensorValue1 = analogRead(A9);
  sensorValue2 = analogRead(A8);
  sensorValue3 = analogRead(A7);
  sensorValue4 = analogRead(A1);
  sensorValue5 = analogRead(A11);
  sensorValue6 = analogRead(A13);

  sensorValue7 = analogRead(A6);
  sensorValue8 = analogRead(A0);
  sensorValue9 = analogRead(A10);
  sensorValue10 = analogRead(A12);

  Serial.printf("%04d %04d: %04d %04d %04d %04d: %04d %04d %04d %04d\n",
  sensorValue1,sensorValue2,sensorValue3,sensorValue4,sensorValue5,
  sensorValue6,sensorValue7,sensorValue8,sensorValue9,sensorValue10);
  // print out the value you read:
  /*
  Serial.print(sensorValue1);
  Serial.print(" ");
  Serial.print(sensorValue2);
  Serial.print(": ");
  Serial.print(sensorValue3);
  Serial.print(" ");
  Serial.print(sensorValue4);
  Serial.print(" ");
  Serial.print(sensorValue5);
  Serial.print(" ");
  Serial.print(sensorValue6);
  Serial.print(": ");
  Serial.print(sensorValue7);
  Serial.print(" ");
  Serial.print(sensorValue8);
  Serial.print(" ");
  Serial.print(sensorValue9);
  Serial.print(" ");
  Serial.print(sensorValue10);
  Serial.print(" ");

  
  Serial.println(" ");
  */
  
  delay(1);        // delay in between reads for stability
}
