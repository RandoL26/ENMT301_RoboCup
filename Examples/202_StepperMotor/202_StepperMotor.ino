
int MAdirpin = 32;
int MAsteppin = 33;
int MBdirpin = 30;
int MBsteppin = 31;

void setup()
{   
  pinMode(MAdirpin,OUTPUT);
  pinMode(MAsteppin,OUTPUT);
  pinMode(MBdirpin,OUTPUT);
  pinMode(MBsteppin,OUTPUT);
}

void loop()
{
  int j;
  
  //Set direction for all channels

  digitalWrite(MAdirpin,LOW);
  digitalWrite(MBdirpin,LOW);
  
  for(j=0;j<=1000;j++)            //Move 1000 steps
  {

    digitalWrite(MAsteppin,LOW);
    digitalWrite(MBsteppin,LOW);
    delayMicroseconds(20);
    digitalWrite(MAsteppin,HIGH);
    digitalWrite(MBsteppin,HIGH);
    delay(1);
  }
  digitalWrite(MAdirpin,HIGH);
  digitalWrite(MBdirpin,HIGH);
    for(j=0;j<=1000;j++)            //Move 1000 steps
  {

    digitalWrite(MAsteppin,LOW);
    digitalWrite(MBsteppin,LOW);
    delayMicroseconds(20);
    digitalWrite(MAsteppin,HIGH);
    digitalWrite(MBsteppin,HIGH);
    delay(1);
  }
}
