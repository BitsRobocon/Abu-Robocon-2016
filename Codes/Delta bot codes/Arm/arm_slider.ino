int motL=2;
int motR=3;

int irR=6;
int irL=7;

int trigPin = 10;
int echoPin = 9;

bool preR=HIGH;
bool preL=LOW;

void setup() 
{
  Serial.begin(9600);
  //pinMode(enab,OUTPUT);
  pinMode(motL,OUTPUT);
  pinMode(motR,OUTPUT);  

  pinMode(irR,INPUT);
  pinMode(irL,INPUT);

  pinMode(trigPin,OUTPUT );
  pinMode(echoPin,INPUT );
}

void loop()
{ 
  //analogWrite(enab,HIGH);
  
  int right=digitalRead(irR);  
  int left=digitalRead(irL);
  //Serial.print(left);
  //Serial.print(right);

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  double  duration = pulseIn(echoPin, HIGH);
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  double  to_cm = duration / 29 / 2;
  Serial.println(to_cm);

  if(right==1 && to_cm>40)
  {
    digitalWrite(motL,HIGH);
    digitalWrite(motR,LOW);
    preL=HIGH;
    preR=LOW;
  }
  else if(left==1 && to_cm>40)
  {
    digitalWrite(motL,LOW);
    digitalWrite(motR,HIGH);    
    preL=LOW;
    preR=HIGH;
  }  
  else if(to_cm<=40)
  {
    digitalWrite(motL,HIGH);
    digitalWrite(motR,HIGH);
  }
  else
  {
    digitalWrite(motL,preL);
    digitalWrite(motR,preR);
  }
}