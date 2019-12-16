
#include <Servo.h> 
 
Servo rtservo,ltservo,armservo,gripperservo,bigservo;  
int rtservo_pin=10,
    ltservo_pin=9,
    armservo_pin=11,
    gripperservo_pin=6;
bool clawstate=true,prev=false;
int i=0,tem=0;
const int buttonPin = 5;
int buttonState;        
int lastButtonState = LOW;
long lastDebounceTime = 0; 
long debounceDelay = 50;
int err=10;
int link1_length=35,link2_length=13;
int basepos = 0, armpos = 0,gripperpos,a,f=0,r=1; 

///////////////////////////////////////////Ultrasonic

int trig[3]={2,3,4};       
int echo[3]={17,18,19};       
double senval[3],lim=3000;  
double senvar[3];  
float maxvar=0.1;  
int p=0,pl=10;

/////////////////////////SERVO EXTREME VALUES

int baseservo_min=10,baseservo_max=150;
int armservo_min=10,armservo_max=170;

//////////////////////////////////////////////////

int base_currentpos=baseservo_min;
int arm_currentpos=armservo_min;
int count=0;

/////////////////////////value for angle taken as input here
int pot1=0,pot2=0;
int pot1_pin=14,pot2_pin=16;
/////////////////////////////////////////////////////////////////////

///////////////////////////INVERSE KINEMATIC/////////////////////

int ServoS_1_Angle = 0;
int ServoS_2_Angle = 0;

                // Define arm Constants
const float l1 = 30.0;      // lower joint length (cm)
const float l2 = 20.0;      // upper joint length (cm)

                    // Correction factors to align servo values with their respective axis
const float S_1_CorrectionFactor = 70 ;     // Align arm "a" with the horizontal when at 0 degrees
const float S_2_CorrectionFactor = 45;     // Align arm "b" with arm "a" when at 0 degrees

                      // Correction factor to shift origin out to edge of the mount
const float X_CorrectionFactor = 0;       // X direction correction factor (cm)
const float Y_CorrectionFactor = 0;       // Y direction correction factor (cm)

                         // Angle Variables
float A;            //Angle oppposite side a (between b and c)
float B;            //Angle oppposite side b
float C;            //Angle oppposite side c
float c;            // Hypotenuse legngth in cm
float theta;        //Angle formed between line from origin to (x,y) and the horizontal

const float pi = M_PI;  //Store pi in a less annoying format
int md=-1;
int X=0,Y=0,Z=0;  //  arm pos
float deltax=1.0,deltay=1.0,deltaz=2.0;

/////////////////////////////////////////////////////////////////////////////////////////////

int motL=2;
int motR=3;

int trigPin1 = 7;
int echoPin1 = 8;

int trigPin2 = 12;
int echoPin2 = 13;

bool preR=HIGH;
bool preL=LOW;

int dist1,dist2;
int walldist;
/////////////////////////////////////FUNCTIONS//////

/*void rackpinion()
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
}*/

////////////////////////////////////////////////////////

double readUS( int i )        
{
  digitalWrite(trig[i], LOW);
  delayMicroseconds(2);
  digitalWrite(trig[i], HIGH);
  delayMicroseconds(3);
  digitalWrite(trig[i], LOW);
    
  return (double)( pulseIn(echo[i], HIGH) )/(double)10;
}

/////////////////////////////////////////////////////////////////////////

void calcangles(float x,float y)//,int z)
{
  x = x + X_CorrectionFactor;
  y = y + Y_CorrectionFactor;
  c = sqrt( sq(x) + sq(y) );                                            // pythagorean theorem
  B = (acos( (sq(l2) - sq(l1) - sq(c))/(-2*l1*c) )) * (180/pi);            // Law of cosines: Angle opposite upper arm section
  C = (acos( (sq(c) - sq(l2) - sq(l1))/(-2*l1*l2) )) * (180/pi);            // Law of cosines: Angle opposite hypotenuse
  theta = (asin( y / c )) * (180/pi);                                   // Solve for theta to correct for lower joint's impact on upper joint's angle
  ServoS_1_Angle = S_1_CorrectionFactor - ((B*md) + theta);                    // Find necessary angle. Add Correction
  ServoS_2_Angle = C - S_2_CorrectionFactor  ;
  X=x;
  Y=y;
}

void movearm(int theta1,int theta2)
{
  theta1=constrain(theta1,baseservo_min,baseservo_max);
  theta2=constrain(theta2,armservo_min,armservo_max);
  if(theta1>=base_currentpos)
  {
    for(int i=base_currentpos;i<=theta1;i+=1)
    {
       rtservo.write(i+err);             
       ltservo.write(180-i);
       delay(15);
    }
  }
  else
  {
    for(int i=base_currentpos;i>=theta1;i-=1)
    {
       rtservo.write(i+err);             
       ltservo.write(180-i);
       delay(15);
    }
  }
  base_currentpos=theta1;
  
  if(theta2>=arm_currentpos)
  {
    for(int i=arm_currentpos;i<=theta2;i+=1)
    {
       armservo.write(i);             
       delay(15);
    }
  }
  else
  {
    for(int i=arm_currentpos;i>=theta2;i-=1)
    {
       armservo.write(i);             
       delay(15);
    }
  }
  arm_currentpos=theta2;
}

void moveplatform(float z)
{
  sensedist();
  Serial.print(dist1);
  Serial.print("\t");
  Serial.print(dist2);
  Serial.println("\t");
  if(z>0&&z>Z)
  {
    while(dist1>walldist-z)
    {
      sensedist();
      moveleft();
      Serial.println("1\t");   
    }
  }
  else if(z>0&&z<Z)
  {
    while(dist1<walldist-z)
    {
      sensedist();
      moveright();
      Serial.println("2\t");
    }
  }
  else if(z<0&&z>Z)
  {
    while(dist2<walldist+z)
    {
      sensedist();
      moveleft();
      Serial.println("3\t");
    }
  }
  else if(z<0&&z<Z)
  {
    while(dist2>walldist+z)
    {
      sensedist();
      moveright();
      Serial.print(walldist);
      Serial.print("\t");
      Serial.print(dist2);
      Serial.println("\t4\t");
    }
  }
  stopm();
  Z=z;
}

  
void movexyz(float x,float y,float z)
{
  delay(1000);
  calcangles(x,y);
  //moveplatform(z);
  movearm(ServoS_1_Angle,ServoS_2_Angle);
  Serial.print("\t");
  Serial.print(ServoS_1_Angle);
  Serial.print("\t");
  Serial.println(ServoS_2_Angle);
}
    
void sensedist()
{
  delay(50);
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  double  duration1 = pulseIn(echoPin1, HIGH);
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  dist1 = duration1 / 29 / 2;
  
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  double  duration2 = pulseIn(echoPin2, HIGH);
  dist2 = duration2 / 29 / 2;
}

void moveleft()
{
  digitalWrite(motL,HIGH);
  digitalWrite(motR,LOW);
}
void moveright()
{
  digitalWrite(motL,LOW);
  digitalWrite(motR,HIGH);
}
void stopm()
{
  digitalWrite(motL,LOW);
  digitalWrite(motR,LOW);
}
  
  
///////////////////////////////////////////////////////////////////////////



void automatic()
{
  /*calcangles(30,-24);
  Serial.print(ServoS_1_Angle);
  Serial.print("\t");
  Serial.println(ServoS_2_Angle);*/
  while(f==0)
  {
    movearm(baseservo_min,armservo_min);
    delay(1000);
    movexyz(10,40,0);
    delay(5000);
    f=1;
  }
  //movearm(ServoS_1_Angle,ServoS_2_Angle);*/
  while(r==1)
  {
    Serial.println("hello");
    ultrasonic();
  }
  /*movexyz(40,-2,0);
  delay(5000);
  movexyz(40,2,0);
  delay(5000);
  movexyz(38,2,0);
  delay(5000);
  movexyz(42,2,0);*/
}  



////////////////////////////////////////Ultrasonic///////////////////////

void ultrasonic()
{
  delay(0000);
  
  for(int i=0;i<3;i++)
    senval[i]=readUS(i);         //Store from function readUS
  for(int i=0;i<3;i++)
    senvar[i]=(3*senval[i]/(senval[0]+senval[1]+senval[2]))-1;   //Store variance
    
  /*if((senval[0]>lim)||(senval[1]>lim)||(senval[2]>lim))
  {
    p++;
    if(p>pl)
    {
      if((senval[0]>lim)&&(senval[1]>lim)&&(senval[2]>lim))
      {
        Serial.print("C");
      }
      else if((senval[0]>lim)&&(senval[1]<lim)&&(senval[2]<lim))
      {
        Serial.print("D R");
      }
      else if((senval[0]<lim)&&(senval[1]>lim)&&(senval[2]<lim))
      {
        Serial.print("U");
      }
        else if((senval[0]<lim)&&(senval[1]<lim)&&(senval[2]>lim))
      {
        Serial.print("D L");
      }
      else if((senval[0]<lim)&&(senval[1]>lim)&&(senval[2]>lim))
      {
        Serial.print("U L");
      }
      else if((senval[0]>lim)&&(senval[1]<lim)&&(senval[2]>lim))
      {
        Serial.print("D");
      }
      else if((senval[0]>lim)&&(senval[1]>lim)&&(senval[2]<lim))
      {
        Serial.print("U R");
      }
      else 
      {
        Serial.print("C (all outside)");
      }
   }
  }
  else*/ 
  {
    if(senvar[0]<maxvar&&senvar[0]>-maxvar&&senvar[1]<maxvar&&senvar[1]>-maxvar&&senvar[2]<maxvar&&senvar[0]>-maxvar)
    {
      Serial.print("C");
     //////////////////////////////attach fan
     attachfan();
    }
    else if(senvar[0]<-maxvar&&senvar[1]>0&&senvar[2]>0)
    {
      Serial.print("U R");
      movexyz(X,Y+deltay,Z-deltaz);
    }
    else if(senvar[1]<-maxvar&&senvar[2]>0&&senvar[0]>0)
    {
      Serial.print("D");
      movexyz(X,Y-deltay,Z);
    }
    else if(senvar[2]<-maxvar&&senvar[0]>0&&senvar[1]>0)
    {
      Serial.print("U L");
      movexyz(X,Y+deltay,Z+deltaz);
    }
    else if(senvar[0]>maxvar&&senvar[1]<0&&senvar[2]<0)
    {
      Serial.print("D L");
      movexyz(X,Y-deltay,Z+deltaz);
    }
    else if(senvar[1]>maxvar&&senvar[2]<0&&senvar[0]<0)
    {
      Serial.print("U");
      movexyz(X,Y+deltay,Z);
    }
    else if(senvar[2]>maxvar&&senvar[0]<0&&senvar[1]<0)
    {
      Serial.print("D R");
      movexyz(X,Y-deltay,Z-deltaz);
    }
    else 
    {
      Serial.print("!!!");
      //movexyz(X+deltax,Y,Z);
    }
  }
  Serial.print("\t");
  Serial.print(senvar[0]);Serial.print("\t");
  Serial.print(senvar[1]);Serial.print("\t");
  Serial.print(senvar[2]);Serial.println("\t");
}
  
///////////////////////////////////////////////////////
  
  
void attachfan()
{
  digitalWrite(13,HIGH);
  movexyz(X-8,Y+3,Z);
  r=0;
}
  
void gripper()
{
  Serial.println(clawstate);
  if(clawstate)//&&!prev)
  {
    for(int x=0;x<=100;x+=1)
    {
      gripperservo.write(x);
      delay(3);
    }
  }
    else
  {
    for(int x=100;x>=0;x-=1)
    {
      gripperservo.write(x);
      delay(3);
    }
  }
}
    
    
void movebaseservo(int pos)
{
  Serial.print(base_currentpos);
  if((base_currentpos<(baseservo_min)))
  {
    base_currentpos=baseservo_min;
    return;
  }
  else if ((base_currentpos>(baseservo_max)))
  {
    base_currentpos=baseservo_max;
    return;
  }
  int i=0;
  if(pos==0)
  {
    i=base_currentpos-1;
    rtservo.write(i+err);             
    ltservo.write(180-i);
    delay(15);
  }
  else if(pos==1)
  {
    i=base_currentpos;
    rtservo.write(i+err);             
    ltservo.write(180-i);
    delay(15);
  }
  else if(pos==2)
  {
    i=base_currentpos+1;
    rtservo.write(i+err);             
    ltservo.write(180-i);
    delay(15);
  }
  base_currentpos=i;
}

void movearmservo(int pos)
{
  Serial.println(arm_currentpos);
  if((arm_currentpos<(armservo_min)))
  {
    arm_currentpos=armservo_min;
    return;
  }
  else if((arm_currentpos>(armservo_max)))
  {
    arm_currentpos=armservo_max;
    return;
  }
  int i=0;
  if(pos==0)
  {
    i=arm_currentpos-1;
    armservo.write(i);
    delay(15);
  }
  else if(pos==1)
  {
    i=arm_currentpos;
    armservo.write(i);
    delay(15);
  }
  else if(pos==2)
  {
    i=arm_currentpos+1;
    armservo.write(i);
    delay(15);
  }
  arm_currentpos=i;
}

/////////////////////////////////////////MANUAL ARM////
 
void manual()
{
    ////////////////////Reading pin values/////////////////////////////////////
  
  int reading = digitalRead(buttonPin);
  pot1=analogRead(pot1_pin);
  pot2=analogRead(pot2_pin);
  if(pot1>700)pot1=2;
  else if(pot1<300)pot1=0;
  else pot1=1;
  if(pot2>700)pot2=2;
  else if(pot2<300)pot2=0;
  else pot2=1;
  /*Serial.print(pot1);
  Serial.print("\t");
  Serial.println(pot2);*/
  
  ///////////Serialread///////////////////////////////////////
  
  /*i=0;
  while(Serial.available()>0)
  {
    char ch1=Serial.read();
    Serial.println(ch1);
    while (Serial.available()>0)
    {
      char ch = Serial.read();
      tem = ch - '0';
      tem=i*10 + tem;
      i=tem;
      Serial.println(tem);
    }
    i=0;
    if(ch1=='a')
    {
      pot1=tem;
    }
    else if(ch1=='b')
    {
      pot2=tem;
    }
  Serial.println(pot1);
  Serial.println(pot2);
  }*/
  
  //////////////////////////////////////////////////////
  
  //////////////Movement commands
  
  movebaseservo(pot1);
  movearmservo(pot2);
  if (reading != lastButtonState) 
  {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (reading != buttonState) 
    {
      buttonState = reading;
      if (buttonState == HIGH) 
      {
        clawstate=!clawstate;
        gripper();
        count++;
      }
    }
  }
  lastButtonState = reading;
  
}


///////////////////////////////////////////////////////////////////////////////////////////////  

/////////////////////////////////////SETUP

void setup() 
{ 
  ltservo.attach(ltservo_pin);
  rtservo.attach(rtservo_pin);
  armservo.attach(armservo_pin); 
  //gripperservo.attach(gripperservo_pin);
  //rtservo.write(baseservo_min + err);
  //ltservo.write(180 - baseservo_min);
  //armservo.write(armservo_min);
  /*pinMode(buttonPin, INPUT);
  pinMode(motL,OUTPUT);
  pinMode(motR,OUTPUT);  
  pinMode(trigPin1,OUTPUT );
  pinMode(echoPin1,INPUT );
  pinMode(trigPin2,OUTPUT );
  pinMode(echoPin2,INPUT );*/
  //digitalWrite(motL,HIGH);
  //digitalWrite(motR,HIGH);
  //pinMode(pot1_pin,INPUT);
  //pinMode(pot2_pin,INPUT);
  for(int i=0;i<3;i++)
  {
    pinMode(trig[i], OUTPUT);
    pinMode(echo[i], INPUT);
  }
  /*digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  double  duration = pulseIn(echoPin2, HIGH);
  walldist = duration / 29 / 2;*/
  Serial.begin(9600);
  /////////////////////////add lipo switch code here/////////////
  
  /////////////////////////////////////////////////////////////////
} 


////////////////////////////////////////////////////////LOOP
 
void loop() 
{ 
  
  //manual();
  
  automatic();
  
} 




