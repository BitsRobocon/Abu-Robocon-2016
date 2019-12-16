#include <PS2X_lib.h>

#define PS2_DAT        8  //14    
#define PS2_CLK        11  //17
#define PS2_SEL        10  //16
#define PS2_CMD        9  //15
#define pressures   false
#define rumble      false

PS2X ps2x;

double cRf = 2;
double cLf = 2;
double cB = 2;
double k = 0.01;

int motRFf=22; //motor right front forward
int motRFb=23; //motor right front backward
int motRFp=5; //motor right front pwm

int motLFf=24; //motor left front forward
int motLFb=25; //motor left front backward
int motLFp=6; //motor left front pwm

int motBf=29; //motor back forward
int motBb=28; //motor back backward
int motBp=7; //motor back pwm

void setup()
{
  Serial.begin (9600);
  
  int error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

   pinMode(motRFf,OUTPUT);
   pinMode(motRFb,OUTPUT);
   pinMode(motRFp,OUTPUT);

   pinMode(motLFf,OUTPUT);
   pinMode(motLFb,OUTPUT);
   pinMode(motLFp,OUTPUT);

   pinMode(motBf,OUTPUT);
   pinMode(motBb,OUTPUT);
   pinMode(motBp,OUTPUT);  
}

void loop() 
{  
  ps2x.read_gamepad();
  
  int x =( ps2x.Analog(PSS_LX)-128);
  int y = -(ps2x.Analog(PSS_LY)-127);
  int r = (ps2x.Analog(PSS_RX)-128);

  if(ps2x.NewButtonState() && ps2x.Button(PSB_R1))
       k = k*(-1);
  if(ps2x.NewButtonState() && ps2x.Button(PSB_CIRCLE))
       cRf = cRf+k ;
  if(ps2x.NewButtonState() && ps2x.Button(PSB_SQUARE))
       cLf = cLf+k;
   if(ps2x.NewButtonState() && ps2x.Button(PSB_CROSS))
       cB  = cB+k;

   Serial.print(cRf);   
   Serial.print(" ");
   Serial.print(cLf);
   Serial.print(" ");
   Serial.print(cB); 
   Serial.print(" ");   
  
   double motRF = cRf * (0.57735*float(y) - 0.33333*float(x) - 0.33333*float(0));   //multiplied by cRF to map upto 255
   double motLF = cLf * (0.57735*float(y) + 0.33333*float(x) + 0.33333*float(0));   //multiplied by cLF to map upto 255
   double motB =  cB  * (0.00000*float(y) + 0.66666*float(x) - 0.33333*float(0));   //multiplied by cB to map upto 255                             //multiplied by 2 to map upto 255

   if(motRF<30 && motRF>-30)
     motRF=0;
    if(motLF<30 && motLF>-30)
       motLF=0;
    if(motB<30 && motB>-30)
       motB=0;
       
    motRF=constrain(motRF,-255,255);
    motLF=constrain(motLF,-255,255);
    motB=constrain(motB,-255,255);
    
    if(motRF>=-30 && motRF<=30)        //these are to prevent stalling of motors
      motRF=0;
    if(motLF>=-30 && motLF<=30)
      motLF=0;
    if(motB>=-30 && motB<=30)
      motB=0;    
        
    setMotorSpeed(motLFf,motLFb,motLFp,motLF);
    setMotorSpeed(motRFf,motRFb,motRFp,motRF);
    setMotorSpeed(motBf,motBb,motBp,motB);
   
}

void setMotorSpeed(int dirPin1,int dirPin2,int pwmPin,int pwmVal)
  {        
    if(pwmVal<0)
    {
      digitalWrite(dirPin1,HIGH);
      digitalWrite(dirPin2,LOW);
      analogWrite(pwmPin,-pwmVal);
    }      
    else if (pwmVal>0)
    {
      digitalWrite(dirPin1,LOW);
      digitalWrite(dirPin2,HIGH);
      analogWrite(pwmPin,pwmVal);
    }
    else
    {
      digitalWrite(dirPin1,HIGH);
      digitalWrite(dirPin2,HIGH);
      analogWrite(pwmPin,0);
     }
}
