#include <PS2X_lib.h>
//read in function ps control
#define PS2_DAT        8  //14    
#define PS2_CLK        11  //17
#define PS2_SEL        10  //16
#define PS2_CMD        9  //15
#define pressures   false
#define rumble      false

PS2X ps2x;
int count=1;
//PS2 definition ends

//to calculate kp find maximum angle
double p,i=0,d;              //kp=(y*0.57735)/(5*sqrt(3)*0.33)  y is the 
double  kp=58 ,ki=0,kd=0; //kp=16.35,ki=0.0018 , kd=16;
int ti, tf, dt;               //ti=time_initial, tf=time_final(current), dt=tf-ti
double prex=0, dx;          //x=weighted sum from all sensors, prex=previous value of x, dx=x-prex
//pid definition ends

double n=11;                      //n -number of sensors
int base=250;                 //base-value of analog sensor below which is white above which is non-white
int sensor[]={A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};//sensor pins
int ir[]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};//these are just some random values which get over-written
//sensor definiton ends

int motRFf=29; //motor right front forward
int motRFb=28; //motor right front backward
int motRFp=7; //motor right front pwm

int motLFf=23; //motor left front forward
int motLFb=22; //motor left front backward
int motLFp=5; //motor left front pwm

int motBf=25; //motor back forward
int motBb=24; //motor back backward
int motBp=6; //motor back pwm
//motor definitions end

void setup()
{
   for(int i=0;i<n;i++)            //loop for setting
   {                               //all ir pin modes
     pinMode(sensor[i],INPUT);     //to input
   }

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
   
   ti=millis(); 
   Serial.begin (9600);
}

void loop()
{
  tf=millis();   
  ps2x.read_gamepad();  
  if(ps2x.NewButtonState() && ps2x.Button(PSB_TRIANGLE))//if the triangle button is pushed
  {                                                     //state chnges between manual control
    count++;                                            // and automatic line follower
  }
//  if(ps2x.NewButtonState() && ps2x.Button(PSB_R3))//if the triangle button is pushed
//  {                                                     //state chnges between manual control
//    count=2;                                            // and automatic line follower
//  }
  
  if(count%2!=0)
  {
    psControl();
  }
  
  else if(count%2==0)
  {
    lineFollow();       
  }
  ti=tf;   
  Serial.println();
}

void psControl()
{
  int x =( ps2x.Analog(PSS_LX)-128);
  int y = -(ps2x.Analog(PSS_LY)-127);
  int r = (ps2x.Analog(PSS_RX)-128);

  if(x==4)
  x=0;
  if(y==-5)
  y=0;
  if(r==4)
  r=0;

  Serial.print("x=");
  Serial.print(x);
  Serial.print("\ty=");
  Serial.print(y);
  Serial.print("\tr=");
  Serial.print(r);

  double cRf = 2;//remember to make global
  double cLf = 2.2;
  double cB =  1.8;
  double k = 0.01;
  
  if(ps2x.NewButtonState() && ps2x.Button(PSB_R1))
       k = k*(-1);
  if(ps2x.NewButtonState() && ps2x.Button(PSB_CIRCLE))
       cRf = cRf+k ;
  if(ps2x.NewButtonState() && ps2x.Button(PSB_SQUARE))
       cLf = cLf+k;
   if(ps2x.NewButtonState() && ps2x.Button(PSB_CROSS))
       cB  = cB+k;  
       
    double motRF = cRf * (0.57735*float(y) - 0.33333*float(x) - 0.33333*float(r));   //multiplied by 4 to map upto 255
    double motLF = cLf * (0.57735*float(y) + 0.33333*float(x) + 0.33333*float(r));   //multiplied by 4 to map upto 255
    double motB =  cB  * (0.00000*float(y) + 0.66666*float(x) - 0.33333*float(r));   //multiplied by 2 to map upto 255

    if(motRF>=-30 && motRF<=30)
      motRF=0;
    if(motLF>=-30 && motLF<=30)
      motLF=0;
    if(motB>=-30 && motB<=30)
      motB=0;

      
    Serial.print("\tmotRF=");
    Serial.print(motRF);
    Serial.print("\tmotLF=");
    Serial.print(motLF);
    Serial.print("\tmotB=");
    Serial.print(motB);
    
    motRF=constrain(motRF,-255,255);
    motLF=constrain(motLF,-255,255);
    motB=constrain(motB,-255,255);        
    
    setMotorSpeed(motLFf,motLFb,motLFp,motLF);
    setMotorSpeed(motRFf,motRFb,motRFp,motRF);
    setMotorSpeed(motBf,motBb,motBp,motB); 
}

void lineFollow()
{  
    double vel = 100 ;                    //takes velocity as input from ps2 controller left analog stick in the y direction
    vel=100;
    int activeSensors=irRead();               //stores the ir values in ir[] and also return number of sensors detecting the line
    double x=weightedSum(activeSensors);      //returns weighted sum of all the sensors 
    
    dt=(tf-ti);                                 //dt=change inn time
    dx= x-prex;                               //dx=change in error - x
    if(activeSensors==0)                      //this is the safe case              
    {                                         //if no sensor is on the line
      x=-prex;                                //direction of rotation of the wheels changes  
    }                                         //bringing the bt back to the line    
    double error=pid(x,dx,dt);                //implements pid and returns the final error value
    
//    if(activeSensors>4)                       //case taken for extra staggering lines
//     error=0;   

//    Serial.print("\t error=");
//    Serial.print(error);  
    motorsWrite(error,ir,vel);                //maps the error values appropriate to the input velocity and writes in the motors

    if(activeSensors>0)               //changes value only if- atleast one sensor is on the line above 
    {
       prex=x;
    }
}

int irRead()
{
  int activeSensors=0;
// Serial.print("Sensor Values ");
  for(int i=0;i<n;i++)
  {
//      ir[i]= analogRead(sensor[i]);//for analog sensor
//      Serial.print(ir[i]);
//      Serial.print(" ");
//      if(ir[i]<base)               //..  
//        ir[i]=0;                   //..
//      else if(ir[i]>base)          //..
//         ir[i]=1;                  //till here
      ir[i]=!digitalRead(sensor[i]);       //for digital sensor           
//      Serial.print(ir[i]);
//      Serial.print(" ");
      if(ir[i]==0)
       activeSensors++;
  }
 // Serial.print(activeSensors);
  
  return activeSensors;
}

double weightedSum(int activeSensors)
{
  double x=0; 
  for(int i=0;i<n;i++) //loop for calculating weighted sum
  {
    x = x + (((double)i-(n/2)+0.5)*ir[i]);//weight difference =1
   // x = x + (((double)i-(n/2)+0.5)*ir[i]);//weight differnce =0.5
  }

/*Reason for dividing by active sensors
  If 2 or more sensors detect then according to above loop the weigtage just gets summed up to give a greater value
  However it should take an intermediate value of the sensors - as, if multiple sensors detect the line it implies that the 
  line lies somewhere intermediate to these sensors, thus should take an intermediate weight
  By dividing by no. of active sensors the I just take average of the weights of these sensors,thus following the above explanation 
*/
            
  if(activeSensors==0)  //as division takes place as it is in the final step 
  {                     //and we dont want to divide by zero
    x=0;                //hence this if statement
    activeSensors=1;
  }
  //Serial.print("\t Weight=");
  x=x/activeSensors;
  //Serial.print(x);
  //Serial.print("\t");
  return(x);

}

double pid(double x,double del_x,double del_t)
{  
  p=kp*x;  
  i= i + ki*x;
  if(x==0)
   i=0;
  d=kd*del_x/(del_t);
  double error=p + i + d;
//  Serial.print("p=");
//  Serial.print(p);
//  Serial.print("\t i=");
//  Serial.print(i);
//  Serial.print("\t d=");
//  Serial.print(d);
//  Serial.print("\t error=");
//  Serial.print(error);   
  return(error);
}
 
void motorsWrite(double error , int irVal[] ,double vel)
{
  /*Y = sin(120)*f1 + sin(60)*f2 + sin(360)*f3
    X = cos(120)*f1 + cos(60)*f2 + cos(360)*f3
    w =      -1*f1 +        1*f2 +       -1*f3
    Solve these equations simultaneously to get values of f1,f2,f3 in terms of X,Y,w
    f1 = motRF
    f2 = motLF
    f3 = motB
  */
  double cRf=1,cLf=1.1,cB=0.9;
  double x = -error;
  if(error>(kp*(n/2)-0.5-2) || error<(-(kp*(n/2)-0.5-2)) )
  vel=170;
  else 
  vel=250;

  //r=map(r,-(kp*((n/2)-0.5)),(kp*((n/2)-0.5)),-128,128);
  double motRF = cRf * (0.57735*float(vel) - 0.33333*float(0) - 0.33333*float(x));   //multiplied by 4 to map upto 255
  double motLF = cLf * (0.57735*float(vel) + 0.33333*float(0) + 0.33333*float(x));   //multiplied by 4 to map upto 255
  double motB =  cB  * (0.00000*float(vel) + 0.66666*float(0) - 0.33333*float(x));   //multiplied by 2 to map upto 255
    
    if(motRF<30 && motRF>-30)
     motRF=0;
    if(motLF<30 && motLF>-30)
       motLF=0;
    if(motB<30 && motB>-30)
       motB=0;

//  Serial.print("\t motRF=");
//  Serial.print(motRF);
//  Serial.print("\t motLF=");
//  Serial.print(motLF);
//  Serial.print("\t motB=");
//  Serial.println(motB);
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
    digitalWrite(dirPin2,HIGH);
    digitalWrite(dirPin1,LOW);
    analogWrite(pwmPin,pwmVal);
  }
  else
  {
    digitalWrite(dirPin1,HIGH);
    digitalWrite(dirPin2,HIGH);
    analogWrite(pwmPin,0);
  }
}
