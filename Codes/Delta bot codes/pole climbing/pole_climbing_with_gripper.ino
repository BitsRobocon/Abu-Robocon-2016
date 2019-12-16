//Solenoid valve climbing mekha-anism
//Normally closed
int u_g=1;//upper_gripperi
int l_g=1;//lower_gripper

int s5b3u=1;//solenoid_5_by_3_upper
int s5b3l=1;//solenoid_5_by_3_lower

int t_c=0;//time_current   //for all the gripping
int t_l=0;//time_last      //and expanding functions
int check=0;

int bump=1;//or the bump sensor
int echo=1;//for detecting distance from
int trig=1;//lower base

double height_last=0;
int cs=1;//count_stroke

int time_curr=0;//for height detecting 
int time_last=0;//ultrasonic sensor

void setup()
{
  pinMode(u_g,OUTPUT);
  pinMode(l_g,OUTPUT);
  pinMode(s5b3u,OUTPUT);
  pinMode(s5b3l,OUTPUT);

  pinMode(bump,INPUT);
  
  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);

  digitalWrite(u_g,HIGH);
  digitalWrite(l_g,HIGH);

  time_last=millis();
}

void loop() 
{
  time_curr=millis();
  int bump_val = digitalRead(bump);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  double  duration = pulseIn(echo, HIGH);
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  double  height_curr = duration / 29 / 2;
  double  height_differ=0;
  if((time_curr-time_last)%300>0)
   {
    height_differ=height_curr-height_last;
    time_last=time_curr;
   }
  
  if(height_differ<-10) // dont forget to use millis
  {
      digitalWrite(l_g,HIGH);// grip lower gripper
      digitalWrite(u_g,HIGH);// grip upper gripper     
  }
  else  if(height_differ>-10)
  {
    if(bump_val==0)
    {
      int t_c=millis();
      if(check==0)
        int t_l=t_c;
        
      int dt=t_c-t_l;
      if(cs%2==0)    
      {
        if(dt>0 && dt<200)       //equivalent to 200 ms delay
        {
          digitalWrite(u_g,HIGH);  // grip upper gripper
          check=1;
        }
        else if(dt>200 && dt<300)//equivalent to 100 ms delay 
          digitalWrite(l_g,LOW);   // un-grip lower gripper
        else if(dt>300 && dt<800)//equivalent to 500 ms delay
        {
          digitalWrite(s5b3l,HIGH);// compress vertical pneumatics
          digitalWrite(s5b3u,LOW);//Higher pressure down so pulls upwards
        }
        else if(dt>800)
        {
          digitalWrite(s5b3l,HIGH);// Stops the pneumatic at its current  
          digitalWrite(s5b3u,HIGH);//position after 500 ms delay
          check=0;
        }
      }
      else if(cs%2==1)
      {
        
        if(dt>0 && dt<200)       //equivalent to 200 ms delay
        {
          if(cs>1)
          digitalWrite(l_g,HIGH);// grip lower gripper
          check=1;
        } 
        else if(dt>200 && dt<300)//equivalent to 100 ms delay
          digitalWrite(u_g,LOW);// UN-grip upper gripper  
        else if(dt>300 && dt<800)//equivalent to 500 ms delay
        {  
          digitalWrite(s5b3l,LOW);// expand vertical pneumatics
          digitalWrite(s5b3u,HIGH);//Higher pressure up so expands 
        }
        else if(dt>800)
        {
          digitalWrite(s5b3l,HIGH);// Stops the pneumatic at its current
          digitalWrite(s5b3u,HIGH);//position after 500 ms delay
          check=0;
        }
      }
      cs++;
    }
    else if(bump_val==1)
    {      
      digitalWrite(s5b3l,HIGH);//stop all vertical pneumatics at position
      digitalWrite(s5b3u,HIGH);// by making both s5b3u and s5b3l as high      
      digitalWrite(l_g,HIGH);// grip lower gripper
      digitalWrite(u_g,HIGH);// grip upper gripper  
    }
  }
  height_last=height_curr;    
}
