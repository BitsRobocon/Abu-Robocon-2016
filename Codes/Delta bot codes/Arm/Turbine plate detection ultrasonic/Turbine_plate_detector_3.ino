int trig[3];       //Trigger pins for US sensor
int echo[3];       //Echo pins for US sensor
double senval[3];  //Store US sensor output
double senvar[3];  //Store variance of senval
float maxvar=0.1;  //Maximum variance allowed for centre

void setup() {
  
  for(int i=0;i<3;i++)
  {
    trig[i]=3+i;   //Define pins 3, 4, 5 for trigger
    echo[i]=6+i;   //Define pins 6, 5, 7 for echo
    pinMode(trig[i], OUTPUT);
    pinMode(echo[i], INPUT);
  }
  
  Serial.begin(9600);
  
}


void loop() {
  
  delay(0000);
  
  for(int i=0;i<3;i++)
    senval[i]=readUS(i);         //Store from function readUS
  for(int i=0;i<3;i++)
    senvar[i]=(3*senval[i]/(senval[0]+senval[1]+senval[2]))-1;   //Store variance
  
  if(senvar[0]<maxvar&&senvar[0]>-maxvar&&senvar[1]<maxvar&&senvar[1]>-maxvar&&senvar[2]<maxvar&&senvar[0]>-maxvar)
    Serial.print("C");
  
  else if(senvar[0]<-maxvar&&senvar[1]>0&&senvar[2]>0)
    Serial.print("D R");
  else if(senvar[1]<-maxvar&&senvar[2]>0&&senvar[0]>0)
    Serial.print("U");
  else if(senvar[2]<-maxvar&&senvar[0]>0&&senvar[1]>0)
    Serial.print("D L");
  
  else if(senvar[0]>maxvar&&senvar[1]<0&&senvar[2]<0)
    Serial.print("U L");
  else if(senvar[1]>maxvar&&senvar[2]<0&&senvar[0]<0)
    Serial.print("D");
  else if(senvar[2]>maxvar&&senvar[0]<0&&senvar[1]<0)
    Serial.print("U R");
  else 
    Serial.print("!!!");
    
  Serial.print("\t");
  Serial.print(senvar[0]);Serial.print("\t");
  Serial.print(senvar[1]);Serial.print("\t");
  Serial.print(senvar[2]);Serial.println("\t");
}

double readUS( int i )         //Define function
{
  digitalWrite(trig[i], LOW);
  delayMicroseconds(2);
  digitalWrite(trig[i], HIGH);
  delayMicroseconds(3);
  digitalWrite(trig[i], LOW);
    
  return (double)( pulseIn(echo[i], HIGH) )/(double)10;
}
