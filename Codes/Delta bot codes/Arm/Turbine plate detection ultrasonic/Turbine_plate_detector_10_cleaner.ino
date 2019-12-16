//case A, when all US sensors are in proper range
//case B, when 1/2 US sensors are off the box
//case C, when all US sensors are off the box

/* ULTRA SOUND */
int trig[3];                  //Trigger pins for US sensor
int echo[3];                  //Echo pins for US sensor
double senval[3][10]={0};     //Store US sensor output and past 9 values, 30 in total
double sendif[3];             //Store weighted difference of senval
double maxvar=0;              //Maximum variance allowed
double maxdif=0.1;            //Maximum weighted difference allowed for centre


/* LOCALIZSATION */
int countA[8]={0};                  //Count C, U, UR, DR, D, DL, UL, !! for case A
int countB[6]={0};                  //Count C, U, R, D, L, !! for case B
int nval2=30;                       //Number of times to increment countA,B[] before setting delx, dely
int counter=0;                      //Keep track of above
double mindist=15;                  //Switch between A and B
int delaytime=10;                   //
int curstate=0;                     //A=1, B=2
int delx=0, dely=0;                 //delta

void setup() {
  
  for(int i=0;i<3;i++)
  {
    trig[i]=3+i;   //Define pins 3, 4, 5 for trigger
    echo[i]=6+i;   //Define pins 6, 7, 8 for echo
    pinMode(trig[i], OUTPUT);
    pinMode(echo[i], INPUT);
  }
  
  Serial.begin(9600);
  
}


void loop() {
  
  delay(0000);
  calcVal();
  
  if ( max( max(senval[0][0],senval[1][0]) ,senval[2][0]) > mindist &&
       max( max(senval[0][1],senval[1][1]) ,senval[2][1]) > mindist &&
       max( max(senval[0][2],senval[1][2]) ,senval[2][2]) > mindist &&
       max( max(senval[0][3],senval[1][3]) ,senval[2][3]) > mindist
     )
  {
    delay(delaytime);
    Serial.print("B");
    countPosB();
    curstate=2;
  }
  
  
  else if ( min( min(senval[0][0],senval[1][0]) ,senval[2][0]) < mindist &&
            min( min(senval[0][1],senval[1][1]) ,senval[2][1]) < mindist &&
            min( min(senval[0][2],senval[1][2]) ,senval[2][2]) < mindist &&
            min( min(senval[0][3],senval[1][3]) ,senval[2][3]) < mindist
          )
  {
    delay(delaytime);
    Serial.print("A");
    countPosA();
    curstate=1;
  }
  
  else 
    curstate=0;
  
//****************************************************************************************************
  
  if(curstate==0)
  {
    delx=0; dely=0;
    zeroCountA(); zeroCountB(); counter=0;
  }
  
  else 
  {
    if(counter==nval2)
    {
      if(curstate==1) calcDelA();
      if(curstate==2) calcDelB();
      zeroCountA(); zeroCountB(); counter=0;
      printDel();
    }
    counter++;
  }
    
}


/**** FUNCTIONS ****/

//******************************************************************************************************************** Function to read US output

double readUS( int i )
{
  digitalWrite(trig[i], LOW);                  //Default delays are 2 and 5 microsec
  delayMicroseconds(5);
  digitalWrite(trig[i], HIGH);
  delayMicroseconds(5);
  digitalWrite(trig[i], LOW);
  return (double)( pulseIn(echo[i], HIGH) )/(double)60;
}

//******************************************************************************************************************** Function to update stack and calc sendif/sum/var

void calcVal (void)
{
  for(int i=0;i<3;i++)
  { 
    for(int j=8;j>=0;j--)
    senval[i][j+1]=senval[i][j];    //Update stack
    senval[i][0]=readUS(i);         //Store from function readUS
  }
  for(int i=0;i<3;i++)
  {
    sendif[i]=(3*senval[i][0]/(senval[0][0]+senval[1][0]+senval[2][0]))-1;   //Store weighted difference
  }
}

//******************************************************************************************************************** Function to zero all countA[]

void zeroCountA (void)
{
  for(int i=0; i<8; i++)
  countA[i]=0;
}

//******************************************************************************************************************** Function to zero all countB[]

void zeroCountB (void)
{
  for(int i=0; i<6; i++)
  countB[i]=0;
}

//******************************************************************************************************************** Function to print delx,y

void printDel()
{
  Serial.print("\tdelx is "); Serial.print(delx);
  Serial.print("  dely is "); Serial.println(dely);
}


//******************************************************************************************************************** Function to count positions and store in countA[], case A

void countPosA (void)
{
  if(sendif[0]<maxdif&&sendif[0]>-maxdif&&sendif[1]<maxdif&&sendif[1]>-maxdif&&sendif[2]<maxdif&&sendif[0]>-maxdif)
  {
    countA[0]++;
  }
  
  
  else if(sendif[0]<-maxdif&&sendif[1]>0&&sendif[2]>0)
  {
    countA[3]++;
  }
  else if(sendif[1]<-maxdif&&sendif[2]>0&&sendif[0]>0)
  {
    countA[1]++;
  }
  else if(sendif[2]<-maxdif&&sendif[0]>0&&sendif[1]>0)
  {
    countA[5]++;
  }
  
  
  else if(sendif[0]>maxdif&&sendif[1]<0&&sendif[2]<0)
  {
    countA[6]++;
  }
  else if(sendif[1]>maxdif&&sendif[2]<0&&sendif[0]<0)
  {
    countA[4]++;
  }
  else if(sendif[2]>maxdif&&sendif[0]<0&&sendif[1]<0)
  {
    countA[2]++;
  }
  else 
  {
    countA[7]++;
  }
}

//******************************************************************************************************************** Function, case B

void countPosB (void)
{
  if(sendif[0]<maxdif&&sendif[0]>-maxdif&&sendif[1]<maxdif&&sendif[1]>-maxdif&&sendif[2]<maxdif&&sendif[0]>-maxdif)
  {
    countB[0]++;
  }
  
  
  else if(sendif[0]<-maxdif&&sendif[1]>0&&sendif[2]>0)
  {
    countB[2]++;
  }
  else if(sendif[1]<-maxdif&&sendif[2]>0&&sendif[0]>0)
  {
    countB[1]++;
  }
  else if(sendif[2]<-maxdif&&sendif[0]>0&&sendif[1]>0)
  {
    countB[4]++;
  }
  
  
  else if(sendif[0]>maxdif&&sendif[1]<0&&sendif[2]<0)
  {
    countB[4]++;
  }
  else if(sendif[1]>maxdif&&sendif[2]<0&&sendif[0]<0)
  {
    countB[3]++;
  }
  else if(sendif[2]>maxdif&&sendif[0]<0&&sendif[1]<0)
  {
    countB[2]++;
  }
  else 
  {
    countB[5]++;
  }
}

//******************************************************************************************************************** Calc maxcountA

int maxCountA (void)
{
  return max ( countA[0], max ( countA[1], max ( countA[2], max ( countA[3], max ( countA[4], max ( countA[5], max ( countA[6],  countA[7] ) ) ) ) ) ) );
}

//******************************************************************************************************************** Calc maxcountB

int maxCountB (void)
{
  return max ( countB[0], max ( countB[1], max ( countB[2], max ( countB[3], max ( countB[4], countB[5] ) ) ) ) );
}

//******************************************************************************************************************** Calc delx,y for case A

void calcDelA (void)
{
  
  if (countA[0]==maxCountA()) {delx= 0; dely= 0;}
  if (countA[1]==maxCountA()) {delx= 0; dely= 1;}
  if (countA[2]==maxCountA()) {delx= 1; dely= 1;}
  if (countA[3]==maxCountA()) {delx= 1; dely=-1;}
  if (countA[4]==maxCountA()) {delx= 0; dely=-1;}
  if (countA[5]==maxCountA()) {delx=-1; dely=-1;}
  if (countA[6]==maxCountA()) {delx=-1; dely= 1;}
  if (countA[7]==maxCountA()) {delx= 0; dely= 0;}
  
}

//******************************************************************************************************************** Calc delx,y for case A

void calcDelB (void)
{
  
  if (countB[0]==maxCountB()) {delx= 0; dely= 0;}
  if (countB[1]==maxCountB()) {delx= 0; dely= 1;}
  if (countB[2]==maxCountB()) {delx= 1; dely= 0;}
  if (countB[3]==maxCountB()) {delx= 0; dely=-1;}
  if (countB[4]==maxCountB()) {delx=-1; dely= 0;}
  if (countB[5]==maxCountB()) {delx= 0; dely= 0;}
  
}
