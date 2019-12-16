#include <Servo.h>//33 10 100
#define cbi(sfr,bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr,bit) (_SFR_BYTE(sfr) |= _BV(bit))
int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;

int riverdetect_count = 0; // to count number of times the river is detected
int turn = 0; // turn at 'T'
int active = 0;
int i;  // variable for running loop
int river_detect = 0; // detect the river
int t = 0; // time for which bot is running in green color
int stp = 0; //variable used in line following at river
int sensorMid_1;
int sensorMid_2;
int sensorMid_3;
int sensorMid_4;
int sensor_top_left;
int sensor_top_right;
int left_sensor = 0;
//variables for pid
float Kp = 45;  //Proportionality variable                                              // Max deviation = 6-3.5 = 2.5 ||  180/2.5 = 72
float Ki = 0.0003;  // Integral                                         // previously 0.00015
float Kd = 0.00;  //Differential
float error = 0; //deviation from line
float previousError = 0;
float totalError = 0; // Integral of errors

//variables for ir sensor
int sensorFirst, sensorLast;                                //sensors for colour detection
int sensor[10] = {A0, A2, A3, A4, A5, A6, A7, A8, A9, A11}; // Pin assignment
int sensorReading[10] = { 0 };
int sensorReading_digital[10] = {0}; // Sensor reading array
float activeSensor = 0;                                     // Count active sensors
float totalSensor = 0;                                      // Total sensor readings
float avgSensor = 6.5;                                      // Average sensor reading
int ir_threshold[10] = {500,280,180,180,180,200,200,380,380,180};                                      // threshold for ir values to change to active sensor
int mp[] = {3, 2, 4, 5};
int readVal = A5;
//int sensor [] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
int a = 0;
int b = 0;
int c = 0;
int d = 0;

//variables for servo
Servo puppies;
int servo_pin = 8;                                          //attached pin no. for servo
int servo_def = 100;                                         //servo input for 90 degrees
int mapping_1 = 35;
int mapping_2 = 30;                                           //max angle change for servo
int pos_o = 0;                                              //servo angle before change
float pos = 0;                                              //initial servo angle

//variables for hill
bool T_turn = 0;                                            //detection of T turn at hill 3
int T_turn_angle = 38;                                      //Offset angle turn at T turn
int slope_lim = 650;
int green_cont = 1;                                         //continuity of green
int count_red = 0;                                          //count "not green" surfaces ( no. of red in arena)
int T_joint_cross = 0;                                     //counter after crossing T turn
int detect;   //detection of green surface

//variables for switch
int left=0;
int right=1;
int count_1=0;
int count_2=0;
int count_3=0;

//variables for river
int riv_ang_lim = 10;                                       //angle limit for opposite direction angle to next turn
int riv_ir_lim = 680;                                       //threshhold for raw line sensor values for river
int river_cross = 0;                                        //var for crossed level of river

int river_turn = 35;





void setup() {
  sbi(ADCSRA,ADPS2);
  sbi(ADCSRA,ADPS1);
  sbi(ADCSRA,ADPS0);
  Serial.begin(9600);
  puppies.attach(servo_pin);
  pinMode(13, INPUT);
  pinMode(10, INPUT);
  pinMode(9, INPUT);
  pinMode(12, INPUT);
  pinMode(11, INPUT);
  for (int i = 0; i < 10; i++)
  {
    pinMode(sensor[i], INPUT);
  }
  for (int i = 0; i < 4; i++)
  { pinMode(mp[i], OUTPUT);
  }
  pinMode(readVal, INPUT); // put your setup code here, to run once:

}

void loop() {
  // switch_read();
  lineFollow();
  Serial.println();
  //if (T_joint_cross == 0)
  detectColour();

  pos_o = pos;
  // put your main code here, to run repeatedly:

}
void switch_read()
{
  left = digitalRead(12);
  right = digitalRead(11);
  count_1 = digitalRead(9);
  count_2 = digitalRead(10);
  count_3 = digitalRead(13);
  if (count_3 != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (count_3 != buttonState) {
      buttonState = count_3;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        ledState = !ledState;
      }
    }
  }
  //Serial.print(left);
  //Serial.print(right);
  //Serial.print(count_1);
  //Serial.print(count_2);
  //Serial.print(count_3);

}
void lineFollow()
{
  ////Serial.print("       ");
////Serial.print(T_turn);
  if (T_turn == 1)                                                                            //just reached hill 3
  {
    if (right == 1)
    {
      pos = servo_def + T_turn_angle;
      Serial.print(pos);
      puppies.write(servo_def+T_turn_angle);
    }
    if (left == 1)
    {

     pos = servo_def - T_turn_angle;
      //Serial.print(pos);
      puppies.write(pos);
    }
    //Serial.print("i am done");
    // puppies.write(pos);
    /*if (pos > pos_o)
    {
      for (int i = pos_o; i < pos; i = i + 5)
      { puppies.write(i);
        delay(80);
      }
    }
    else if (pos < pos_o)
    {
      for (int i = pos_o; i > pos; i = i - 5)
      { puppies.write(i);
        delay(80);
      }
    }
    //Serial.println("i am done");
    */
    delay(1000);
    T_turn = 0;
    T_joint_cross =1;
  }
  else
  {
    activeSensor = readSensor1();
    if(left_sensor==0 || left_sensor==3)
    {
      Serial.print("following line");
      Serial.print(river_detect);
    previousError = error;                                                                    // save previous error for differential
    error = avgSensor - 5.5;                                                                    // Error (deviation from center)
    totalError += error;                                                                        // Cumulative error for integral
    if (sensorReading_digital == 0)
      pos = pos_o;
    else
      pos = (Kp * error) + (Kd * (error - previousError)) + (Ki * totalError); //Position calculation
      //Serial.print(pos);
    pos = map(pos, -157, 112, servo_def - mapping_1, servo_def + mapping_2);
    if (pos > pos_o)
        {
          for (int i = pos_o; i < pos; i = i + 5)
          {
            puppies.write(i);
            
            delay(40);
          }
        }
        else if (pos < pos_o)
        {
          for (int i = pos_o; i > pos; i = i - 5)
          { puppies.write(i);
            delay(25);
          }
        }
            
  }
    //print_river_pos();                                                                          //Print river position
   // Serial.print(pos);
     if(river_detect==0)
     river_detect = river(pos);
     else if(river_detect==1)
     //Serial.print("river detected");

//Serial.print(left_sensor)  ;   //saving previous servo input
    if (T_joint_cross == 1 && left_sensor<=2 && river_detect==1)
    { ////Serial.print(sensorReading[0]);
      if (sensorReading[0] < 400 && left_sensor == 1)
      {Serial.print("turning left");
        left_sensor = left_sensor + 1;
        pos = servo_def - 40;
        if (pos > pos_o)
        {
          for (int i = pos_o; i < pos; i = i + 5)
          {
            puppies.write(i);
            delay(0);
          }
        }
        else if (pos < pos_o)
        {
          for (int i = pos_o; i > pos; i = i - 5)
          { puppies.write(i);
            delay(0);
          }
        }
        delay(300);

        
      }
      if ( sensorReading[9] < 150 && left_sensor == 0)
      { left_sensor = left_sensor + 1;
        pos = servo_def + 40;
        
        if (pos > pos_o)
        {
          for (int i = pos_o; i < pos; i = i + 5)

          { puppies.write(i);
            delay(0);
          }
        }
        else if (pos < pos_o)
        {
          for (int i = pos_o; i > pos; i = i - 5)
          { puppies.write(i);
            delay(0);
          }
        }
        delay(300);
      }
      if ( sensorReading[9] < 150 && left_sensor == 2)
      { left_sensor = left_sensor + 1;
        pos = servo_def +10;
        if (pos > pos_o)
        {
          for (int i = pos_o; i < pos; i = i + 5)

          { puppies.write(i);
            delay(0);
          }
        }
        else if (pos < pos_o)
        {
          for (int i = pos_o; i > pos; i = i - 5)
          { puppies.write(i);
            delay(0);
          }
        }
        delay(100);
      }





    }                //print servo input
  }

}


float readSensor1(void)
{
  
  for (int i = 0; i < 10; i++)
  {
    if(T_joint_cross==0 && i==0)
    sensorReading[i]=1023;
    else if(T_joint_cross==0 && i==9)
    sensorReading[i]=1023;
    else
    sensorReading[i] = analogRead(sensor[i]);
    Serial.print(sensorReading[i]);
    Serial.print("\t");
    /*  if (i=0)
      {
      sensor_top_left=sensorReading[i];
      }
      if(i=9)
      {
      sensor_top_right=sensorReading[i];
      }*/
      
      
    if (sensorReading[i] <= ir_threshold[i])
      sensorReading_digital[i] = 1;
    else
      sensorReading_digital[i] = 0;

    //  //Serial.print(sensorReading[i]);
    //s  //Serial.print("\t");


    if (sensorReading_digital[i] == 1) {
      activeSensor += 1;
    }
    totalSensor += sensorReading_digital[i] * (i + 1);
  }
  active = activeSensor;
  if (activeSensor != 0)
  {
    avgSensor = totalSensor / activeSensor;
  }
  activeSensor = 0; totalSensor = 0;
  return activeSensor;
}


float readSensor2(void)
{


  for (int i = 1; i < 17; i++)
  {
    digitalWrite(mp[0], a);
    digitalWrite(mp[1], b);
    digitalWrite(mp[2], c);
    digitalWrite(mp[3], d);
    sensorReading[i - 1] = analogRead(readVal);
    //   //Serial.print(a);
    //   //Serial.print(b);
    //   //Serial.print(c);
    //   //Serial.print(d);
    //   //Serial.println();


    if (i % 8 == 0) a = !a;
    if (i % 4 == 0) b = !b;
    if (i % 2 == 0) c = !c;
    d = !d;

  }
  int temp[] = {1, 2, 3, 4};
  for (int j = 0; j < 4; j++)
  {
    temp[j] = sensorReading[8 + j];
  }
  for (int j = 0; j < 4; j++)
  {
    sensorReading[8 + j] = temp[3 - j];
  }


  for (int i = 0; i < 12; i++)
  {

    //Serial.print(sensorReading[i]);
    //Serial.print("\t");

    if (sensorReading[i] <= ir_threshold[i])
      sensorReading_digital[i] = 1;
    else
      sensorReading_digital[i] = 0;



    if (sensorReading_digital[i] == 1) {
      activeSensor += 1;
    }
    totalSensor += sensorReading_digital[i] * (i + 1);
  }

  if (activeSensor != 0)
  {
    avgSensor = totalSensor / activeSensor;
  }
  activeSensor = 0; totalSensor = 0;
  return activeSensor;
}

int detectColour()
{
  if (count_1 == 1)
  { count_red = 1;
    ////Serial.println("Hill 1");

  }
  if (count_2 == 1)
  { count_red = 2;
    // //Serial.println("Hill 2");


  }
  if (count_3 == 1)
  { count_red = 3;
    // //Serial.println("Highland");
    T_turn = 1;
  }

  sensorFirst = sensorReading[1];
  sensorLast = sensorReading[8];
 // //Serial.print(sensorFirst);
 // //Serial.print(sensorLast);

  if (T_joint_cross==0 && sensorFirst > slope_lim && sensorLast > slope_lim)
  {
    detect = 1;                                         //red detected

    if (green_cont == 1 && t > 10)
    {
      green_cont = 0;
      count_red = count_red + 1;
      //to count no. of times green surfaces/slopes that have been crossed
    }

    if (count_red == 3)
    {
      T_turn = 1;                                       //3 green surfaces crossed
    }
  }
  else
  {
    detect = 0;                                         //green detected

    if (green_cont == 1)
    {
      t = t + 1;                                        //continuity of green
    }
    else
    {
      green_cont = 1;                                   //start of green
      t = 0;
    }
  }

  //print_end__ir_sens();

  print_count_red();
}
void print_count_red()
{
  if (detect == 1)
  {
  Serial.print("red = ");
  }
  Serial.println(count_red);
}
int river(float pos)
{
  


  if (T_joint_cross == 0)
   { river_detect = 0;   
    //Serial.print("    ");
//Serial.print("not the river"); 
   }//T turn not crossed on hill 3
  else
  {
    //Serial.print(sensorReading[0]);
    //Serial.print('/t');
        //Serial.print(sensorReading[2]);
            //Serial.print('/t');
            //Serial.print(sensorReading[7]);
                //Serial.print('/t');
                //Serial.print(sensorReading[9]);
                //Serial.println();
               
//Serial.print(riverdetect_count);
    if (sensorReading[3] < 600 && sensorReading[5] < 600 && sensorReading[6] < 600 && sensorReading[9] < 600)
    {
      
      riverdetect_count = riverdetect_count + 1;
    Serial.print("river detected");
    
    
      if (riverdetect_count>3)
      {
        river_detect = 1;
      }
      else
      {
        river_detect = 0;
      }
    }
  }
  return river_detect;
}
