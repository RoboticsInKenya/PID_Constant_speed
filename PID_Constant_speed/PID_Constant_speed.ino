//copyright reserve @ Robotics in Kenya
//For Subscribe to My Youtube Channe, Robotics In Kenya
//Reach Me:
// FaceBook: Robotics In Kenya
// Instagram: robotics_in_kenya
//Whatsapp: +254700670999
//email: roboticsinkenya@gmail.com
//This code utilizes PID to monitor Motor Speed
//and drive it at a constant speed iregadless of the Load 
//

#include <PID_v1.h>
const byte encoder0pinA = 2;
const byte encoder0pinB = 3;

int motorPinC1 = 8; 
int motorPinC2 = 9; 
const int enablePinC= 10; 


byte L_encoderpinAlast;
double l_PIDcount;
boolean Direction;
boolean result;

double Output, Input, Setpoint;

double Kp= 1.0, Ki=11.1, Kd= 0.02; //tune the Pid To
int error = 2;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
 
void setup()
{ 
   Serial.begin(9600);
   pinMode(motorPinC1, OUTPUT);   
   pinMode(motorPinC2, OUTPUT);  
   pinMode(enablePinC, OUTPUT);  
   
   Setpoint =22;  //Set the output value of the PID
   myPID.SetMode(AUTOMATIC);
   myPID.SetSampleTime(10);
  EncoderInit();
}
 
void loop()
{   
      advance();//Motor Forward
      Input=abs(l_PIDcount);
      result=myPID.Compute();
      //PID conversion is complete and returns 1
      if(result)
      {
        Serial.print("Pulse: ");
        Serial.print(l_PIDcount-10);
        l_PIDcount = 0; //Count clear, wait for the next count
        Serial.print("   Output  "); 
        Serial.println(Output);       
      }
       

}
 
void EncoderInit()
{
  Direction = true;//default -> Forward 
  pinMode(encoder0pinB,INPUT); 
  attachInterrupt(0, wheelSpeed, CHANGE);
}
 
void wheelSpeed()
{
  int lastState = digitalRead(encoder0pinA);
  if((L_encoderpinAlast == LOW) && lastState==HIGH)
  {
    int val = digitalRead(encoder0pinB);
    if(val == LOW && Direction)
    {
      Direction = false; //Reverse
    }
    else if(val == HIGH && !Direction)
    {
      Direction = true;  //Forward
    }
  }
  L_encoderpinAlast = lastState;
 
  if(!Direction)  l_PIDcount++;
  else  l_PIDcount--;

}
void advance()//Motor Forward
{
     digitalWrite(motorPinC1,HIGH);
     digitalWrite(motorPinC2,LOW);
     
     analogWrite(enablePinC,Output);
}
