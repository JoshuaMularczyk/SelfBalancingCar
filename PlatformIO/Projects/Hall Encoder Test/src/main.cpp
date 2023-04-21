#include <Arduino.h>
//TB6612 pins
const int right_R1=12;    
const int right_R2=19;
const int PWM_R=5;
const int left_L1=14;
const int left_L2=27;
const int PWM_L=13;

const int PinA_left = 16;    // set the left motor’s pulse pin to D5
const int PinA_right = 17;    //set the right motor’s pulse pin to D4

int times=0,newtime=0,d_time=100;   // time, new time, time interval
int valA=0,valB=0,flagA=0,flagB=0;    //variable valA and valB for calculating the number of pulse

void setup() 
{
  Serial.begin(9600);
  
  pinMode(right_R1,OUTPUT);    // set the TB6612 pins to OUTPUT
  pinMode(right_R2,OUTPUT);
  pinMode(PWM_R,OUTPUT);
  pinMode(left_L1,OUTPUT);
  pinMode(left_L2,OUTPUT);
  pinMode(PWM_L,OUTPUT);

  pinMode(PinA_left,INPUT);    // set the pulse pin to INPUT
  pinMode(PinA_right,INPUT);

}

void loop() 
{
  //both motors turn forward
  digitalWrite(right_R1,HIGH);
  digitalWrite(right_R2,LOW);
  digitalWrite(left_L1,HIGH);
  digitalWrite(left_L2,LOW);
  analogWrite(PWM_R,100);   //write into PWM value 0~255（speed）
  analogWrite(PWM_L,200);

  newtime=times=millis();     //make newtime and times equal to the time the program runs to here 
  while((newtime-times)<d_time)    //if less than the setting d_time，always loop
  {
    if(digitalRead(PinA_left)==HIGH&&flagA==0)   // if detects HIGH
    {
      valA++;      //valA plus 1
      flagA=1;
    }
    if(digitalRead(PinA_left)==LOW&&flagA==1)    // if LOW
    {
      valA++;     //valA plus 1
      flagA=0;
    }
    
    if(digitalRead(PinA_right)==HIGH&&flagB==0)
    {
      valB++;
      flagB=1;
    }
    if(digitalRead(PinA_right)==LOW&&flagB==1)
    {
      valB++;
      flagB=0;
    }
    newtime=millis();        //newtime equals to the time the program runs to here
  }
  Serial.println(valA);      // print out the value of valA and B
  Serial.println(valB);
  valA=valB=0;             //set to 0
}