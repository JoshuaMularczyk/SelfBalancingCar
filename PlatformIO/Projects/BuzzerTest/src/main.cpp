#include <Arduino.h>
const int buz = 23;    // set the buzzer pin 
const int btn = 18;    //set the button pin 
int button;    //button variable
void buzzer(void);

void setup() 
{
  pinMode(btn,INPUT);       //set to INPUT state
  pinMode(buz,OUTPUT);       //set to OUTPUT state 
}

void loop() 
{
  button = digitalRead(btn);       //assign the button value to variable button
  if(button == 0)    //if press the button
  {
    delay(10);    //delay time 
    if(button == 0)     //judge again, if the button is pressed
    {
      buzzer();  // execute the subfunction of buzzer
    }
  }
  else        // button not pressed
  {
    digitalWrite(buz,LOW);    // buzzer not sounds
  }
}

//buzzer makes tick sound
void buzzer()      
{
    for(int i=0;i<50;i++)
    {
    digitalWrite(buz,HIGH);
    delay(1);
    digitalWrite(buz,LOW);
    delay(1);
    }
    delay(50);
    for(int i=0;i<50;i++)
    {
    digitalWrite(buz,HIGH);
    delay(1);
    digitalWrite(buz,LOW);
    delay(1);
    }
}