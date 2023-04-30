#include <Arduino.h>

#include <ESP32Encoder.h>

ESP32Encoder encoder;
ESP32Encoder encoder2;

//TB6612 pins
const int right_R1=12;    
const int right_R2=19;
const int PWM_R=5;
const int left_L1=14;
const int left_L2=27;
const int PWM_L=13;

hw_timer_t *My_timer = NULL;
volatile int64_t delta_omega_left = 0;
volatile int64_t delta_omega_right = 0;
volatile int64_t omega_left = 0;
volatile int64_t omega_right = 0;
int64_t new_omega_right, new_omega_left;

void IRAM_ATTR onTimer(){
new_omega_left = encoder.getCount();
delta_omega_left = new_omega_left - omega_left;
new_omega_right = encoder2.getCount();
delta_omega_right = new_omega_right - omega_right;
omega_left = new_omega_left;
omega_right = new_omega_right;
}

void setup(){
	
	Serial.begin(115200);

  pinMode(right_R1,OUTPUT);     // set all TB6612pins to OUTPUT
  pinMode(right_R2,OUTPUT);
  pinMode(PWM_R,OUTPUT);
  pinMode(left_L1,OUTPUT);
  pinMode(left_L2,OUTPUT);
  pinMode(PWM_L,OUTPUT);

  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 100000, true);  //Millle number is the sample time in uS.
  timerAlarmEnable(My_timer); //Just Enable

	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors=UP;

	// use pin D5 and D2 for the first encoder
	encoder.attachHalfQuad(D5, D2);
	// use pin D3 and D4 for the second encoder (Order matters)
	encoder2.attachHalfQuad(D3, D4); 

	// clear the encoder's raw count and set the tracked count to zero
	encoder2.clearCount();
	Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));
}

void loop(){
  //Write to motor pins
  digitalWrite(right_R1,LOW);
  digitalWrite(right_R2,HIGH);
  digitalWrite(left_L1,LOW);
  digitalWrite(left_L2,HIGH);
  analogWrite(PWM_R,100);   // write into PWM value 0~255（speed）
  analogWrite(PWM_L,100);

	// Loop and read the count
	Serial.println("Encoder count = " + String((int32_t)encoder.getCount()) + " " + String((int32_t)encoder2.getCount()));
  Serial.println("Speed " + String((int32_t)delta_omega_left) + " " + String((int32_t)delta_omega_right));
  //Serial.println("Speed_right " + String((int32_t)delta_omega_right));
	delay(100);
}