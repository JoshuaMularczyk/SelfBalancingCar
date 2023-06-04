#include <Arduino.h>
#include <ESP32Encoder.h>
#include <MPU6050.h>
#include <Wire.h> 
#include <cmath>

#define DT 10 // dt is milliseconds

ESP32Encoder encoder;
ESP32Encoder encoder2;
MPU6050 mpu6050;

//TB6612 pins
const int right_R1=12;    
const int right_R2=19;
const int PWM_R=5;
const int left_L1=14;
const int left_L2=27;
const int PWM_L=13;

//Encoder Speed Values
hw_timer_t *My_timer = NULL;
volatile int64_t delta_omega_left = 0;
volatile int64_t delta_omega_right = 0;
volatile int64_t delta2_omega_left = 0;
volatile int64_t delta2_omega_right = 0;
volatile int64_t omega_left = 0;
volatile int64_t omega_right = 0;
int64_t new_omega_right, new_omega_left, old_delta_omega_left, old_delta_omega_right;
float position = 0;
float positionk = 0;
float velocity = 0;
float velocityk = 0;
float u = 0;
int16_t ax, ay, az, gx, gy, gz;     //Define three-axis acceleration, three-axis gyroscope variables
float Angle;   //angle variable
int16_t Gyro_x;   //Angular velocity variable

int32_t PWMvoltage = 0;
float dt = DT/1000.0;  //The value of dt is the filter sampling time.  (How do we know the loop takes 0.05 seconds?  Better fix this!)
long int ms_last;
float phi = 0;
float anglek;
float angle_speed;
float angle_speedk;
float friction = 0;
float Pdot[4] = { 0, 0, 0, 0 };
float P[2][2] = { { 1, 0 }, { 0, 1 } };
float PCt_0, PCt_1, E;
//Function Delcarations
void Full_Order_Observer();
void Friction();
void motorControl();


void setup() {
  
 // Join the I2C bus
  Wire.begin();           //Join the I2C bus sequence
  Serial.begin(115200);   //open serial monitor and set the baud rate to 115200
  delay(1500);
  mpu6050.initialize();   //initialize MPU6050
  delay(2);

  pinMode(right_R1,OUTPUT);     // set all TB6612pins to OUTPUT
  pinMode(right_R2,OUTPUT);
  pinMode(PWM_R,OUTPUT);
  pinMode(left_L1,OUTPUT);
  pinMode(left_L2,OUTPUT);
  pinMode(PWM_L,OUTPUT);

	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors=UP;

	// use pin D5 and D2 for the first encoder
	encoder.attachHalfQuad(D5, D2);
	// use pin D3 and D4 for the second encoder (Order matters)
	encoder2.attachHalfQuad(D3, D4); 

	// clear the encoder's raw count and set the tracked count to zero
  encoder.clearCount();
	encoder2.clearCount();

}

void loop() {
  //omega_left/omega_right is position
  //delta_Omega_left/Delta_Omega_right is velocity
  dt = (millis() - ms_last) / 1000.0;  //My attempt at fixing the dt problem
  ms_last = millis();
  //Calculate Angles
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC to get MPU6050 six-axis data  ax ay az gx gy gz
  angle_speed = -gx / 7506.;                                 //The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
  ay = ay / 1600;                                       // actually forward/backwards acceleration
  az = az / 1600;                                       // actually vertical acceleration


  new_omega_left = encoder.getCount();
  delta_omega_left = (new_omega_left - omega_left) / dt;
  //delta2_omega_left = (new_delta_omega_left - delta_omega_left)*10;
  new_omega_right = encoder2.getCount();
  delta_omega_right = (new_omega_right - omega_right) / dt;
  //delta2_omega_right = (new_delta_omega_right - delta_omega_right)*10;
  omega_left = new_omega_left;
  omega_right = new_omega_right;

  phi = omega_left / 104.85;

  //Control systems code
  //step 1 convert to usable x
  
  //step 2 multiply k and x
  //PWMvoltage = -34.91 * position_left + 7.81 * velocity_left - 42.79 * angle - 0.20 * angle_speed;
  //PWMvoltage = -4152.3 * position_left + 2.9637 * velocity_left - 1144.3 * angle - 1857.4 * angle_speed;
  PWMvoltage = -377.9645 * position_left + 21.8051 * velocity_left - 46.645 * angle - 5.3885 * angle_speed;
  //step 3 convert u to pwm value
  //Friction();
  if (velocityk == 0.0 & old_delta_omega_left == 0.0 & PWMvoltage >= 0 ) {
    PWMvoltage += 14;
  } else if (velocityk == 0.0 & old_delta_omega_left == 0.0 & PWMvoltage < 0) {
    PWMvoltage -= 14;
  }

  Serial.print("PWM = ");
  Serial.println(PWMvoltage);
  //step 4 run motor control
  motorControl();
  delay(DT-2);

}

void motorControl() {
  //Write to motor pins
  if (PWMvoltage <= 0) {
    digitalWrite(right_R1,HIGH);
    digitalWrite(right_R2,LOW);
    digitalWrite(left_L1,HIGH);
    digitalWrite(left_L2,LOW);
    analogWrite(PWM_R,-PWMvoltage);   // write into PWM value 0~255（speed）
    analogWrite(PWM_L,-PWMvoltage);
  } else {
    digitalWrite(right_R1,LOW);
    digitalWrite(right_R2,HIGH);
    digitalWrite(left_L1,LOW);
    digitalWrite(left_L2,HIGH);
    analogWrite(PWM_R,PWMvoltage);   // write into PWM value 0~255（speed）
    analogWrite(PWM_L,PWMvoltage);
  }
}
// make sure to add in ky inside the parenthesis.
void Full_Order_Observer() {
positionk = positionk + dt * (-350 * positionk + velocityk + 5.5583 * phi -0.9382 * ay + 0.056935 * angle_speed);
velocityk = velocityk + dt * (1181.1 * positionk - 368.195 * velocityk - 88.522 * anglek + 4890.052 * angle_speedk + 2.5942 * phi - 0.91978 * ay - 0.81861 * angle_speed);
anglek = anglek + dt * (57.358 * velocityk - 100 * anglek - 3950 * angle_speedk + 1.8001 * phi -1.3340 * ay + 2.5485 * angle_speed);
angle_speedk = angle_speedk + dt * (-69534 * positionk - 4468.1 * velocityk - 4631.2 * anglek -1021.31 * angle_speedk -0.00995988 * phi + 17.388 * ay + 372.65 * angle_speed);
}
