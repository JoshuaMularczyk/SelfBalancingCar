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
float accelx = 0;

int32_t PWMvoltage = 0;
float dt = DT/1000.0;  //The value of dt is the filter sampling time.  (How do we know the loop takes 0.05 seconds?  Better fix this!)
long int ms_last = 0;
float phi = 0;
float anglek = 0;
float angle_speed = 0;
float angle_speedk = 0;
float friction = 0;
//Function Delcarations
void Full_Order_Observer();
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
  accelx = ay / 1600.;                                       // actually forward/backwards acceleration


  omega_left = encoder.getCount();
  phi = omega_left / 104.85;

  //Control systems code
  //PWMvoltage = -34.91 * position_left + 7.81 * velocity_left - 42.79 * angle - 0.20 * angle_speed;
  //PWMvoltage = -4152.3 * position_left + 2.9637 * velocity_left - 1144.3 * angle - 1857.4 * angle_speed;
 
  PWMvoltage = 22360.67 * positionk + 107.42 * velocityk + 1431.13 * anglek + 634.75 * angle_speedk;
  //Friction();
  if (PWMvoltage > 255) {
    PWMvoltage = 255;
  }
  if (PWMvoltage < -255) {
    PWMvoltage = -255;
  }
  if (anglek > 0.785 || anglek < -0.785) {
    PWMvoltage = 0;
  }
/*if (PWMvoltage >= 0 & PWMvoltage <= 14) {
    PWMvoltage += 14;
  } else if (PWMvoltage < 0 & PWMvoltage >=-14) {
    PWMvoltage -= 14;
  }*/
  Full_Order_Observer();
  Serial.print("phi = ");
  Serial.print(phi);
  Serial.print("   accelx = ");
  Serial.print(accelx);
  Serial.print("   angle_speed = ");
  Serial.println(angle_speed);
  //Serial.print("positionk = ");
  //Serial.print(positionk);
  //Serial.print("   velocityk = ");
  //Serial.print(velocityk);
  //Serial.print("   anglek = ");
  //Serial.print(anglek);
  //Serial.print("   angle_speedk = ");
  //Serial.print(angle_speedk);
  Serial.print("   PWM = ");
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
positionk = positionk + dt * (-29.631 * positionk + 1.0113 * velocityk + 0.00787 * anglek + 0.0086555 * angle_speedk + 1 * phi + 13733 * accelx - 8667.1 * angle_speed);
velocityk = velocityk + dt * (-0.055348 * positionk - 91.737 * velocityk - 63.53 * anglek + 0.9707 * angle_speedk + 3.1275 * PWMvoltage + 1868 * phi - 10.762 * accelx - 87.717 * angle_speed);
anglek = anglek + dt * (0.09305 * positionk - 72.67 * velocityk - 50.44 * anglek - 1.033 * angle_speedk - 3140.7 * phi - 87.944 * accelx + 2.1075 * angle_speed);
angle_speedk = angle_speedk + dt * (7.609 * positionk - 7832.2 * velocityk - 1897.3 * anglek - 151.75 * angle_speedk - 183.9689 * PWMvoltage - 25.681 * phi - 35.806 * accelx + 148.96 * angle_speed);
}
