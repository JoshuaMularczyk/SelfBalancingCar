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
volatile int64_t omega_left = 0;
volatile int64_t omega_right = 0;
int64_t new_omega_right, new_omega_left;


int16_t ax, ay, az, gx, gy, gz;     //Define three-axis acceleration, three-axis gyroscope variables
float Angle;   //angle variable
int16_t Gyro_x;   //Angular velocity variable
///////////////////////Kalman_Filter////////////////////////////
float Q_angle = 0.001;  //Covariance of gyroscope noise
float Q_gyro = 0.003;   //Covariance of gyroscope drift noise
float R_angle = 0.5;    //Covariance of accelerometer
char C_0 = 1;
float dt = DT/1000.0;  //The value of dt is the filter sampling time.  (How do we know the loop takes 0.05 seconds?  Better fix this!)
long int ms_last;
float K1 = 0.05;  // a function containing the Kalman gain is used to calculate the deviation of the optimal estimate.
float K_0, K_1, t_0, t_1;
float angle_err;
float q_bias;  //gyroscope drift

float accelz = 0;
float angle;
float angle_speed;

float Pdot[4] = { 0, 0, 0, 0 };
float P[2][2] = { { 1, 0 }, { 0, 1 } };
float PCt_0, PCt_1, E;
//Function Delcarations
void IRAM_ATTR onTimer();
void Kalman_Filter(double angle_m, double gyro_m);

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
  encoder.clearCount();
	encoder2.clearCount();

}

void loop() {
  dt = (millis() - ms_last) / 1000.0;  //My attempt at fixing the dt problem
  ms_last = millis();
  //Calculate Angles
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC to get MPU6050 six-axis data  ax ay az gx gy gz
  Angle = -atan2(ay , az);                  //Radial rotation angle calculation formula; the negative sign is direction processing
  Gyro_x = -gx / 131;                                   //The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
	Kalman_Filter(Angle, Gyro_x);
  delay(DT-2);
}

void motorControl(int32_t Speed) {
  //Write to motor pins
  if (signbit(Speed)) {
    digitalWrite(right_R1,HIGH);
    digitalWrite(right_R2,LOW);
    digitalWrite(left_L1,HIGH);
    digitalWrite(left_L2,LOW);
    analogWrite(PWM_R,-Speed);   // write into PWM value 0~255（speed）
    analogWrite(PWM_L,-Speed);
  } else {
    digitalWrite(right_R1,LOW);
    digitalWrite(right_R2,HIGH);
    digitalWrite(left_L1,LOW);
    digitalWrite(left_L2,HIGH);
    analogWrite(PWM_R,Speed);   // write into PWM value 0~255（speed）
    analogWrite(PWM_L,Speed);
  }
}
void IRAM_ATTR onTimer(){
new_omega_left = encoder.getCount();
delta_omega_left = new_omega_left - omega_left;
new_omega_right = encoder2.getCount();
delta_omega_right = new_omega_right - omega_right;
omega_left = new_omega_left;
omega_right = new_omega_right;
}
void Kalman_Filter(double angle_m, double gyro_m) {
  angle += (gyro_m - q_bias) * dt;  //Prior estimate
  angle_err = angle_m - angle;

  Pdot[0] = dt * P[1][1] + Q_angle - P[0][1] - P[1][0];  //Differential of azimuth error covariance  ****Rob changed this!
  Pdot[1] = -P[1][1];
  Pdot[2] = -P[1][1];
  Pdot[3] = Q_gyro;

  P[0][0] += Pdot[0] * dt;  //The integral of the covariance differential of the prior estimate error
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;

  //Intermediate variable of matrix multiplication
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  //Denominator
  E = R_angle + C_0 * PCt_0;
  //Gain value
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;

  t_0 = PCt_0;  //Intermediate variable of matrix multiplication
  t_1 = C_0 * P[0][1];

  P[0][0] -= K_0 * t_0;  //Posterior estimation error covariance
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;

  q_bias += K_1 * angle_err;      //Posterior estimation
  angle_speed = gyro_m - q_bias;  //The differential value of the output value; work out the optimal angular velocity
  angle += K_0 * angle_err;       ////Posterior estimation; work out the optimal angle
}