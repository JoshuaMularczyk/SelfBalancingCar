#include <MPU6050.h>      //MPU6050 library
#include <Wire.h>        //IIC communication library

#define DT 10 // dt is milliseconds

MPU6050 mpu6050;     //Instantiate an MPU6050 object; name mpu6050
int16_t ax, ay, az, gx, gy, gz;     //Define three-axis acceleration, three-axis gyroscope variables

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
int32_t PWMvoltage = 0;

float accelz = 0;
float angle;
float angle_speed;
float friction = 0;
float Pdot[4] = { 0, 0, 0, 0 };
float P[2][2] = { { 1, 0 }, { 0, 1 } };
float PCt_0, PCt_1, E;


float Angle;   //angle variable
double Gyro_x;   //Angular velocity variable

void Kalman_Filter(double angle_m, double gyro_m);


void setup() 
{
  // Join the I2C bus
  Wire.begin();                            //Join the I2C bus sequence
  Serial.begin(115200);                       //open serial monitor and set the baud rate to 9600
  delay(1500);
  mpu6050.initialize();                       //initialize MPU6050
  delay(2);
}

void loop() 
{
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC to get MPU6050 six-axis data  ax ay az gx gy gz
  Angle = atan2(ay , az);           //Radial rotation angle calculation formula; the negative sign is direction processing
  Gyro_x = gx / 7506.;              //7506. The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
  Kalman_Filter(Angle, Gyro_x);
  Serial.print("Accely = ");
  Serial.print(ay);
  Serial.print("   Accelz = ");
  Serial.print(az); 
  Serial.print("   Angle = ");
  Serial.print(Angle);
  Serial.print("   Gyro_x = ");
  Serial.print(Gyro_x);
  Serial.print("   FilterAngle = ");
  Serial.print(angle);
  Serial.print("   FilterAngleSpeed = ");
  Serial.println(angle_speed);
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