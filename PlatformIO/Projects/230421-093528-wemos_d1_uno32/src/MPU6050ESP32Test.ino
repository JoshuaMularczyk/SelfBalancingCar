#include <MPU6050.h>      //MPU6050 library
#include <Wire.h>        //IIC communication library

MPU6050 mpu6050;     //Instantiate an MPU6050 object; name mpu6050
int16_t ax, ay, az, gx, gy, gz;     //Define three-axis acceleration, three-axis gyroscope variables

float Angle;   //angle variable
int16_t Gyro_x;   //Angular velocity variable

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
  Angle = -atan2(ay , az) * (180/ PI);           //Radial rotation angle calculation formula; the negative sign is direction processing
  Gyro_x = -gx / 131;              //The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
  Serial.print("Angle = ");
  Serial.print(Angle);
  Serial.print("   Gyro_x = ");
  Serial.println(Gyro_x);
}