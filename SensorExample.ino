 /*Author: Naterads
Working on headtracking*/

#include "mpu6050dec.h"
#include "KalmanFilter.h"
#include <Wire.h>
#include <math.h>


mpuVals mVals;
mpuBias biasVals;
mpuAngles angleVals;

const int biasArrayLength = 500;
int bias[biasArrayLength];
kalmanFilter* kF_G_Z;//gyros z axis kalman filter
double estError = 10;
double estimate = 0;
double measurement = 0;

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println("InvenSense MPU-6050 ReWrite");
  Serial.println("May 2018");
  Wire.begin();
  Serial.println("Configuring registers");

  //configure MPU6050 registers
  //reset the sensor
  writeByte(MPU_powerMan,MPU_RESET);
  delay(500);
  writeByte(MPU_powerMan,0); //wakes up mpu6050
  writeByte(MPU_GYRO_CONFIG, MPU_FS500);//set the sensitivity of gyro
  writeByte(MPU_ACCEL_CONFIG, MPU_FS2G);//set sensitivity of accelerometer
  //writeByte(MPU_INTERRUPT_REGISTER, MPU_DATAREADY_INTERRUPT); //enables interrupts when new data is in sensors registers

  //configure the biases
  //Serial.println("Configuring biases");  
  //findBias();
  Serial.println("Find error in z measurement");
  Serial.print("Error: ");
  int16_t eMeas_Z = calcMeasurementError();
  Serial.println(eMeas_Z);
  kF_G_Z= new kalmanFilter(eMeas_Z, estError);
}

//the main loop being executed by microcontroller
void loop() {
  //take a measurement
  readValues();
  measurement = (double)mVals.gyro_z;
  //use the kalman filter to get an estimate from the raw data
  estimate = kF_G_Z->filter(measurement);
  //print the results to the screen
  Serial.print("Meas: ");
  Serial.print(mVals.gyro_z);
  Serial.print("\tEst: ");
  Serial.println(estimate);
}

//calculates the bias on each axis
void findBias(){
  Serial.print("gx: ");//roll
  biasVals.gyro_x = biasHelper(GYRO_XOUT_H);
  Serial.print("gy: ");//pitch
  biasVals.gyro_y = biasHelper(GYRO_YOUT_H);
  Serial.print("gz: ");//yaw
  biasVals.gyro_z = biasHelper(GYRO_ZOUT_H);
  Serial.print("aroll: ");
  readValues();
  biasVals.accel_roll = accel_roll();
  Serial.println(biasVals.accel_roll);
  Serial.print("apitch: ");
  biasVals.accel_pitch = accel_pitch();
  Serial.println(biasVals.accel_pitch);
//accelerometer does not have a bias, but will have bias angles for pitch and angle for the starting position
  //to do calculate pitch and roll bias values
}

//finds pitch from accelerometer readings
double accel_pitch(){
  double yd = (double)mVals.accel_y, zd = (double)mVals.accel_z;
  double acc_pitch = acos(zd/(sqrt(yd*yd + zd*zd)));//in radians
  return acc_pitch*180/M_PI; //converts to degrees
}

//finds roll from accelerometer readings
double accel_roll(){
  double xd = (double)mVals.accel_x, zd = (double)mVals.accel_z;
  double accel_roll = acos(zd/(sqrt(xd*xd + zd*zd)));//in radians
  return accel_roll*180/M_PI;//converts to degrees
}


//called by the findbias function is a helper method
int16_t biasHelper(uint8_t addr){
  long total = 0;
  for(int i = 0; i<biasArrayLength;i++){
    total += readValueRegister(addr);
  }
  total = total/biasArrayLength;
  Serial.println(total);
  return (int16_t)total;
}



//computes the translation angles from the values writen to vals registers
void computeAngles(){
  angleVals.a_pitch = accel_pitch();
  angleVals.a_roll = accel_roll();
  //todo
  angleVals.g_pitch = 0;
  angleVals.g_roll = 0;
  angleVals.g_yaw = 0;
}

//pulls all the raw values from the sensor and puts them in the referenced value
void readValues(){
  Wire.beginTransmission(MPU_address);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_address,14,1);
  mVals.accel_x = Wire.read() << 8 | Wire.read();
  mVals.accel_y = Wire.read() << 8 | Wire.read();
  mVals.accel_z = Wire.read() << 8 | Wire.read();
  mVals.temp = Wire.read() << 8 | Wire.read();//temperature
  mVals.gyro_x = Wire.read() << 8 | Wire.read();
  mVals.gyro_y = Wire.read() << 8 | Wire.read();
  mVals.gyro_z = Wire.read() << 8 | Wire.read();
}


//integrates from first principles
/*void angleInteg(double dps){
  t2 = millis();
  x2 = dps;
  angle += x2 * ((double)t2/1000.0-(double)t1/1000.0) + 0.5* (x2 - x1)*((double)t2/1000.0-(double)t1/1000.0);
  t1 = t2;
  x1 = x2;
}*/

//reads a single value from two part register
int16_t readValueRegister(uint8_t addr){
  Wire.beginTransmission(MPU_address);
  Wire.write(addr);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_address,2,1);
  int16_t mpu_val = Wire.read() << 8 | Wire.read();
  return mpu_val;
}

//reads raw value from register
uint8_t readRegister(uint8_t addr){
  Wire.beginTransmission(MPU_address);
  Wire.write(addr);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_address,1);
  uint8_t Val = Wire.read();
  return Val;
}

//writes a single byte to a register
void writeByte(uint8_t addr, uint8_t value){
  Wire.beginTransmission(MPU_address);
  Wire.write(addr);
  Wire.write(value);
  Wire.endTransmission(true);
}

//finds the error from a single sensor +- a certain value
int16_t calcMeasurementError(){
  const unsigned int testMinMax = 2000;
  readValues();
  int16_t temp = mVals.gyro_z;
  int16_t mini = temp, maxi = temp;
  //find maximum and minimum values over 2000 iterations
  for(int i = 0; i<testMinMax; i++){
    readValues();
    temp = mVals.gyro_z;
    if(temp > maxi){
      maxi = temp;
    }
    if(temp < mini){
      mini = temp;
    }
  }
 //find the middle to find the error in measurement
 temp = abs(maxi - mini);
 return temp/2;
}

