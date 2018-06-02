#include "mpu6050dec.h"
#include <Wire.h>

mpuVals mVals;
double angle = 0;

//these change for different sensitivity values
const double inLow = -32768, inHigh = 32767, outLow = -500, outHigh = 500;
const double gyro_x_bias = 4.45;
const int filterSize = 20;
int filterPointer = 0;
double gyro_x_filter[filterSize];
unsigned long t1 = 0,t2 = 0, x1=0, x2=0; //for integration by first principles

void setup() {
  
  Serial.begin(9600);
  Serial.println("InvenSense MPU-6050 test");
  Serial.println("May 2018");

  filterInit();
  
  Wire.begin();
  writeByte(MPU_powerMan,0);
  writeByte(MPU_GYRO_CONFIG, MPU_FS500);//set the sensitivity of gyro
  
  Serial.print("Printing gyroscope sensitivity register: ");
  Serial.println(readRegister(MPU_GYRO_CONFIG));
  //Serial.println("Printing Gyroscope x-axis values");

  //setup interrupts
  
}

void filterInit(){
  for(int i = 0; i < filterSize; i++){
    gyro_x_filter[i] = 0;
  }
}

void loop() {
  mVals.gyro_x = readValueRegister(GYRO_XOUT_H);//get the raw value
  //angle = ((double)500/(double)32768)*(double)mVals.gyro_x; //make more precise
  angle = (((double)mVals.gyro_x - inLow)/(inHigh - inLow))*(outHigh - outLow) + outLow + gyro_x_bias;//when you want to make sure everything is a double
  gyro_x_filter[filterPointer] = angle;
  filterPointer = (filterPointer + 1) % filterSize; //first 10 readings should be ignored
  
  Serial.println();
  Serial.print(angleInteg());
  Serial.print(" deg, from raw value: ");
  Serial.println(mVals.gyro_x);
  //delay(10);
}

//averages to reduce some of the jitter
double aveFilter(){
  double total = 0;
  for(int i = 0; i<filterSize;i++){
    total += gyro_x_filter[i];
  }
  return total/filterSize;
}

//pulls the raw values from the sensor and puts them in the referenced value
void readValues(mpuVals* v){
    
}

//can be made more efficient
//integrates from first principles
double angleInteg(){
  t2 = millis();
  x2 = aveFilter();
  angle += x2 * ((double)t2/1000.0-(double)t1/1000.0) + 0.5* (x2 - x1)*((double)t2/1000.0-(double)t1/1000.0);
  t1 = t2;
  x1 = x2;
  return angle;
}

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
