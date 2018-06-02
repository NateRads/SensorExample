//includes the register definitions and a structure for reading values from sensor
#define MPU_address 0x68
#define MPU_powerMan 0x6B

//config registers
#define MPU_GYRO_CONFIG 0x1B

//values used to set range of sensors
#define MPU_FS500 0x08 //+/- 500 deg/sec 
#define MPU_FSMASK 0x18

//output registers
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

//Readings from mpu6050 gyro/accelerometer
struct mpuVals{
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
};

