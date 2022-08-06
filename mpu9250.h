#ifndef MPU9250_H
#define MPU9250_H
#include <Wire.h>
#include <MPU9250_WE.h>


MPU9250_WE myMPU9250 = MPU9250_WE();

xyzFloat acc;
xyzFloat gyr;
xyzFloat mag;

bool init_mpu() {
  #ifdef I2C
  #define I2C
  Wire.begin(); 
  #endif
  if(myMPU9250.init() && myMPU9250.initMagnetometer()){
    myMPU9250.autoOffsets();
    myMPU9250.enableGyrDLPF();
    myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
    myMPU9250.setSampleRateDivider(5);
    myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_2000);
    myMPU9250.setAccRange(MPU9250_ACC_RANGE_16G);
    myMPU9250.enableAccDLPF(true);
    myMPU9250.setAccDLPF(MPU9250_DLPF_6);
    myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
    return true;
  }
  return false;
}

void update_mpu(){
  acc = myMPU9250.getGValues();
  gyr = myMPU9250.getGyrValues();
  mag = myMPU9250.getMagValues();
}

float gyr_X(){
  return gyr.x;
}

float gyr_Y(){
  return gyr.y;
}

float gyr_Z(){
  return gyr.z;
}

float acc_X(){
  return acc.x;
}

float acc_Y(){
  return acc.y;
}

float acc_Z(){
  return acc.z;
}

float mag_X(){
  return mag.x;
}

float mag_Y(){
  return mag.y;
}

float mag_Z(){
  return mag.z;
}

#endif
