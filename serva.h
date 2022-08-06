#ifndef SERVA_H
#define SERVA_H

#include <ESP32Servo.h>
#include "PPMReader.h"

const int servoPin_1 = 15;
const int servoPin_2 = 4;
const int servoPin_3 = 5;
const int servoPin_4 = 23;
const int antenaPin = 19;
const int enginePin = 18;

PPMReader ppmReader(antenaPin);
Servo servo_1;  
Servo servo_2;  
Servo servo_3;
Servo servo_4;
Servo engine;

int servo_control[8] = {1500,1500,1000,1500,1500,1500,1500,1500};
int servo_control_buf;

void init_control() {
  servo_1.attach(servoPin_1);  
  servo_2.attach(servoPin_2);  
  servo_3.attach(servoPin_3); 
  servo_4.attach(servoPin_4);
  engine.attach(enginePin);
}

void read_control() {
  for(int i = 0; i < 8; ++i){
    servo_control_buf = ppmReader.get(i);
    if(servo_control_buf>=1000 && servo_control_buf<=2000){
      servo_control[i] = servo_control_buf;    
    }
  }
}

void write_control(){
  servo_1.write(servo_control[0]);
  servo_2.write(servo_control[1]);
  /*servo_3.write(servo_control[4]);
  servo_4.write(servo_control[5]);*/
  engine.write(servo_control[2]);
}

#endif
