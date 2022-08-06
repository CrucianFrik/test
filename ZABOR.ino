#include "all_data_1.h"

//----------------------------------------------
#include "MPU9250_9Axis_MotionApps41.h"
MPU9250 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
#endif
//----------------------------------------------


String ans;
const char sep =',';

TaskHandle_t Task1;


void setup() {
  init_piezo();
  if (init_bmp()){delay_piezo(100);}
  delay(100);
  if (init_mpu()){delay_piezo(100);}
  delay(650);
  if (init_sd()){delay_piezo(100);}
  init_control();
  //init_gps();
  delay_piezo(1000);
  xTaskCreatePinnedToCore(control,"Task1",10000,NULL,1,&Task1,0);

  //----------------------------------------------
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

  if (devStatus == 0) {
      Serial.println("Enabling DMP...");
      mpu.setDMPEnabled(true);

      Serial.println("Enabling interrupt detection (Arduino external interrupt 0)...");
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      Serial.println("DMP ready! Waiting for first interrupt...");
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      Serial.print("DMP Initialization failed (code ");
      Serial.print(devStatus);
      Serial.println(")");
  }
  //----------------------------------------------


}

void control( void * pvParameters ){
  while(true){
    read_control();
    write_control();
    vTaskDelay(1);
  } 
}

void loop() {
  //update_gps();
  update_mpu();
  ans = (
    "\n" +
    String(gyr_X()) + sep +
    String(gyr_Y()) + sep +
    String(gyr_Z()) + sep +
    
    String(acc_X()) + sep +
    String(acc_Y()) + sep +
    String(acc_Z()) + sep +
    
    String(mag_X()) + sep +
    String(mag_Y()) + sep +
    String(mag_Z()) + sep +

    String(alt()) + sep +
    
   // String(lat(),6) + sep + String(lng(),6) + sep +

    String(servo_control[0]) + sep +
    String(servo_control[1]) + sep +
    String(servo_control[2]) + sep +
    String(micros()));

    to_file(ans);

    //----------------------------------------------
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
    }
    OUTPUT_READABLE_YAWPITCHROLL
    //----------------------------------------------

  }
