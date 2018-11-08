/*
    mou6000_spi_ros.ino
    Author: Numai-san
*/

//------------------------------------------------------------------------------
// Includes

#include "MPU6000.h"
#include <SPI.h>    // required by MPU6000::cpp

//------------------------------------------------------------------------------
// Definitions
#define SENSOR_TOPIC_RATE    100 //Hz
#define SPI_CLOCK 8000000  // 8MHz clock works.
#define SS_PIN   10 
#define INT_PIN  2

//------------------------------------------------------------------------------
// Variables
int data_ready = false;

uint8_t teapot[28] = { '$', 0x03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

//------------------------------------------------------------------------------
// Functions

void setup() {
    // Init Serial for use by Send.cpp and Receive.cpp
    Serial.begin(115200);
    // Init modules
    MPU6000::init();
    // Attach interrupt
    pinMode(INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), dataReadyCallback, RISING);
}

void dataReadyCallback()
{
  data_ready = true;
}

void loop() {
  while(data_ready == false and digitalRead(INT_PIN)==LOW){}
  MPU6000::read();
  //Quaternion (fixed)
  teapot[2] = 0x40;
  teapot[3] = 0x00;
  teapot[4] = 0x00;
  teapot[5] = 0x00;
  teapot[6] = 0x00;
  teapot[7] = 0x00;
  teapot[8] = 0x00;
  teapot[9] = 0x00;
  
  teapot[10] = (int8_t)MPU6000::sensors.gyrXByteH; //Gyro X H
  teapot[11] = (int8_t)MPU6000::sensors.gyrXByteL; //Gyro X L
  teapot[12] = (int8_t)MPU6000::sensors.gyrYByteH; //Gyro Y H
  teapot[13] = (int8_t)MPU6000::sensors.gyrYByteL; //Gyro Y L
  teapot[14] = (int8_t)MPU6000::sensors.gyrZByteH; //Gyro Z H 
  teapot[15] = (int8_t)MPU6000::sensors.gyrZByteL; //Gyro Z L
    
  teapot[16] = (int8_t)MPU6000::sensors.accXByteH; //Acc X H
  teapot[17] = (int8_t)MPU6000::sensors.accXByteL; //Acc X L
  teapot[18] = (int8_t)MPU6000::sensors.accYByteH; //Acc Y H
  teapot[19] = (int8_t)MPU6000::sensors.accYByteL; //Acc Y L
  teapot[20] = (int8_t)MPU6000::sensors.accZByteH; //Acc Z H
  teapot[21] = (int8_t)MPU6000::sensors.accZByteL; //Acc Z L
  
  teapot[22] = 0x00;
  teapot[23] = 0x00;
  
  teapot[25]++;
  data_ready = false;
  Serial.write(teapot, 28);
  //delay(1000);
}


//------------------------------------------------------------------------------
// End of file
