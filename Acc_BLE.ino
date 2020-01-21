//-------------------------------------------------------------------------------
//  TinyCircuits ST BLE TinyShield UART Example Sketch
//  Last Updated 2 March 2016
//
//  This demo sets up the BlueNRG-MS chipset of the ST BLE module for compatiblity 
//  with Nordic's virtual UART connection, and can pass data between the Arduino
//  serial monitor and Nordic nRF UART V2.0 app or another compatible BLE
//  terminal. This example is written specifically to be fairly code compatible
//  with the Nordic NRF8001 example, with a replacement UART.ino file with
//  'aci_loop' and 'BLEsetup' functions to allow easy replacement. 
//
//  Written by Ben Rose, TinyCircuits http://tinycircuits.com
//
//-------------------------------------------------------------------------------
#include <SPI.h>
#include <STBLE.h>
#include <Wire.h>
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"



//Debug output adds extra flash and memory requirements!
#ifndef BLE_DEBUG
#define BLE_DEBUG true
#endif

#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif

#define DISPLAY_INTERVAL 100

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

unsigned long displayInterval = 50;
unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

uint8_t ble_rx_buffer[21];
uint8_t ble_rx_buffer_len = 0;
uint8_t ble_connection_state = false;
#define PIPE_UART_OVER_BTLE_UART_TX_TX 0

#ifdef SERIAL_POR_MONITOR
  #define SerialMonitor SERIAL_PORT_MONITOR
#else
  #define SerialMonitor Serial
#endif

void setup() {
 SerialMonitor.begin(230400);
  // SerialMonitorInterface.begin(230400);
  int errcode;
  while (!SerialMonitor); //This line will block until a serial monitor is opened with TinyScreen+!
   Wire.begin();
   BLEsetup();
delay(15);
  imu = RTIMU::createIMU(&settings);                        // create the imu object

  SerialMonitorInterface.print("ArduinoIMU starting using device "); SerialMonitorInterface.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
    SerialMonitorInterface.print("Failed to init IMU: "); SerialMonitorInterface.println(errcode);
  }

  if (imu->getCalibrationValid())
    SerialMonitorInterface.println("Using compass calibration");
  else
    SerialMonitorInterface.println("No valid compass calibration data");

  lastDisplay = lastRate = millis();
  sampleCount = 0;

  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.

  fusion.setSlerpPower(0.02);

  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor

  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
  
}


void loop() {
  aci_loop();//Process any ACI commands or events from the NRF8001- main BLE handler, must run often. Keep main loop short.
  unsigned long now = millis();
  unsigned long delta;
 
 char tempx[15];
 char tempy[15];
 char tempz[15];
  if (imu->IMURead()) {                                // get the latest data if ready yet
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    sampleCount++;
    if ((delta = now - lastRate) >= 1000) {
      /*SerialMonitorInterface.print("Sample rate: "); SerialMonitorInterface.print(sampleCount);
        if (imu->IMUGyroBiasValid())
        SerialMonitorInterface.println(", gyro bias valid");
        else
        SerialMonitorInterface.println(", calculating gyro bias - don't move IMU!!");
      */
      sampleCount = 0;
      lastRate = now;
    }
    if ((now - lastDisplay) >= displayInterval) {
      lastDisplay = now;
      RTVector3 accelData = imu->getAccel();
  
      RTVector3 fusionData = fusion.getFusionPose();
      char sendBuffer[21] = " " ;
      uint8_t sendLength = 20;
      //displayAxis("Accel:", accelData.x(), accelData.y(), accelData.z());        // accel data
      //displayAxis("Gyro:", gyroData.x(), gyroData.y(), gyroData.z());            // gyro data
      //displayAxis("Mag:", compassData.x(), compassData.y(), compassData.z());    // compass data
      //displayDegrees("Pose:", fusionData.x(), fusionData.y(), fusionData.z());   // fused output
     // float x = (float)(accelData.x() );//* RTMATH_RAD_TO_DEGREE);
String x = (String) (accelData.x());
x.toCharArray(tempx, 5);
      //float y = (float)(accelData.y() );//* RTMATH_RAD_TO_DEGREE);
     String y = (String) (accelData.y());
y.toCharArray(tempy, 5);
   //   float z = (float)(accelData.z() );// * RTMATH_RAD_TO_DEGREE);
      String z = (String) (accelData.z());
z.toCharArray(tempz, 5);

      
//      sprintf(data, "My code is  %d.%02d", (int)x, (int)(x*100)%100);
//     SerialMonitorInterface.print(data);
      sprintf(sendBuffer, "%s %s %s ", tempx, tempy, tempz);
    
   
      SerialMonitorInterface.print("Send Buffer "); SerialMonitorInterface.println(sendBuffer);

      if (!lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)sendBuffer, sendLength))
      {
        SerialMonitorInterface.println(F("TX dropped!"));
      }
     // SerialMonitorInterface.println();


      SerialMonitorInterface.print(" A_x: ");  SerialMonitorInterface.print(x);
      SerialMonitorInterface.print(" A_y: ");  SerialMonitorInterface.print(y);
      SerialMonitorInterface.print(" A_z: ");SerialMonitorInterface.println(z);
    // SerialMonitorInterface.println(sendBuffer);     

    }
  }

}
