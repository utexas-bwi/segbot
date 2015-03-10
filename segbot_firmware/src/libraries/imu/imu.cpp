/* -*- mode: C++ -*- */
/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (C) 2015, Jack O'Quin
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the authors nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** @file
 *
 *  Arduino-attached MPU-9150 inertial measurement unit for Segbot
 *  version 2.
 */

#include <Arduino.h>
#include <imu.h>

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>

#define DEVICE_TO_USE 0                 ///< use the device at 0x68
MPU9150Lib MPU;                         ///< the MPU object

///  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates
///  the sensor data and DMP output
#define MPU_UPDATE_RATE  (20)

///  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates
///  the magnetometer data MAG_UPDATE_RATE should be less than or
///  equal to the MPU_UPDATE_RATE
#define MAG_UPDATE_RATE  (10)

///  MPU_MAG_MIX defines the influence that the magnetometer has on
///  the yaw output.  The magnetometer itself is quite noisy so some
///  mixing with the gyro yaw can help significantly.  This option
///  gives a good mix of those values:
#define  MPU_MAG_MIX_GYRO_AND_MAG       10 ///< a good mix value 

///  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz
#define MPU_LPF_RATE   40

/** Constructor: sets poll period to 50 msecs (40 Hz). */
Imu::Imu(): ArduinoDevice(1000/MPU_LPF_RATE)
{
  Wire.begin();
  MPU.selectDevice(DEVICE_TO_USE);

  // start the device
  MPU.init(MPU_UPDATE_RATE,
           MPU_MAG_MIX_GYRO_AND_MAG,
           MAG_UPDATE_RATE,
           MPU_LPF_RATE);
}

/** Poll interface.
 *
 *  When this function is called periodically, it reads the battery
 *  voltage, sending it in a serial message.
 */
void Imu::poll(void)
{
  // only needed if device has changed since init but good form anyway:
  MPU.selectDevice(DEVICE_TO_USE);
  if (MPU.read())                  // get the latest data if ready yet
    {
      Serial.print("I");
      Serial.print(DEVICE_TO_USE);
      Serial.print(" accel ");

      // print the calibrated acceleration data
      MPU.printVector(MPU.m_calAccel);
      Serial.println();
  }
}
