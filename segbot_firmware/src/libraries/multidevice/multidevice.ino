/* -*- mode: C++ -*- */
/*********************************************************************
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
 *  Arduino sensor firmware for Segbot version 2.
 *
 *  This program is driven by timer events.  On a 30Hz cycle, it polls
 *  an array of devices, which each may send results in a serial
 *  message.
 */

#include "arduino_device.h"

// include each device from a separate "Arduino library"
#include "sonar.h"
#include "voltmeter.h"
//#include "imu.h"                        // ignore IMU

// Ugly hacks needed to circumvent Arduino build silliness:
#include <NewPing.h>                    // to resolve sonar reference
// ignore IMU for now
//#include <Wire.h>                       // to resolve IMU references
//#include "I2Cdev.h"
//#include "MPU9150Lib.h"
//#include "CalLib.h"
//#include <dmpKey.h>
//#include <dmpmap.h>
//#include <inv_mpu.h>
//#include <inv_mpu_dmp_motion_driver.h>
//#include <EEPROM.h>

// Number of devices with IMU:
//#define N_DEVICES 3                   ///< number of devices to poll
// without IMU:
#define N_DEVICES 2                   ///< number of devices to poll

#define LED_PIN 13                    ///< pin with LED attached.
#define POLL_INTERVAL 10              ///< poll interval in milliseconds

ArduinoDevice *devices[N_DEVICES];    ///< All the device handlers
unsigned long poll_timer;             ///< time of next poll cycle

/// Called once on start-up.
void setup() 
{
  Serial.begin(115200);                 // serial port baud rate
  pinMode(LED_PIN, OUTPUT);             // configure LED output

  // The first poll starts after 75ms, giving the Arduino time to
  // initialize.  Others follow at POLL_INTERVAL ms.
  poll_timer = millis() + 75;

  // allocate and initialize all attached devices
  devices[0] = new Voltmeter();
  devices[1] = new Sonar();
  //devices[2] = new Imu();               // ignore IMU
}

/// Called repeatedly in Arduino main loop, must complete in under 33ms.
void loop()
{
  if (millis() >= poll_timer)      // time for next device poll cycle?
    {
      poll_timer += POLL_INTERVAL;
      for (uint8_t dev = 0; dev < N_DEVICES; ++dev)
        {
          if (devices[dev]->check(POLL_INTERVAL))
            {
              // turn the LED on or off with each poll
              digitalWrite(LED_PIN, !digitalRead(LED_PIN));
              devices[dev]->poll();      // poll the device
            }
        }
    }
}
