/* -*- mode: C++ -*-
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2013, Jose Bigio
*  Copyright (C) 2014, Jack O'Quin
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

#include <NewPing.h>

// Arduino digital control pins for triggering sonar pings.
#define TRIGGER_PIN1  30
#define TRIGGER_PIN2  28
#define TRIGGER_PIN3  26
#define TRIGGER_PIN4  24
#define TRIGGER_PIN5  22

// Arduino pin tied to echo pin on the ultrasonic sensor.
#define ECHO_PIN1     12
#define ECHO_PIN2     11 
#define ECHO_PIN3     10 
#define ECHO_PIN4     9 
#define ECHO_PIN5     8

// Maximum sonar distance we want to ping (in centimeters).
// The maximum sensor distance is rated at 4 to 5 meters.
#define MAX_DISTANCE 200

// Create NewPing object for each sonar.  Sonars are numbered from
// right to left in the robot's frame of reference.
NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE);
NewPing sonar4(TRIGGER_PIN4, ECHO_PIN4, MAX_DISTANCE);
NewPing sonar5(TRIGGER_PIN5, ECHO_PIN5, MAX_DISTANCE);

// Called once at Arduino initialization.
void setup() {
  // Open serial monitor at 115200 baud to see ping results.
  Serial.begin(115200);
}

// Called periodically in Arduino main loop.
void loop() {

  // Wait 50ms between pings (about 20 pings/sec). 29ms should be the
  // shortest delay between pings.
  delay(50);
  
  // Send ping, get ping time in microseconds (uS).
  unsigned int uS1 = sonar1.ping();
  unsigned int uS2 = sonar2.ping();
  unsigned int uS3 = sonar3.ping();
  unsigned int uS4 = sonar4.ping();
  unsigned int uS5 = sonar5.ping();

  // Convert ping times to distance and send results to serial port.
  // (0 = outside set distance range, no ping echo)
  Serial.print("Ping1: ");
  Serial.print(uS1 / US_ROUNDTRIP_CM);
  Serial.print("cm  ");
  
  Serial.print("Ping2: ");
  Serial.print(uS2 / US_ROUNDTRIP_CM);
  Serial.print("cm  ");
  
  Serial.print("Ping3: ");
  Serial.print(uS3 / US_ROUNDTRIP_CM);
  Serial.print("cm  ");
  
  Serial.print("Ping4: ");
  Serial.print(uS4 / US_ROUNDTRIP_CM);
  Serial.print("cm  ");
  
  Serial.print("Ping5: ");
  Serial.print(uS5 / US_ROUNDTRIP_CM);
  Serial.println("cm  ");
}
