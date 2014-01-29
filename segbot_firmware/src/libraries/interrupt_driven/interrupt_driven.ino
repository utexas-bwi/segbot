/* -*- mode: C++ -*-
/*********************************************************************
* Software License Agreement (BSD License)
*
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

// Based on NewPing library timer-driven 15 sensor example.
//
// Keep in mind this example is event-driven. Your complete sketch
// needs to be written so there's no "delay" commands and the loop()
// cycles at faster than a 33ms rate. If other processes take longer
// than 33ms, you'll need to increase PING_INTERVAL so it doesn't get
// behind.
 
#include <NewPing.h>

#define N_SENSORS      5          // Number of sensors
//#define N_SENSORS      7          // Number of sensors including IR
#define MAX_DISTANCE 200          // Maximum distance to ping (in cm)

// Each sensor is pinged at 33ms intervals (29ms is about the minimum
// to avoid cross-sensor echo). 
#define PING_INTERVAL 33

// One cycle of all sensors takes 33 * N_SENSORS ms.
#define CYCLE_MS (PING_INTERVAL * N_SENSORS)

// The time when the next ping should happen for each sensor.
unsigned long ping_timer[N_SENSORS];   

uint8_t current_sensor = 0;            // which sensor is active
unsigned int distance[N_SENSORS];      // current ping distances

// Each sensor's trigger pin, echo pin, and max distance to ping.
NewPing sensor[N_SENSORS] =
  {
    NewPing(30, 12, MAX_DISTANCE),
    NewPing(28, 11, MAX_DISTANCE),
    NewPing(26, 10, MAX_DISTANCE),
    NewPing(24,  9, MAX_DISTANCE),
    NewPing(22,  8, MAX_DISTANCE),
    //NewPing(20,  7, MAX_DISTANCE),
    //NewPing(18,  6, MAX_DISTANCE),
  };

#define LED_PIN 13                      // Pin with LED attached.

// Called once on start-up.
void setup() 
{
  Serial.begin(115200);                 // serial port baud rate
  pinMode(LED_PIN, OUTPUT);             // configure LED output

  // The first ping starts after 75ms, giving the Arduino time to
  // initialize.  Others follow at PING_INTERVAL ms.
  ping_timer[0] = millis() + 75;
  for (uint8_t i = 1; i < N_SENSORS; i++)
    {
      ping_timer[i] = ping_timer[i-1] + PING_INTERVAL;
    }
}

// Called repeatedly in Arduino main loop.
void loop()
{
  for (uint8_t i = 0; i < N_SENSORS; i++)
    {
      if (millis() >= ping_timer[i])     // time to ping this sensor?
        {
          ping_timer[i] += PING_INTERVAL * N_SENSORS;

          if (i == 0 && (current_sensor == N_SENSORS - 1))
            send_results();

          // Cancel previous timer and start a new ping cycle.
          sensor[current_sensor].timer_stop();
          current_sensor = i;
          distance[current_sensor] = 0; // in case of no echo
          sensor[current_sensor].ping_timer(echoCheck); // start next ping
          // (processing continues, interrupt will call echoCheck to
          // look for echo).
        }
    }
}

// If ping received, set the sensor distance to array.
void echoCheck()
{ 
  if (sensor[current_sensor].check_timer())
    distance[current_sensor] =
      sensor[current_sensor].ping_result / US_ROUNDTRIP_CM;
}

// Ping cycle complete, send the results.
void send_results()
{
  for (uint8_t i = 0; i < N_SENSORS; i++)
    {
      Serial.print(i);
      Serial.print("=");
      Serial.print(distance[i]);
      Serial.print("cm ");
    }
  Serial.println();

  // turn the LED on or off with each message sent
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}
