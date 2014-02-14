/* -*- mode: C++ -*- */
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

/** @file Arduino sensor firmware.
 *
 *  This program is driven by timer events.  To improve cycle
 *  frequency, the sonars are grouped into sets pointing in divergent
 *  directions to avoid mutual interference.  We ping them in parallel
 *  for a 33ms cycle.
 */
 
#include <NewPing.h>

#define N_IR 2                          // number of infrared sensors
#define N_SONARS 5                      // number of sonars
#define N_GROUPS 3                      // number of sonar polling groups
#define MAX_GROUP (N_SONARS + N_GROUPS - 1) / N_GROUPS
#define N_SENSORS (N_SONARS + N_IR)     // total number of sensors
#define PING_INTERVAL 33                // ping duration (in milliseconds)
#define LED_PIN 13                      // pin with LED attached.
#define MAX_DISTANCE 200                // maximum distance to ping (in cm)

// declare each sonar with trigger pin, echo pin, and max distance
NewPing sonar[N_SONARS] =
  {
    NewPing(30, 12, MAX_DISTANCE),
    NewPing(28, 11, MAX_DISTANCE),
    NewPing(26, 10, MAX_DISTANCE),
    NewPing(24,  9, MAX_DISTANCE),
    NewPing(22,  8, MAX_DISTANCE)
  };

#if N_IR > 0
int ir_pin[N_IR] = {A0, A1};            // array of IR analog pins to poll
#endif

// corresponding sensor distances: first sonars, then infrared sensors
unsigned int distance[N_SENSORS];

// To improve cycle frequency, the sonars are grouped into sets
// pointing in divergent directions to avoid mutual interference.
unsigned long ping_timer;               // time to start next group
struct {
  uint8_t n_sonars;                     // number of sonars in the group
  uint8_t sensor[MAX_GROUP];            // sensor indexes in the group
} groups[N_GROUPS];

uint8_t current_group = N_GROUPS;       // which sonar group is active

/// Called once on start-up.
void setup() 
{
  Serial.begin(115200);                 // serial port baud rate
  pinMode(LED_PIN, OUTPUT);             // configure LED output

  // The first ping starts after 75ms, giving the Arduino time to
  // initialize.  Others follow at PING_INTERVAL ms.
  ping_timer = millis() + 75;

  // initialize the sonar groups
  for (uint8_t grp = 0; grp < N_GROUPS; ++grp)
    {
      groups[grp].n_sonars = 0;
      for (uint8_t i = grp; i < N_SONARS; i += N_GROUPS)
        {
          groups[grp].sensor[groups[grp].n_sonars] = i;
          ++groups[grp].n_sonars;
        }
    }
  for (uint8_t i = 0; i < N_SENSORS; ++i)
    {
      distance[i] = 0;
    }
}

/// Called repeatedly in Arduino main loop, must complete in under 33ms.
void loop()
{
  if (millis() >= ping_timer)       // time to start next sonar group?
    {
      ping_timer += PING_INTERVAL;

      if (current_group < N_GROUPS)     // previous group was active?
        {
          // finish previous cycle
          poll_infrared();
          send_results();

          // cancel previous timers
          for (uint8_t i = 0; i < groups[current_group].n_sonars; ++i)
            {
              sonar[groups[current_group].sensor[i]].timer_stop();
            }
        }

      // start a new ping cycle
      ++current_group;
      if (current_group >= N_GROUPS)
        current_group = 0;
      for (uint8_t i = 0; i < groups[current_group].n_sonars; ++i)
        {
          uint8_t sensor = groups[current_group].sensor[i];
          distance[sensor] = 0;                  // in case of no echo
          sonar[sensor].ping_timer(timer_event); // start next ping
        }
    }
}

/// Timer interrupt handler.
//
//  If ping completed, set the sonar distance in the array.
void timer_event()
{ 
  for (uint8_t i = 0; i < groups[current_group].n_sonars; ++i)
    {
      uint8_t sensor = groups[current_group].sensor[i];
      if (sonar[sensor].check_timer())
        {
          distance[sensor] = sonar[sensor].ping_result / US_ROUNDTRIP_CM;
        }
    }
}

/// Poll all infrared sensors
//
//  Done at the end of every cycle.  These sensors do not interfere
//  with one another, and their input is desired as frequently as
//  possible to avoid falling down stairs.
//
//  These sensors are noisy: maybe it would help to sample the analog
//  inputs on every timer interrupt and compute a median filter or
//  exponentially weighted moving average.
void poll_infrared()
{ 
#if N_IR > 0
  for (uint8_t i = 0; i < N_IR; ++i)
    {
      float volts = analogRead(ir_pin[i]) * (5.0 / 1024);
      distance[N_SONARS + i] = 65.0 * pow(volts, -1.10);
    }
#endif
}

/// Format reading for one sensor.
inline void format_reading(uint8_t sensor)
{
      Serial.print(sensor);
      Serial.print("=");
      Serial.print(distance[sensor]);
      Serial.print("cm ");
}

/// Sonar ping cycle complete, send the results.
void send_results()
{
  // format readings for current sonar group
  for (uint8_t i = 0; i < groups[current_group].n_sonars; ++i)
    {
      format_reading(groups[current_group].sensor[i]);
    }

  // append readings for any infrared sensors
  for (uint8_t i = N_SONARS; i < N_SENSORS; ++i)
    {
      format_reading(i);
    }

  // send single-line message
  Serial.println();

  // turn the LED on or off with each message sent
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}
