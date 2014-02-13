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
 *  This program is driven by timer events.  The sonars are grouped
 *  into sets which point in different directions.  We ping them in
 *  parallel for a 33ms cycle.  The main loop() must complete in less
 *  than 33ms.
 */
 
#include <NewPing.h>

#define N_IR 2                          // Number of infrared sensors
#define N_SONARS 5                      // Number of sonars
#define N_GROUPS 3                      // Number of sonar polling groups
#define N_SENSORS (N_SONARS + N_IR)     // Total number of sensors
#define MAX_DISTANCE 200                // Maximum distance to ping (in cm)
#define PING_INTERVAL 33                // Ping duration (in milliseconds)
#define LED_PIN 13                      // Pin with LED attached.

// The time for the next sonar group ping to begin.
unsigned long ping_timer;   

uint8_t current_group = N_GROUPS;       // which sonar group is active
unsigned int distance[N_SENSORS];       // current sensor distances

// Each sonars's trigger pin, echo pin, and max distance to ping.
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

// Called once on start-up.
void setup() 
{
  Serial.begin(115200);                 // serial port baud rate
  pinMode(LED_PIN, OUTPUT);             // configure LED output

  // The first ping starts after 75ms, giving the Arduino time to
  // initialize.  Others follow at PING_INTERVAL ms.
  ping_timer = millis() + 75;
  for (uint8_t i = 0; i < N_SENSORS; i++)
    {
      distance[i] = 0;
    }
}

// Called repeatedly in Arduino main loop.
void loop()
{
  if (millis() >= ping_timer)       // time to start next sonar group?
    {
      ping_timer += PING_INTERVAL;

      if (current_group < N_GROUPS)
        {
          // finish previous cycle
          poll_infrared();
          send_results();

          // cancel previous timer and start a new ping cycle.
          sonar[current_group].timer_stop();
          distance[current_group] = 0; // in case of no echo
          sonar[current_group].ping_timer(timer_event); // start next ping

          ++current_group;
          if (current_group >= N_GROUPS)
            current_group = 0;
        }
    }
}

// Timer interrupt handler:
//
// If ping completed, set the sonar distance in the array.
void timer_event()
{ 
  for (uint8_t i = 0; i < groups[current_group].n_sonars; i++)
    {
      if (sonar[i].check_timer())
        distance[groups[current_group].sensor] =
          sonar[i].ping_result / US_ROUNDTRIP_CM;
    }
}

/// Poll all infrared sensors
//
//  Done at the end of every cycle.  These sensors do not interfere
//  with one another, and their input is desired as frequently as
//  possible to avoid falling down stairs.
//
//  TODO: These sensors are noisy: see if it helps to sample the
//  analog inputs on every timer interrupt and compute exponentially
//  weighted moving averages.
void poll_infrared()
{ 
#if N_IR > 0
  for (uint8_t i = 0; i < N_IR; i++)
    {
      float volts = analogRead(ir_pin[i]) * (5.0 / 1024);
      distance[N_CYCLES + i] = 65.0 * pow(volts, -1.10);
    }
#endif
}

// Sonar ping cycle complete, send the results.
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
