/* -*- mode: C++ -*- */
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2015, Max Svetlik, Pato Lankenau, Jack O'Quin
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

/** @file Sonar implementation for Segbot version 2.
 *
 *  This program is driven by timer events.  On a 30Hz cycle, it pings
 *  one sonar, and sends the previous results in a serial message.  In
 *  the next cycle, it pings the next sonar in a round-robin fashion.
 */

#include <NewPing.h>
#include <sonar.h>

// declare each sonar with trigger pin, echo pin, and max distance
static NewPing sonar[N_SONARS] =
  {
    NewPing(30, 4, MAX_DISTANCE),
    NewPing(28, 3, MAX_DISTANCE),
    NewPing(26, 2, MAX_DISTANCE),
  };

/** Timer interrupt handler.
 *
 *  If ping has completed, set the current distance.
 */
void Sonar::timer_event()
{ 
  if (sonar[current_sonar_].check_timer())
    {
      distance_ = sonar[current_sonar_].ping_result / US_ROUNDTRIP_CM;
    }
}

/** Poll interface.
 *
 *  When this function is called periodically, it sends any previous
 *  result in a serial message, then starts polling the next sonar.
 */
void Sonar::poll(void)
{
  if (current_sonar_ < N_SONARS)         // previous sonar was active?
    {
      // cancel previous timer
      sonar[current_sonar_].timer_stop();

      // publish previous cycle results
      Serial.print("S");                // sonar message marker
      Serial.print(current_sonar_);
      Serial.print("=");
      Serial.print(distance_);
      Serial.println("cm ");
    }

  // next sonar in the rotation
  if (++current_sonar_ >= N_SONARS)
    current_sonar_ = 0;

  // start another ping
  distance_ = 0;                        // in case of no echo
  sonar[current_sonar_].ping_timer(timer_event);
}
