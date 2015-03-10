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
 *  Sonar implementation for Segbot version 2.
 *
 *  This program is driven by timer events.  On a 30Hz cycle, it sends
 *  any previous results in a serial message, then pings the next
 *  sonar in a round-robin fashion.
 */

#include <NewPing.h>
#include <sonar.h>

#define MAX_DISTANCE 200              ///< maximum distance to ping (cm)
#define N_SONARS 3                    ///< number of sonars

/** Each sonar with trigger pin, echo pin, and max distance. */
static NewPing sonar[N_SONARS] =
  {
    NewPing(30, 4, MAX_DISTANCE),
    NewPing(28, 3, MAX_DISTANCE),
    NewPing(26, 2, MAX_DISTANCE),
  };

// These should be class variables, but they are static because the
// NewPing and Arduino timer interfaces can only handle static
// functions, not a class member functions.
static uint8_t static_current = N_SONARS; ///< currently active sonar
static unsigned int static_distance;      ///< distance of current ping

/** Timer interrupt handler.
 *
 *  If ping has completed, set the current distance.
 *
 * This should be a class member function, but it is static because
 * the NewPing and Arduino timer interfaces only allow static
 * functions.
 */
void timer_event()
{ 
  if (sonar[static_current].check_timer())
    {
      static_distance = sonar[static_current].ping_result / US_ROUNDTRIP_CM;
    }
}

/** Poll interface.
 *
 *  When this function is called periodically, it sends any previous
 *  result in a serial message, then starts polling the next sonar.
 */
void Sonar::poll(void)
{
  if (static_current < N_SONARS)         // previous sonar was active?
    {
      // cancel previous timer
      sonar[static_current].timer_stop();

      // publish previous cycle results
      Serial.print("S");                // sonar message marker
      Serial.print(static_current);
      Serial.print("=");
      Serial.print(static_distance);
      Serial.println("cm ");
    }

  // next sonar in the rotation
  if (++static_current >= N_SONARS)
    static_current = 0;

  // start another ping
  static_distance = 0;                        // in case of no echo
  sonar[static_current].ping_timer(timer_event);
}
