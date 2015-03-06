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
 *  Base class for Segbot version 2 Arduino device.
 */

#ifndef _ARDUINO_DEVICE_
#define _ARDUINO_DEVICE_ 1

/** Base class for handling Arduino-attached devices. */
class ArduinoDevice
{
public:

  /** Constructor.
   *
   *  @param poll_msec desired interval between poll calls (milliseconds).
   */
  ArduinoDevice(int poll_msec):
    poll_msec_(poll_msec),
    next_poll_(poll_msec) {}

  /** Periodic device handler.
   *
   *  May send a complete serial message line, if data available.
   */
  virtual void poll() = 0;

  /** Check if time to poll this device.
   *
   *  @param interval Milliseconds since last @c check() call.
   *  @returns @c true, if time to call poll() now; @c false, otherwise.
   */
  bool check(int interval)
  {
    next_poll_ -= interval;
    if (next_poll_ > 0)
      return false;                     // not time for next poll

    next_poll_ = poll_msec_;            // reset counter
    return true;                        // time for next poll
  }

private:
  int poll_msec_;              ///< number of msecs between poll() calls
  int next_poll_;              ///< number of msecs until next poll()
};

#endif // _ARDUINO_DEVICE_
