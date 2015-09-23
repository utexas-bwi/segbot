#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (C) 2015, Jack O'Quin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the author nor of other contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
This module handles sonar messages from the Arduino Mega 2560 mounted
on BWI segbots, publishing them as ROS sensor_msgs/Range.
"""

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import re

import rospy
from sensor_msgs.msg import Range


class SonarAttributes(Range):
    """ Subclass of sensor_msgs/Range, for filling in sonar attributes."""
    def __init__(self, frame_id,
                 field_of_view=0.4, min_range=0.01, max_range=2.0):
        super(SonarAttributes, self).__init__(
            radiation_type=Range.ULTRASOUND,
            field_of_view=field_of_view,
            min_range=min_range,
            max_range=max_range)
        if frame_id:
            self.header.frame_id = frame_id


class SonarMessages(object):
    """ ROS message translation for UTexas BWI segbot Arduino sonar ranges. """
    def __init__(self):
        self.pub = rospy.Publisher('sonar', Range, queue_size=6)
        self.parser = re.compile(r'(\d+)=(\d+)cm')
        """ Extracts list of distances from the Arduino serial message.

        :returns: List of (sonar, distance) pairs of strings reported
            for the sonars, may be empty.
        """

        # Our version 2 segbots have three sonars.
        # TODO: define this configuration information using ROS parameters
        self.sonars = [
            SonarAttributes('sonar0_link'),
            SonarAttributes('sonar1_link'),
            SonarAttributes('sonar2_link')]

    def publish(self, serial_msg):
        """ Publish a ROS Range message for each reading.

        :param serial_msg: sonar message from Arduino.
        :type serial_msg: str
        """
        # Parse serial message line into a list of (sonar, distance)
        # pairs of strings reported for one or more sonars.  The
        # strings represent integers, with distances in centimeters.
        readings = self.parser.findall(serial_msg)
        if not readings:                # no data available?
            return
        now = rospy.Time.now()          # time when message received
        for sonar, distance in readings:
            sonar = int(sonar)
            if sonar >= len(self.sonars):  # unknown sonar?
                continue                   # skip this reading
            msg = self.sonars[sonar]
            msg.header.stamp = now
            distance = int(distance)
            if distance == 0:           # nothing detected?
                msg.range = float('+inf')  # follow REP-0117
            else:
                # Convert distance from int centimeters to float meters.
                msg.range = 0.01 * float(distance)
            self.pub.publish(msg)


sonar = SonarMessages()                # make a SonarMessages instance
handler = sonar.publish                # declare message handler interface
""" This interface is called once for each sonar message received. """
