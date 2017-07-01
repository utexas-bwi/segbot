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
This module handles voltage messages from the Arduino Mega 2560
mounted on BWI segbots, publishing them as ROS
smart_battery_msgs/SmartBatteryStatus messages.
"""

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import re

import rospy
from smart_battery_msgs.msg import SmartBatteryStatus
from geometry_msgs.msg import Vector3


class VoltmeterMessages(object):
    """ ROS message translation for UTexas BWI segbot voltage obtained
    from Arduino sensor. """
    def __init__(self):
        self.parser = re.compile(
            r'V(\d+(\.\d*)?|\.\d+)')
        """ Extracts voltage reading from the Arduino serial message.
        :returns: voltage data string reported, may be empty.
        """
        self.pub = rospy.Publisher('battery0', SmartBatteryStatus,
                                   queue_size=1)
        # initialize constant fields in battery message
        self.battery = SmartBatteryStatus(
            rate=float('nan'),
            charge=float('nan'),
            capacity=float('nan'),
            design_capacity=float('nan'),
            charge_state=SmartBatteryStatus.DISCHARGING,
            present=1)

    def publish(self, serial_msg):
        """ Publish a ROS Range message for each reading.

        :param serial_msg: voltage message from Arduino.
        :type serial_msg: str

        TODO: publish /diagnostic message, too
        """
        # Parse serial message line into a list of voltage data strings
        # containing integers.
        readings = self.parser.match(serial_msg)
        if not readings:                # no data available?
            rospy.logwarn('Invalid voltage message: ' + serial_msg)
            return

        # Format ROS battery status message
        self.battery.header.stamp = rospy.Time.now()
        self.battery.voltage = float(readings.group(1))
        rospy.logdebug('voltage: ' + str(self.battery.voltage))
        self.pub.publish(self.battery)


voltmeter = VoltmeterMessages()   # make an VoltmeterMessages instance
handler = voltmeter.publish       # declare message handler interface
""" This interface is called once for each voltage message received. """
