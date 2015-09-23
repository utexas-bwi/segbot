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
This module handles IMU messages from the Arduino Mega 2560 mounted
on BWI segbots, publishing them as ROS sensor_msgs/Imu.
"""

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import re

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


def convert_accel(int_val):
    """ Convert acceleration component from device value to ROS value.

    :param int_val: Reading from device.
    :type int_val: str
    :returns: float value in meters per second squared.

    TODO: Figure out the actual conversion. The device readings are a
    signed A/D converter value based on a full range of 2g, which is
    about 19.6 m/s/s.
    """
    return (float(int_val) / 32768.0) * 19.6


class ImuAttributes(Imu):
    """ Subclass of sensor_msgs/Imu, for filling in IMU attributes.

    The IMU orientation is ignored.  That information is not very
    useful, because the fixed frame of reference is not well-defined.

    The covariances of angular velocity and linear acceleration are
    presently unknown.  It would be helpful to figure out what they
    should be.
    """
    def __init__(self, frame_id):
        super(ImuAttributes, self).__init__(
            orientation_covariance=[-1.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0],
            angular_velocity_covariance=[-1.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0],
            linear_acceleration_covariance=[0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0])
        if frame_id:
            self.header.frame_id = frame_id


class ImuMessages(object):
    """ ROS message translation for UTexas BWI segbot Arduino imu ranges. """
    def __init__(self):
        self.parser = re.compile(
            r'I(\d+) accel x: ([-+]?\d+) y: ([-+]?\d+) z: ([-+]?\d+)')
        """ Extracts IMU data from the Arduino serial message.

        :returns: List of IMU data strings reported, may be empty.
        """
        self.pubs = [rospy.Publisher('imu0', Imu, queue_size=1)]
        self.imus = [ImuAttributes('imu0')]

    def publish(self, serial_msg):
        """ Publish a ROS Range message for each reading.

        :param serial_msg: IMU message from Arduino.
        :type serial_msg: str
        """
        # Parse serial message line into a list of IMU data strings
        # containing integers.
        readings = self.parser.match(serial_msg)
        if not readings:                # no data available?
            rospy.logwarn('Invalid IMU message: ' + serial_msg)
            return
        now = rospy.Time.now()          # time when message received
        imu = int(readings.group(1))

        if imu >= len(self.imus):       # unknown IMU?
            rospy.logwarn('Invalid IMU number: ' + str(imu))
            return                      # skip this reading

        # Convert IMU acceleration components to ROS units
        x = convert_accel(readings.group(2))
        y = convert_accel(readings.group(3))
        z = convert_accel(readings.group(4))
        rospy.logdebug('IMU' + str(imu)
                       + ' accel: ' + str(x) + ', ' + str(y) + ', ' + str(z))
        msg = self.imus[imu]
        msg.linear_acceleration = Vector3(x, y, z)
        msg.header.stamp = now

        self.pubs[imu].publish(msg)


imu = ImuMessages()                # make an ImuMessages instance
handler = imu.publish              # declare message handler interface
""" This interface is called once for each IMU message received. """
