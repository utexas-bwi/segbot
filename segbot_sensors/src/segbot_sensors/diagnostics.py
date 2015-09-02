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
ROS diagnostics interface for the Arduino device driver node.
"""

import rospy
import threading
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class Diagnostics(object):
    """ ROS diagnostics interface for the Arduino device driver node. """

    def __init__(self, topic='/diagnostics'):
        self.pub = rospy.Publisher(topic, DiagnosticArray, queue_size=1)
        self.lock = threading.RLock()
        """ Mutex lock for updating diagnostic status. """
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish)
        """ Publish diagnostic info once a second. """
        self.devices = {}
        """ Dictionary of reporting devices by name. """

    def publish(self, event):
        """ Publish current diagnostics status once a second.

        Runs in a separate timer thread, so locking is required.

        :param event: rospy.TimerEvent for this call
        """
        with self.lock:
            array = DiagnosticArray()
            array.header.stamp = event.current_real
            array.status = list(self.devices.values())
            self.pub.publish(array)

    def update(self, status):
        """ Update device status.

        :param status: New status for some device, replaces any
            previous status.
        :type status: diagnostic_msgs/DiagnosticStatus
        """
        with self.lock:
            self.devices[status.name] = status
