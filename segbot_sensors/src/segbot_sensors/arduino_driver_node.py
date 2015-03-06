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
This is a ROS device driver node that reads and parses serial messages
from the Arduino Mega 2560 mounted on the version 2 BWI segbots,
passing each line of data to a Python module specific to that attached
device.

.. note::

   TODO: add diagnositics

"""

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import importlib
import rospy

from diagnostic_msgs.msg import DiagnosticStatus
from .diagnostics import Diagnostics
from .serial import ArduinoDevice


class ArduinoDriver(object):
    """ ROS driver node for UTexas BWI segbot Arduino messages. """
    def __init__(self, port='/dev/ttyACM0', baud=115200,
                 node_name='arduino_driver'):
        rospy.init_node(node_name)
        port = rospy.get_param('~port', port)
        baud = rospy.get_param('~baud', baud)

        self.diag = Diagnostics()
        """ Diagnostics collector and publisher. """
        self.status = DiagnosticStatus(name=node_name)
        """ Current diagnostic status for this driver. """
        self.set_status(
            level=DiagnosticStatus.OK,
            message='Arduino driver starting.')

        self.arduino = ArduinoDevice(port, baud)
        """ Arduino serial device connection. """
        rospy.on_shutdown(self.shutdown)

        # define the known message types
        self.msgs = {}
        """ Dictionary of message types and handlers. """
        self.add('I', 'segbot_sensors.imu')
        self.add('S', 'segbot_sensors.sonar')
        self.add('V', 'segbot_sensors.voltmeter')

    def add(self, type_char, handler):
        """ Define another Arduino message.

        :param type_char: Identifying first character of this message.
        :type type_char: str
        :param handler: Python module for handling those messages.
        :type handler: str

        Adds messages starting with type_char to arduino_msgs

        :raises: :exc:`.ValueError` if request already exists.
        :raises: :exc:`.ImportError` if unable to import handler.
        """
        if type_char in self.msgs:
            raise ValueError('Duplicate Arduino message type: ' + type_char)
        self.msgs[type_char] = importlib.import_module(handler)

    def set_status(self, level, message):
        """ Update diagnostic status. """
        self.status.level = level
        self.status.message = message
        self.diag.update(self.status)

    def shutdown(self):
        """ Called by rospy on shutdown. """
        self.arduino.close()

    def spin(self):
        """ Main driver loop. """
        slow_poll = rospy.Rate(1.0)     # slow poll frequency
        while not rospy.is_shutdown():

            if self.arduino.ok():       # device connected?
                msg = self.arduino.read()
                if len(msg) > 0:        # something read?
                    handler = self.msgs.get(msg[0:1])
                    if handler is not None:
                        handler.handler(msg)
                    else:
                        rospy.logwarn('unknown Arduino message: ' + msg)
                self.set_status(
                    level=DiagnosticStatus.OK,
                    message='Arduino driver running.')

            elif self.arduino.open():   # open succeeded?
                pass

            else:
                slow_poll.sleep()
                self.set_status(
                    level=DiagnosticStatus.ERROR,
                    message='Arduino not connected.')


def main():
    """ Arduino driver node main entry point."""
    node = ArduinoDriver()
    node.spin()                         # run main driver loop

if __name__ == '__main__':
    main()
