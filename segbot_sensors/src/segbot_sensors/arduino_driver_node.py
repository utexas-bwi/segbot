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

import io
import importlib
import select
import serial

import rospy
from sensor_msgs.msg import Range


class ArduinoDevice(object):
    """ Class for managing the Arduino serial port connection.
    """
    def __init__(self, port='/dev/ttyACM0', baud=115200):
        self.port = port
        """ Path name for Arduino serial port. """
        self.baud = baud
        """ Baud rate for Arduino serial port. """
        self.dev = None
        """ Arduino serial device connection. """

    def close(self):
        if self.dev:
            self.dev.close()
        self.dev = None

    def ok(self):
        """ :returns: ``True`` if Arduino contacted. """
        return self.dev is not None

    def open(self):
        """ Open the Arduino serial device interface.

        :returns: ``True`` if open succeeds.
        """
        try:
            self.dev = serial.Serial(self.port, self.baud)
        except IOError as e:
            # HACK: serial does not return errno.ENOTTY as it should,
            #       so check the exact string.
            enotty = ("Could not configure port: "
                      + "(25, 'Inappropriate ioctl for device')")
            if str(e) != enotty:        # is it a serial port?
                rospy.logerr('Serial port open failed at '
                             + str(self.baud) + ' baud: ' + str(e))
                return False
        else:
            rospy.loginfo('Serial port ' + self.port + ' opened at '
                          + str(self.baud) + ' baud.')
            self.dev.flushInput()       # discard any old data
            return True

        # Not a serial port: see if it's a regular file with test data.
        try:
            self.dev = io.open(self.port, 'rb')
        except IOError as e:
            rospy.logerr('File open failed: ' + str(e))
            return False
        else:
            rospy.loginfo('Test file opened: ' + self.port)
            return True

    def read(self):
        """ Read a line from the serial port.

        :returns: a string containing the next line, maybe empty.
        """
        serial_msg = ''
        try:
            serial_msg = self.dev.readline()
        except serial.SerialException as e:
            rospy.logerr('Serial port ' + self.port +
                         ' read failed: ' + str(e))
            self.close()
        except (select.error, OSError) as e:
            errno_, perror = e.args
            rospy.logwarn('Serial port read error: ' + str(perror))
            self.close()
        else:
            # Sometimes the Arduino sends out-of-range characters on
            # start-up. Just ignore them.
            serial_msg = serial_msg.decode('ascii', 'ignore')
            rospy.logdebug('Arduino message: ' + serial_msg)
            if serial_msg == '':        # end of test data file?
                self.close()            # test ended
            return serial_msg
        return ''                       # no data read


class ArduinoDriver(object):
    """ ROS driver node for UTexas BWI segbot Arduino messages. """
    def __init__(self, port='/dev/ttyACM0', baud=115200):
        rospy.init_node('arduino_driver')
        port = rospy.get_param('~port', port)
        baud = rospy.get_param('~baud', baud)
        self.arduino = ArduinoDevice(port, baud)
        """ Arduino serial device connection. """
        rospy.on_shutdown(self.shutdown)

        # define the known message types
        self.msgs = {}
        """ Dictionary of message types and handlers. """
        self.add('S', 'segbot_sensors.sonar')
        self.add('I', 'segbot_sensors.imu')

        self.spin()                     # run main driver loop

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

    def shutdown(self):
        """ Called by rospy on shutdown. """
        self.arduino.close()

    def spin(self):
        """ Main driver loop. """
        slow_poll = rospy.Rate(0.25)    # slow poll frequency
        while not rospy.is_shutdown():
            if self.arduino.ok():       # device connected?
                msg = self.arduino.read()
                if len(msg) == 0:       # nothing read?
                    continue
                handler = self.msgs.get(msg[0:1])
                if handler is not None:
                    handler.main(msg)
                else:
                    rospy.logwarn('unknown Arduino message: ' + msg)
            elif self.arduino.open():   # open succeeded?
                pass
            else:
                slow_poll.sleep()


def main():
    """ Arduino driver node main entry point."""
    node = ArduinoDriver()

if __name__ == '__main__':
    main()
