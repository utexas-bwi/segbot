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
This module handles serial messages from the Arduino Mega 2560 mounted
on the version 2 BWI segbots.
"""

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import io
import rospy
import select
import serial


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
            enotty = ("Could not configure port: " +
                      "(25, 'Inappropriate ioctl for device')")
            if str(e) != enotty:        # is it a serial port?
                rospy.logerr('Serial port open failed at ' +
                             str(self.baud) + ' baud: ' + str(e))
                return False
        else:
            rospy.loginfo('Serial port ' + self.port + ' opened at ' +
                          str(self.baud) + ' baud.')
            self.dev.flushInput()       # discard any old data
            return True

        # Not a serial port: see if it's a regular file with test data.
        try:
            self.dev = io.open(self.port, 'rb')
        except IOError as e:
            rospy.logerr('File open failed: ' + str(e))
            return False
        else:
            rospy.logdebug('Test file opened: ' + self.port)
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
