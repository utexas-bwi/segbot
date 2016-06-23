#!/usr/bin/env python
import rospy
import roslib
import sys
import rospy
import rosbag
import numpy
import scipy.optimize
import rospkg
import socket

from matplotlib import pyplot
from scipy.optimize import curve_fit
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg

"""
This program is a diagnostic tool for interpreting offline voltage data from the Segbot fleet.
In particular, the program reads in a .bag file, and extracts voltage data published on the /diagnostics topic.

Currently, this
            plots the voltage data as well as the fitted model.
            writes the model to file for time-remaining estimation (handled elsewhere)

Possible improvements:
    Have ways of merging multple runs into one model, or an existing model and an additional run
    Change from half hours to hours, without breaking fitting function.
        (note this would also requires changes to the battery diagnostics node)

Author: Maxwell J Svetlik
"""

sliding_window = 1
half_hour_conversion = 1800
eol_voltage = 10.75


def model_func(t, A, K, C):
    return -A * numpy.exp(K * t) + C


def fit_exp_nonlinear(t, y):
    opt_parms, parm_cov = scipy.optimize.curve_fit(model_func, t, y, maxfev=1000)
    A, K, C = opt_parms
    return A, K, C


class battery_profiler:

    def __init__(self):
        print 'STARTING BATTERY PROFILER'
        self.voltage_values = []
        self.time_values = []
        self.time_base = -1
        self.rospack = rospkg.RosPack()
        self.model_filename = self.rospack.get_path('segbot_sensors') + "/config/battery_profile"

    def mySmooth(self, data, window):
        smoothed = numpy.zeros((len(data), 1))
        for d in range(len(data)):
            smoothed[d] = numpy.mean(data[max(0, d-window):d])
        return smoothed

    """
    returns the time remaining based on the current voltage value, and the estimated time to reach
    the end of life voltage (eol_voltage). Note that this voltage may need to be changed based on the specific
    battery type. For instance, on a segbot_v3, this value should be around 71.5
    """
    def get_time_estimate(self, A, K, C, voltage):
        max_life = numpy.log((eol_voltage - C) / -A) / K
        if voltage > C:
                A = -A
                return (numpy.log((voltage - C) / -A) / K + max_life)
        cur_life = numpy.log((voltage - C) / -A) / K
        return max_life - cur_life

    def writeModelToFile(self, A, K, C):
        print "Opening the file at {}...".format(self.model_filename)
        target = open(self.model_filename, 'w')
        target.write(str(A)+'\n')
        target.write(str(K)+'\n')
        target.write(str(C)+'\n')
        target.close()
        print "Model written to configuration file successfully."

    def listener(self):
        try:
            bag = rosbag.Bag(sys.argv[1])
        except IOError:
            print 'Could not open bag file at ' + sys.argv[1]
            sys.exit(-1)
        topic, msg_arr, t = bag.read_messages(topics=['/diagnostics']).next()
        self.time_base = msg_arr.header.stamp.to_sec()
        print 'Base time: {}\n'.format(self.time_base)

        for topic, msg_arr, t in bag.read_messages(topics=['/diagnostics']):
            for msg in msg_arr.status:
                if msg.name == 'voltage':
                    self.voltage_values.append(float(msg.values[0].value))
                    """
                    Collect relative times, and convert into half-hours
                    Note that this is necessary for the exp-fit function; minutes are too large, hours are too small
                    in terms of values that the fit function can handle.
                    """
                    self.time_values.append(float((msg_arr.header.stamp.to_sec() -
                                                   self.time_base) / half_hour_conversion))
        bag.close()

        pyplot.plot(self.time_values, self.mySmooth(self.voltage_values, sliding_window), 'b', label="Voltage Data")
        title = 'Segbot Voltage over Time on Active Use: {}'.format(socket.gethostname())
        pyplot.legend(loc=4)
        pyplot.title(title)
        pyplot.xlabel('Time ( 30-min Increments)')
        pyplot.ylabel('Voltage')
        pyplot.legend(loc='upper right')

        total_time = self.time_values[len(self.time_values) - 1] - self.time_values[0]
        print 'EXPERIMENT DATA'
        print 'Total run time: {}'.format(total_time)
        print 'Starting voltage: {}'.format(self.voltage_values[0])
        print 'Ending voltage: {}'.format(self.voltage_values[len(self.voltage_values) - 1])

        A, K, C = fit_exp_nonlinear(numpy.asarray(self.time_values), numpy.asarray(self.voltage_values))
        fit_y = model_func(numpy.asarray(self.time_values), A, K, C)
        print 'Model: -{} * exp({}x) + {}\n'.format(A, K, C)
        self.writeModelToFile(A, K, C)

        pyplot.plot(numpy.asarray(self.time_values), fit_y, "r--", label="Exponential Model")
        pyplot.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print 'Missing argument: path to bagfile. Exiting.'
        sys.exit(-1)
    battery_profiler = battery_profiler()
    rospy.init_node('battery_profiler', anonymous=True)
    battery_profiler.listener()
