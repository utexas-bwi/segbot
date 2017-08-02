Overview
========

The `segbot_sensors`_ package contains special-purpose drivers for the
UTexas BWI segbots.


ROS interfaces
==============

arduino_driver
--------------

This node reads serial messages from the Arduino board, converting
data from various attached sensors into ROS messages.

Published topics
''''''''''''''''

``battery0`` (`smart_battery_msgs/SmartBatteryStatus`_)
    Main segbot battery voltage measurements.

``imu0`` (`sensor_msgs/Imu`_)
    Acceleration data from the inertial measurement unit.

``sonar`` (`sensor_msgs/Range`_)
    Sonar ranges for each attached device in its own tf frame.

``/diagnostics`` (`diagnostic_msgs/DiagnosticArray`_)
    Various device diagnostics.

Parameters
''''''''''

``~baud`` (int, default: 115200)
    Baud rate for the Arduino serial port.

``~port`` (string, default: /dev/ttyACM0)
    Serial port attached to the Arduino.

Usage
'''''

To run just this driver node::

    $ rosrun segbot_sensors arduino_driver

ranges_to_cloud
---------------

This node converts sonar Range messages to point clouds.

Subscribed topics
'''''''''''''''''

``sonar`` (`sensor_msgs/Range`_)
    Sonar ranges for each attached device.

Published topics
''''''''''''''''

``sensor_ranges`` (`sensor_msgs/PointCloud2`_)
    A centered distance point for each sonar range.

Usage
'''''

To run both the ``arduino_driver`` and this node::

    $ roslaunch segbot_sensors arduino.launch

.. _`diagnostic_msgs/DiagnosticArray`:
   http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticArray.html
.. _`segbot_sensors`:
   http://wiki.ros.org/segbot_sensors
.. _`sensor_msgs/Imu`:
   http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
.. _`sensor_msgs/PointCloud2`:
   http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
.. _`sensor_msgs/Range`:
   http://docs.ros.org/api/sensor_msgs/html/msg/Range.html
.. _`smart_battery_msgs/SmartBatteryStatus`:
   http://docs.ros.org/api/smart_battery_msgs/html/msg/SmartBatteryStatus.html
