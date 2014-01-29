Segbot Arduino firmware
=======================

This package contains Arduino firmware for the UTexas BWI segbot
sensor array.


Build and Install
-----------------

The CMakeLists.txt does not yet support make and install from the
command line.  To build and install firmware on the Arduino board,
first install the Arduino development tools.  On Ubuntu, run these
commands:

    $ sudo apt-get install arduino
    $ mkdir ~/sketchbook 
    $ cd ~/sketchbook
    $ ln -s $(rospack find segbot_firmware)/src/libraries .

Plug in the USB cable, then run Arduino GUI:

    $ arduino

Select ``File > Sketchbook > Libraries`` followed by the desired
firmware version.  Then, click the ``->`` icon to compile the
microcode and load it into the controller.

ROS Driver
----------

When the firmware is loaded, start ``roscore`` and run the ROS device
driver:

    $ rosrun segbot_sensors sensor_ranges_driver

The driver publishes sensor readings on the ``/sensor_ranges`` topic.
To see the data, run:

    $ rostopic echo /sensor_ranges
