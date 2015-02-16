Segbot Arduino firmware
=======================

This package contains Arduino firmware for the UTexas BWI segbot
sensor array.

Access to Serial Ports
----------------------

To access the Arduino serial port, you must be authorized for the
"dialout" group.  To verify that you are a member of that group, run
this command:

    $ id | grep dialout

If you are in the group, "dialout" will appear in the printed output.   

If you are not yet authorized, run this command on a personal Ubuntu
system:

    $ sudo usermod -a -G dialout your_user_id

**But**, *do not* run that command on any of the BWI robots or lab
machines.  They maintain a shared user and group database, so ask a
system administrator to update it for you.

In either case, logout and login after your group membership has been
updated, and verify that you are now in the "dialout" group.

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

Plug in the USB cable, then run the Arduino GUI:

    $ arduino

Select ``File > Sketchbook > Libraries`` followed by the desired
firmware version.  The standard one is named ``segbot_arduino``.
Then, click the ``->`` icon to compile the microcode and load it into
the controller.

ROS Driver
----------

When the firmware is loaded, you can run the ROS device driver:

    $ roslaunch segbot_sensors arduino.launch --screen

The ``segbot_sensors`` package documentation describes the topics
published.
