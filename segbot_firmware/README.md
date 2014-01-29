Segbot Arduino firmware
=======================

Build and install
-----------------

The CMakeLists.txt does not yet support make and install from the
command line.  To build and install firmware on the Arduino board,
first install the Arduino development tools.  On Ubuntu, run these
commands:

    $ sudo apt-get install arduino
    $ mkdir ~/sketchbook 
    $ cd ~/sketchbook
    $ ln -s libraries $(rospack find segbot_firmware)/src/libraries .

Then run Arduino GUI:

    $ arduino

Select ``File > Sketchbook > Libraries`` followed by the desired
firmware version.  Then, click the ``->`` icon to compile the
microcode and load it into the controller.
