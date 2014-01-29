Segbot Arduino firmware
=======================

Build and install
-----------------

The CMakeLists.txt does not yet support make and install from the
command line.  To build and install firmware on the Arduino board,
first install the Arduino development tools.  On Ubunut, install this
package:

    $ sudo apt-get install arduino

Then run these commands:

    $ mkdir ~/sketchbook 
    $ cd ~/sketchbook
    $ ln -s libraries $(rospack find segbot_firmware)/src/libraries .
    $ arduino

In the Arduino GUI, select File > Sketchbook > Libraries and the
desired firmware version.  Then, click on the ``->`` icon to compile
and load the program into the controller.
