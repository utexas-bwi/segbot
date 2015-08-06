^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package segbot_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.3 (2015-08-05)
------------------
* segbot_bringup: launch stop controller with segway base (`#41 <https://github.com/utexas-bwi/segbot/issues/41>`_)
* segbot_bringup: pass correct port to Arduino driver
* fixed launch command for arduino and diagnostics. closes `#36 <https://github.com/utexas-bwi/segbot/issues/36>`_.
* segbot_bringup: convert to package format two (`#35 <https://github.com/utexas-bwi/segbot/issues/35>`_)
* segbot_bringup: add diagnostics and arduino to segbot_v2 launch (`#35 <https://github.com/utexas-bwi/segbot/issues/35>`_)
* segbot_bringup: minor launch file cleanup and formatting
* Contributors: Jack O'Quin, Piyush Khandelwal

0.3.2 (2015-03-31)
------------------

0.3.1 (2015-03-24)
------------------
* removed redundant segbot v1 models.
* updated segbot v2 model to use correct URDF.
* Contributors: Piyush Khandelwal

0.3.0 (2015-03-14)
------------------
* fixed default depth_registered flag to allow kinect to work properly.
* use different filters for segbot v1 and segbot v2 due to differences between different laser models.
* added launch file for segbot v2.
* fixed some issues with frame ids for multi robot setup.
* Changed the value of "depth_only" to "true" so that the published point cloud contains color information as well.
* Contributors: Jivko Sinapov, Piyush Khandelwal

0.2.1 (2014-04-24)
------------------
* segbot_bringup: install udev rules (`#26
  <https://github.com/utexas-bwi/segbot/issues/26>`_)
* fix some problems catkin_lint discovered

0.2.0 (2014-04-17)
------------------

* Release to Hydro.
* Added udev rules for segbot, and updated the kinect laserscan
  configuration.
* Catkin_lint approved packages.
* Updated descriptions and licensing information
* Fixed licensing information from borrowed scripts.

0.1.9 (2013-12-15)
------------------
* revert update for `#16`_ now that we can use the standard
  robot_state_publisher (`#21`_).

.. _`#16`: https://github.com/utexas-bwi/segbot/issues/16
.. _`#21`: https://github.com/utexas-bwi/segbot/issues/21

0.1.8 (2013-12-04)
------------------
* use ``roslaunch_add_files_check()`` to test that required launch
  file dependencies are declared.
* fixed a few minor errors found on the real robot
* added configuration with both the hokuyo and kinect in it
* preparing to add kinect as well as the hokuyo for navigation
* update change history
* segbot_bringup: use roslaunch_add_file_check() (`#20 <https://github.com/utexas-bwi/segbot/issues/20>`_)
* Contributors: BWI, Jack O'Quin, Piyush Khandelwal

0.1.7 (2013-09-03)
------------------

0.1.6 (2013-08-13)
------------------
* removed nodelet manager reuse for depthimage_to_laserscan for the time being
* added a script for getting the ip address
* fixed segbot_bringup to work with upcoming changes to freenect/openni launch files
* fixed tf prefix setting for gazebo openni_kinect plugin
* adding prefix enabled copy of robot_state_publisher. closes `#16 <https://github.com/utexas-bwi/segbot/issues/16>`_
* cleaned up segbot description
* renamed all internal launch files to xml. They don't show up in roslaunch any more. closes `#15 <https://github.com/utexas-bwi/segbot/issues/15>`_
* moved joint state publisher to auxiliary files - now runs on both the real robots and in simulation. closes `#14 <https://github.com/utexas-bwi/segbot/issues/14>`_
* significantly cleaned up segbot_description. need new version of xacro to be released before merging with devel. closes `#7 <https://github.com/utexas-bwi/segbot/issues/7>`_, `#12 <https://github.com/utexas-bwi/segbot/issues/12>`_

0.1.5 (2013-07-16)
------------------
* cleaned up and alphabetized cmake and package description files

0.1.4 (2013-07-13)
------------------
* releasing 0.1.4 with properly formatted changelogs. see `#10 <https://github.com/utexas-bwi/segbot/issues/10>`_
* some cmake cleanup

0.1.3 (2013-07-10)
------------------
* fixed teleop name and installation
* switched python teleop script from rosbuild to catkin
* added keyboard teleop script from teleop_twist_keyboard
* fixed launch directory location

0.1.0 (2013-06-28)
------------------
* now requires robot state publisher and joint state publisher
* catkinized segbot_bringup. closes `#6 <https://github.com/utexas-bwi/segbot/issues/6>`_
* catkinizing against hydro. progress towards `#6 <https://github.com/utexas-bwi/segbot/issues/6>`_
* all 3 kinect launch files working as expected on real hardware
* The standard segbot kinect configuration now works on real hardware
* more cleanup. will now test and fix on robot
* merging common files and more cleanup
* Merge branch 'master' of github.com:utexas-bwi/segbot
* modifying launch files and correcting typo for summer cleanup
* added parameter to disable nodelet manager if already running
* updating dependencies now that libsegwayrmp and segway_rmp have been successfully catkinized
* removing dependencies during catkinization process
* checking in configuration code not committed last time
* added a no sensor configuration. also finally fixed spelling mistake.
* a couple of bug fixes
* added a new configuration for the Pharos IRISS group
* added a robot configuration for assignment 1
* updated manifest for segbot_bringup
* the tmp file is now deletted after each use. fixed the hokuyo configuration. closes `#3 <https://github.com/utexas-bwi/segbot/issues/3>`_
* using full gazebo model by default
* added the hokuyo laser filter in the auxillary configuration of the hokuyo robot
* changed state_publisher to robot_state_publisher as suggested by warning
* a larger number of changes (added collision+material properties for gazebo, some bug fixes for hokuyo and kinect launches). Still trying to get gazebo to behave properly
* fixed up h/w launch files, separating out common simulation elements
* reogranized all the sensor launch files
* fixed a number of tf_prefix related issues
* fixed a bug where joint states were not being published on the real robots
* fixed a bug where joint states were not being published on the real robots, also introduced a hokuyo based launch file
* fixed up the kinect based segway launch file (missing the device id for now)
* some launch file reorganization
* added a separate package to hold launch files for sensors, mostly while kinect issues are sorted out
* fixing launch files inside the bringup package
* consolidated all launch files to the brringup package
* initial commit of the segbot package from the svn repository
