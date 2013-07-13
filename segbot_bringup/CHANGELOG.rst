^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package segbot_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
