^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package segbot_sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2015-03-14)
------------------
* use different list of filters for segbot v1 and segbot v2 due to differences between different laser models.
* added a launch file for ethernet hokuyo for segbot v2.
* added some code to process low battery messages. The robots will now attempt to send an email if they're reaching 
  low battery levels.
* big rework of all the arduino code to support the sensors on the new mounting plate.
* added some diagnostic publications.
* removed hack in nan_to_inf filter that was setting values to max_range - epsilon.
* Contributors: Jack O'Quin, Jivko Sinapov, Maxwell Svetlik, Piyush Khandelwal

0.2.1 (2014-04-24)
------------------
* fix some problems catkin_lint discovered
* segbot_sensors: make roslaunch checks complete (`#20
  <https://github.com/utexas-bwi/segbot/issues/20>`_)

0.2.0 (2014-04-17)
------------------

* Release to Hydro.
* segbot_sensors: add explicit build dependency on nodelet.
* segbot_sensors: restore missing build dependencies
  (fixes `#25 <https://github.com/utexas-bwi/segbot/issues/25>`_).
* Added udev rules for segbot, and updated the kinect laserscan
  configuration.
* Removed catkin_lint errors.
  Modify parser so a single Arduino message can provide any subset of the sensors.
* sensor_ranges_driver: get port and baud from ROS parameters
  If port is not a tty, open as a regular file containing test data.
* segbot_sensors: add explicit message dependency to CMakeLists.txt.
* segbot_sensors: add no-op ranges to cloud nodelet.
* sensor_ranges_driver: ignore non-ASCII characters sent from device.
* sensor_ranges_driver: improve shutdown and error handling.
* segbot_sensors: use roslint on Python scripts to do PEP8 checking.
* segbot_sensors: add sensor_ranges_driver for Arduino data.
* Updated descriptions and licensing information.

0.1.9 (2013-12-15)
------------------

0.1.8 (2013-12-04)
------------------
* add missing segbot_sensors dependency on depthimage_to_laserscan
  (`#20 <https://github.com/utexas-bwi/segbot/issues/20>`_).
* use ``roslaunch_add_files_check()`` to test that required launch
  file dependencies are declared.
* fixed a few minor errors found on the real robot
* added configuration with both the hokuyo and kinect in it
* preparing to add kinect as well as the hokuyo for navigation
* change nan to inf filter to handle max_range+ values as well. need to sort this all out in navigation
* update change history
* segbot_sensors: test any kinect launch files that do not us openni_launch (`#20 <https://github.com/utexas-bwi/segbot/issues/20>`_)
* add roslaunch_add_file_check() unit test (`#20 <https://github.com/utexas-bwi/segbot/issues/20>`_, `ros-drivers/openni_launch#10 <https://github.com/ros-drivers/openni_launch/issues/10>`_)
* add missing segbot_sensors dependency on depthimage_to_laserscan (`#20 <https://github.com/utexas-bwi/segbot/issues/20>`_)
* fixed issue with hydro costmap not accepting +inf range values

0.1.7 (2013-09-03)
------------------
* increased footprint where laser points are ignored. closes `#19 <https://github.com/utexas-bwi/segbot/issues/19>`_

0.1.6 (2013-08-13)
------------------
* removed nodelet manager reuse for depthimage_to_laserscan for the time being
* fixed segbot_bringup to work with upcoming changes to freenect/openni launch files
* assume no reading from kinect means clear. closes `#17 <https://github.com/utexas-bwi/segbot/issues/17>`_
* fixed cmake function ordering
* removed nan_to_inf filter from hokuyo config. closes `#11 <https://github.com/utexas-bwi/segbot/issues/11>`_

0.1.5 (2013-07-16)
------------------
* cleaned up and alphabetized cmake and package description files
* nan_to_inf temporarily sets 0 values to inf as well. see `#5 <https://github.com/ros-drivers/hokuyo_node/issues/5>`_

0.1.4 (2013-07-13)
------------------
* releasing 0.1.4 with properly formatted changelogs. see `#10 <https://github.com/utexas-bwi/segbot/issues/10>`_
* some cmake cleanup

0.1.3 (2013-07-10)
------------------
* uncommented openni_launch as it has now been released into hydro. closes `#9 <https://github.com/utexas-bwi/segbot/issues/9>`_

0.1.0 (2013-06-28)
------------------
* fixed typo in filename
* adding convenience vizualization launch file
* now using only 50 pixels for depthimage_to_laserscan
* added dependency on filters to use plugin export correctly
* freenect_launch has been released into hydro, uncommenting run_depend in package.xml
* finished catkinizing segbot_sensors `#6 <https://github.com/utexas-bwi/segbot/issues/6>`_
* catkinizing against hydro. progress towards `#6 <https://github.com/utexas-bwi/segbot/issues/6>`_
* added NanToInf to exported plugins list
* fixed filters to split off footprint exclusion from processing nan values
* converting nans from the sensor to positive infinite - feature used by new costmap_2d to assume no readings are empty
* using entire height of kinect to generate laserscan. This does mean we get some points from the ground. need to improve filters to handle this
* checking in configuration code not committed last time
* chaning laser range to front 150 degrees
* Revert "Test commit"
  This reverts commit 46c41cf9697ff40e67a750438d91d226fc34b3bd.
* Test commit
* Created launch file for USB cameras
* footprint filter now handles min and max ranges correctly
* added a new configuration for the Pharos IRISS group
* updated manifest for sensors package
* updated code to use depthimage_to_laserscan instead of pointcloud_to_laserscan
* added a launch file, changed filter name to get rid of deprecation warning, added reading of tf prefix
* bug fixes + now publishing the footprint polygon + appropriate configuration changes
* added an untested laser filter plugin for removing laser returns on the segbot polygon footprint
* fixed up h/w launch files, separating out common simulation elements
* reogranized all the sensor launch files
* a simple launch script to test naming and namespacing for the kinect
* fixed a bug in kinect.launch, also added device_id as a parameter
* some basic fixes to the hokuyo launch stuff - requires some testing on the actual hokuyo
* changed the custom version of openni.launch to use kinect_frames from freenect_launch. This allows having a top level namespace in the kinect frames as well
* fixed file permissions, also added a tf link from the base of the laser to the the laser itself
* fixed a number of tf_prefix related issues
* fixed parameter values for pointcloud_to_laserscan
* fixed up the kinect based segway launch file (missing the device id for now)
* added convenience launch script for the hokuyo
* renamed launch file argument appropriately
* directly accessing main freenect launch file (as I should have in the first place)
* some untested launch files for the kinect
* some launch file reorganization
* added a separate package to hold launch files for sensors, mostly while kinect issues are sorted out
