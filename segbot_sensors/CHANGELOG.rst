^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package segbot_sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
