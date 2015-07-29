^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package segbot_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2015-03-31)
------------------
* Allow explicit amcl laser max range paramter instantiation. closes `#31 <https://github.com/utexas-bwi/segbot_apps/issues/31>`_.
* Closes `#29 <https://github.com/utexas-bwi/segbot_apps/issues/29>`_
  - Get latest version of move_base to get fix introduced in https://github.com/ros-planning/navigation/pull/295
  This allows setting tolerance to 0 when calling make_plan from segbot_logical_translator.
  - Don't use any tolerance when testing whether a door was open or not.
  - Don't clear costmap around robot when testing for an open door.
* Increase local costmap size to 8m x 8m. Increasing obstacle and raytrace range for primary laser to match. Closes `#26 <https://github.com/utexas-bwi/segbot_apps/issues/26>`_.
* updated planner frequency to 1.0. Closes `#25 <https://github.com/utexas-bwi/segbot_apps/issues/25>`_
* Contributors: Piyush Khandelwal

0.3.0 (2015-03-24)
------------------
* a number of navigation changes to allow for correct navigation with multiple namespaced robots, and use the multimap.
* fixed a bunch of catkin_lint errors. also installed missing scripts. closes `#23 <https://github.com/utexas-bwi/segbot_apps/issues/23>`_
* added better approach point determination. only navigate via the current location
* Fixed nan_to_inf filter, still testing laser on new robot.
* revised map_server's node name to make it unique
* tuned many of the amcl parameters to make navigation better.
* Made the door checker test for the door to be open 3 times before going through. Works much better.
* Contributors: BWI, Jivko Sinapov, Piyush Khandelwal, Shiqi Zhang, bwi

0.2.1 (2014-04-22)
------------------

0.2.0 (2014-04-19)
------------------
* use ``roslaunch_add_files_check()`` to test that required launch
  file dependencies are declared.

0.1.5 (2013-09-03)
------------------
* closes `#8 <https://github.com/utexas-bwi/segbot_apps/issues/8>`_

0.1.4 (2013-08-12)
------------------
* added a real world navigation file
* updated and cleaned up visualization config
* cleaned up launch and configuration files. closes `#6 <https://github.com/utexas-bwi/segbot_apps/issues/6>`_
* removed confusing map_namespace parameter
* added map_topic parameter for multi-robot scenarios

0.1.3 (2013-07-16)
------------------
* added missing dependencies (closes `#4 <https://github.com/utexas-bwi/segbot_apps/issues/4>`_). Cleaned up package and cmake files.
* changed costmap visualization to occupancy grid in rviz configuration

0.1.2 (2013-07-13)
------------------
* removed dependency on navigation meta-package. progress towards `#3 <https://github.com/utexas-bwi/segbot_apps/issues/3>`_

0.1.1 (2013-07-10)
------------------
* navigation has been released as a system dependency

0.1.0 (2013-06-28)
------------------
* removed redundant doc file
* uncommented runtime launch dependencies on navigation and eband_local_planner as they have not been released into hydro yet
* catkinized segbot_apps
* updating the eband visualization configuration for hydro-devel
* increased footprint size to produce an inscribed radius of 0.3
* fixed footprint location while waiting for `ros-planning/navigation#63 <https://github.com/ros-planning/navigation/issues/63>`_ to be fixed
* commenting out hydro-devel navigation test code. This should not be checked in until navigation through hydro-devel is fixed
* changes to prepare for the catkinization of eband_local_planner against hydro-devel in navigation
* some improvements to navigation
* fixed a bug in the eband trajectory controller
* in-place rotation at goal now supported
* merged goal tolerance parameters between local planner and trajectory controller.
* add launch for e-band navigation
* fixed for the regular nav stack launch file as well. closes `#1 <https://github.com/utexas-bwi/segbot_apps/issues/1>`_
* hmm not sure why this file was here
* fix for the eband costmap having an incorrect topic. `#1 <https://github.com/utexas-bwi/segbot_apps/issues/1>`_
* updated launch file to use any visualization configuration + reorganized eband configuration file
* checking in new parameters for the eband local planner
* inital differential drive trajectory controller - looks pretty good. needs a bit more code improvent, dynamic reconfigure and stricter obstacle testing
* removed some unnecessary launch files and added an rviz configuration + launch file for testing autonomous navigation
* basic amcl + move base demo works (but is not very good)
* removed old ens basement maps from the repo
* removed joy gmapping file - joystick control not directly supported
* removed redundant sensor files (moved to segbot_sensors)
* initial port of of navigation and controller code from the svn repository
