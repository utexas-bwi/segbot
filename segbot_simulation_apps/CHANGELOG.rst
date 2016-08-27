^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package segbot_simulation_apps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2016-08-27)
------------------

0.3.4 (2016-08-08)
------------------
* added some code to remove redundant ros timers from the timeout queue. fixed incorrect footprint clearing.
* updated simulated calls that open the doors to automatically close those doors.
* finished + tested compilation of robot teleportation script.
* separated some common functionality in door handler so that it can be used by robot teleporter.
* added some stuff for the robot teleporter
* Contributors: Piyush Khandelwal

0.3.3 (2015-08-05)
------------------
* merge segbot_simulator packages into segbot (`#46 <https://github.com/utexas-bwi/segbot/issues/46>`_)
* Contributors: Jack O'Quin

0.3.1 (2015-03-31)
------------------

0.3.0 (2015-03-24)
------------------
* added missing dependency on multi_level_map_msgs.
* migrated DoorHandlerInterface from segbot_simulation_apps to bwi_msgs in the bwi_common repository.
* updated door handler to use new multimap.
* Contributors: Piyush Khandelwal

0.2.1 (2014-04-22)
------------------

0.2.0 (2014-04-20)
------------------
* initial release to Hydro
* install header correctly
* removed catkin_lint errors
* moved resolveDoor to common functions in bwi_planning_common
* added service to modify door status
* added a package for simulation apps. For now, added a door handler.
