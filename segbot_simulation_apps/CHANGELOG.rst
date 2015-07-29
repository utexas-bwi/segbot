^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package segbot_simulation_apps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
