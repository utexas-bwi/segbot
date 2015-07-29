^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package segbot_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2015-03-31)
------------------
* updated setting amcl laser max range. see segbot_apps`#31 <https://github.com/utexas-bwi/segbot_simulator/issues/31>`_.
* Contributors: Piyush Khandelwal

0.3.0 (2015-03-24)
------------------
* use the updated segbot_v1 hokuyo only launch file as default.
* now set yaw correctly for true localization as well
* use different eband parameters in simulation.
* allow changing the move base server.
* Contributors: Piyush Khandelwal

0.2.1 (2014-04-22)
------------------

0.2.0 (2014-04-20)
------------------
* use ``roslaunch_add_file_check()`` to test launch file
  dependencies.
* add missing ``map_server`` dependency.

0.1.5 (2013-09-03)
------------------
* mark config dir for installation. closes `#16 <https://github.com/utexas-bwi/segbot_simulator/issues/16>`_

0.1.4 (2013-08-13)
------------------
* reflected launch file location change for auxiliary segbot files

0.1.3 (2013-08-12)
------------------
* fixed markers and laserscan in the multiple segbot viz configuration
* finalized the multiple segbots script. made use_full_gazebo_model a parameter
* cleaned up segbot_mobile_base (tf_prefix, fake_localization) - all tested using the multiple segbots launch file. closes `#8 <https://github.com/utexas-bwi/segbot_simulator/issues/8>`_. closes `#9 <https://github.com/utexas-bwi/segbot_simulator/issues/9>`_
* updated single robot navigation script to use internal components of segbot_mobile_base
* renamed bwi_test_world to segbot_test_world. closes `#10 <https://github.com/utexas-bwi/segbot_simulator/issues/10>`_
* removed segbot_gazebo_plugins. closes `#7 <https://github.com/utexas-bwi/segbot_simulator/issues/7>`_

0.1.2 (2013-07-16)
------------------
* cmake and package description cleanup and alphabetization

0.1.1 (2013-07-15)
------------------

0.1.0 (2013-07-10)
------------------
* removing fake amcl, as an upstream version exists somewhere
* added complete navigation solution
* changing launch file permissions to execute
* removed redundant lines of code and added segbot_navigation for a complete navigation solution launch file
* upgraded test world to SDF 1.4
* fixed bwi_test_world to work with gazebo_ros_pkgs for hydro transition
* catkinization complete
* fixed cmake and launch errors for gazebo_ros_pkgs migration
* fixing auxiliary file location
* piyushk: fixed typo error in launch file
* fixing default auxilliary file name
* defaulting to starting at 0,0 - now that we are approaching a fix towards not being able to spawn the model at any location apart from 0,0
* add a simple simulation launch file
* reduced size of unit objects to use them for obstacle avoidance testing
* added a map and map server launch file for the bwi_test_world gazebo world
* hammer on the table was causing problems - it has been removed
* the full gazebo model is enabled by default - the simple model should only be used in certain experiments
* fixing launch scripts for nvidia optimus launch prefix
* removed extra launch files before correcting autonomous navigation and gmapping
* changed default world name to gazebo
* finished test world file
* a bit more cleanup
* a working test gazebo environment
* removed old compile rules for BWI controllers that should not be in the segbot package, and fix the plugins dependency
* generate a SDF1.2 world file
* trying to fix the main segway simulation launch file. work still in progress
* merged the debug and world launch files
* importing relecant files from svn repository
