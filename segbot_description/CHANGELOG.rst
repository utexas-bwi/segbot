^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package segbot_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.2 (2015-03-31)
------------------
* added sensor plate and sonars to tf tree for segbot v2.
* make standoff urdf more generic. dropped the 632 suffix and added radius as a parameter
* Contributors: Piyush Khandelwal

0.3.1 (2015-03-24)
------------------
* removed redundant segbot v1 models.
* added new segbot v2 model URDF.
* Contributors: Piyush Khandelwal

0.3.0 (2015-03-14)
------------------
* modified simple robot model in gazebo to be more accurate while still being able to detect other robots.
* temp update to change range of hokuyo in simulation.
* added the kinect.dae mesh that was missing in the meshes folder of segbot_description package
* Contributors: Jivko Sinapov, Piyush Khandelwal

0.2.1 (2014-04-24)
------------------
* fix some problems catkin_lint discovered

0.2.0 (2014-04-17)
------------------

* Release to Hydro.
* segbot_description: correct height of hokuyo in other robots.
* segbot_description: coordinate corrections for sensor_mount and
  hokuyo.
* Add sensor mount top plate, still approximate.
* segbot_description: add approximate location of sensor mount.
* segbot_description: add chassis mount plate.
* Add launch file for displaying xacro files.
* Updated descriptions and licensing information

0.1.9 (2013-12-15)
------------------

0.1.8 (2013-12-04)
------------------
* added configuration with both the hokuyo and kinect in it
* segbot_description: add empty new change entry
* Contributors: Jack O'Quin, Piyush Khandelwal

0.1.7 (2013-09-03)
------------------
* increasing inertia to keep robot vertical when using gazebo_ros_planar_move

0.1.6 (2013-08-13)
------------------
* increased model height to make the robots more visible in gazebo
* added inertial and collision changes that make the robot move properly under the simple model assumption in gazebo. The collision model of the robot no longer receives returns from the laser.
* significantly cleaned up segbot_description. need new version of xacro to be released before merging with devel. closes `#7 <https://github.com/utexas-bwi/segbot/issues/7>`_, `#12 <https://github.com/utexas-bwi/segbot/issues/12>`_
* moved plugins to new version in gazebo_plugins. closes `#13 <https://github.com/utexas-bwi/segbot/issues/13>`_

0.1.5 (2013-07-16)
------------------
* cleaned up and alphabetized cmake and package description files
* reverting visual plugin change until pull request makes it to gazebo. See `#21 <https://bitbucket.org/osrf/sdformat/pull-request/21/patch-to-allow-parser_urdf-to-parse-visual/diff>`_

0.1.4 (2013-07-13)
------------------
* releasing 0.1.4 with properly formatted changelogs. see `#10 <https://github.com/utexas-bwi/segbot/issues/10>`_
* some cmake cleanup

0.1.3 (2013-07-10)
------------------
* URDF now sets all parameters for diff_drive plugin, removing unneeded warnings
* reverted robot radius

0.1.0 (2013-06-28)
------------------
* now using only 50 pixels for depthimage_to_laserscan
* xacro is only required by launch files
* catkinizing against hydro. progress towards `#6 <https://github.com/utexas-bwi/segbot/issues/6>`_
* ported diff drive plugin to gazebo_ros_pkgs. Cosmetic changes to the plugin are reflected here
* reverting laptop fix while patch propogates to system gazebo release
* fixed library name for differential drive plugin
* video plugin works as expected with patched gazebo
* removing the object controller plugin for the robot with no sensors
* raising height of hokuyo to something that might be inaccurate. I am currently seeing returns from wheels in simulation
* removing segbot_iriss and readding segbot (i.e. w/o sensors) configuration file
* chaning hokuyo to only see the front 150 degrees
* The standard segbot kinect configuration now works on real hardware
* temporarily bypassing visual plugin hanndling as visual plugins are not handled in gazebo
* commenting out video controller while gazebo issues being sorted out
* updating dependencies now that libsegwayrmp and segway_rmp have been successfully catkinized
* fixed bug while converting to sdf
* updating all plugins to sdf
* moved running ros plugins to top-level urdf, as these are not necessarily required for every experiment
* reduced the radius of the robot further
* reduced model size for the simple robot model
* updated segbot hokuyo configuration to inverted lidar position
* updating xml syntax based on current status of patch
* added reference to plugin inside the laptop
* added a laptop to the robot for visualization purposes -- also converted battery box to its own separate urdf file
* using the significantly improved (unpublished) object controller plugin from bwi_gazebo_entities for the simple controller
* added a no sensor configuration. also finally fixed spelling mistake.
* added a new configuration for the Pharos IRISS group
* added a robot configuration for assignment 1
* updated the manifest for segbot description
* the tmp file is now deletted after each use. fixed the hokuyo configuration. closes `#3 <https://github.com/utexas-bwi/segbot/issues/3>`_
* using full gazebo model by default
* lowered height of hokuyo for simulation
* fixed collision issues between simple and full model
* some minor urdf cleanup
* merged the 2 segbot plugins
* reenabled transmission - now gazebo produces the correct joint states - useful in rviz visualization
* added the kinect frames to the urdf description - easier for both h/w and simulation
* fixed a number of bugs in the kinect gazebo configuration
* fixed some surface properties, removed transmission as recommended by ros wiki page
* changed mesh units from inches to meters, effectively magnifying them 40x. This way the scaling in the urdf makes sense in both ROS and gazebo
* still trying to get simulator rendering to work
* a larger number of changes (added collision+material properties for gazebo, some bug fixes for hokuyo and kinect launches). Still trying to get gazebo to behave properly
* a bit more cleaning up of the urdf file description
* more cleanup in the gazebo launch files
* minor change in color to get some depth in rviz
* a bit more file renaming
* some relocation/renaming of the gazebo specific urdf files
* constructed a mesh model for the hokuyo and replaced it in the urdf file. Now the hokuyo needs a separate frame for the laser location
* added some constants for appropriate values (i.e. values used in the final robot urdf files)
* you can obtain the full gazebo model via a script -- the script itself can be improved still
* modified default configurations to use new base and chassis macros
* finished the entire segbot chassis
* fixed a minor mistake in the Kinect mount, added a mount description for the hokuyo as well - still missing a mesh model for the hokuyo
* removing accidentally commited test urdf file
* prepared basic component urdf files + created kinect mount
* fixed some inconsistencies in frame ids for the kinect based robot
* the segbot description package has been fixed - at least to a good enough point
* fixed the kinect based robot description files
* fixed a small typo in the hokuyo description file, plus introduced kinect files (copied from pr2_description for now). removed the pr2_description dependency
* moved final segbot instantiations to separate robots directory
* more cleanup
* added urdf description for hokuyo 04lx (based on hokuyo 30lx from pr2_description with minor changes), removed the kinect urdf (as pr2_description has a better version that can be used as is
* consolidated all launch files to the brringup package
* initial commit of the segbot package from the svn repository
