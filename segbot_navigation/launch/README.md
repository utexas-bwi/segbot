How to create a new map:

1) To create a new map, use the gmapping launch file for the version of BWI Bot you are using:

For version 2 use the following:
roslaunch segbot_navigation robot_with_gmapping_v2.launch

For version 3 use the following:
Before launching gmapping adjust the velodyne laser range max from 40 to 80 in the following launch file:
segbot/segbot_sensors/launch/velodyne/velodyne-laserscan.launch

roslaunch segbot_navigation robot_with_gmapping_v3.launch

This will bring up the necessary tools to create the map and visualize it as it's being computed. 

In order to driver the robot around you must also run the keyboard teleoperation node:
rosrun segbot_bringup teleop_twist_keyboard


2) Saving the map

Once you are done with the map you can save it with the following command:
rosrun map_server map_saver -f path/name

NOTE!: This only works if gmapping (one of the tools launched by the first launch file) is still active!


3) Annotating the map

To annotate the map you need to specify the path of the yaml file of the map, and a directory to store the data:

rosrun bwi_planning_common logical_marker _map_file:=mapfile.yaml _data_directory:=aDirectory
