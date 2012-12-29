rosrun xacro xacro.py $1 > test.urdf && cat test.urdf && rosrun urdf check_urdf test.urdf && roslaunch urdf_tutorial display.launch model:=test.urdf
rm -f test.urdf
