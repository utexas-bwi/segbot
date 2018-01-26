# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['segbot_arm_manipulation'],
    package_dir={'': 'src'}
    #requires=['std_msgs', 'rospy', 'jaco_msgs', 'geometry_msgs', 'segbot_arm_manipulation', 'sensor_msgs','segbot_arm_perception']
    )

setup(**setup_args)
