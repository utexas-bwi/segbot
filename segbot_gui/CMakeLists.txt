cmake_minimum_required(VERSION 2.8.3)
project(segbot_gui)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/question_dialog_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
