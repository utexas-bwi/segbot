cmake_minimum_required(VERSION 2.8.3)
project(segbot_firmware)

find_package(catkin REQUIRED)

catkin_package()

# TODO: figure out how to automate Arduino cross-compile and loading.

# include firmware sources in the distribution package
install(DIRECTORY src
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
