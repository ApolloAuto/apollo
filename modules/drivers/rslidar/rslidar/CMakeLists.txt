cmake_minimum_required(VERSION 2.8.3)
project(rslidar)
find_package(catkin REQUIRED)
catkin_package(
    CATKIN_DEPENDS)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
