cmake_minimum_required(VERSION 2.8.3)
project(lslidar_compensator)

add_definitions(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  angles
  roscpp
  pluginlib
  sensor_msgs
  pcl_ros
  pcl_conversions
  lslidar_msgs
  nodelet
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp sensor_msgs pluginlib nodelet
    pcl_ros pcl_conversions
    lslidar_msgs
    eigen_conversions
    angles
  DEPENDS
    Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

# Lslidar C16 compensator node
add_executable(lslidar_compensator_node src/compensator_node.cpp src/compensator.cpp)
add_dependencies(lslidar_compensator_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lslidar_compensator_node
	${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

add_library(lslidar_compensator_nodelet src/compensator_nodelet.cpp src/compensator.cpp)
target_link_libraries(lslidar_compensator_nodelet
	${catkin_LIBRARIES}
    ${PCL_LIBRARIES})
add_dependencies(lslidar_compensator_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
