cmake_minimum_required(VERSION 2.8.3)
project(lslidar_driver)

add_definitions(-std=c++0x)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  lslidar_msgs
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    nodelet
    lslidar_msgs
  DEPENDS
    boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

# Leishen c16 lidar driver
add_library(lslidar_driver
  src/lslidar_driver.cc
)
target_link_libraries(lslidar_driver
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Leishen c16 lidar nodelet
add_executable(lslidar_driver_node
  src/lslidar_driver_node.cc
)
target_link_libraries(lslidar_driver_node
  lslidar_driver
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_driver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Leishen c16 lidar nodelet
add_library(lslidar_driver_nodelet
  src/lslidar_driver_nodelet.cc
)
target_link_libraries(lslidar_driver_nodelet
  lslidar_driver
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_driver_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


# install options
install(
    TARGETS
    lslidar_driver
    lslidar_driver_node
    lslidar_driver_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_lslidar.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
