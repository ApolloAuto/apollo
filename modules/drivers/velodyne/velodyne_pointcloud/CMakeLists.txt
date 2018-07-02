cmake_minimum_required(VERSION 2.8.3)
project(velodyne_pointcloud)

set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    velodyne_driver
    velodyne_msgs
    eigen_conversions)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
include_directories(include ${catkin_INCLUDE_DIRS})


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
    add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -fopenmp")

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    DEPENDS yaml-cpp python-yaml
    INCLUDE_DIRS include
    LIBRARIES velodyne_parser)

add_subdirectory(src/lib)

######################
#       cloud        #
######################
add_executable(convert_node src/convert_node.cpp src/convert.cpp)
target_link_libraries(convert_node
    velodyne_parser
	${catkin_LIBRARIES})

add_library(convert_nodelet src/convert_nodelet.cpp src/convert.cpp)
target_link_libraries(convert_nodelet
    velodyne_parser
	${catkin_LIBRARIES})

######################
#  pointcloud_dump   #
######################
add_executable(pointcloud_dump_node src/pointcloud_dump_node.cpp src/pointcloud_dump.cpp)
target_link_libraries(pointcloud_dump_node
    velodyne_parser
	${catkin_LIBRARIES})

add_library(pointcloud_dump_nodelet src/pointcloud_dump_nodelet.cpp src/pointcloud_dump.cpp)
target_link_libraries(pointcloud_dump_nodelet
    velodyne_parser
	${catkin_LIBRARIES})

######################
#    pcd_exporter    #
######################
add_executable(pcd_exporter_node src/pcd_exporter_node.cpp src/pcd_exporter.cpp)
target_link_libraries(pcd_exporter_node
	${catkin_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_COMMON_LIBRARIES})

add_library(pcd_exporter_nodelet src/pcd_exporter_nodelet.cpp src/pcd_exporter.cpp)
target_link_libraries(pcd_exporter_nodelet
	${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

######################
#     compensator    #
######################
add_library(compensator_node src/compensator_nodelet.cpp src/compensator.cpp)
target_link_libraries(compensator_node
	${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

add_library(compensator_nodelet src/compensator_nodelet.cpp src/compensator.cpp)
target_link_libraries(compensator_nodelet
	${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

catkin_install_python(PROGRAMS
    src/extrinsics_broadcaster.py
    src/velodyne_check.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
    TARGETS
    convert_node
    compensator_node
    pcd_exporter_node
    pointcloud_dump_node
    convert_nodelet
    compensator_nodelet
    pcd_exporter_nodelet
    pointcloud_dump_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)
install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
