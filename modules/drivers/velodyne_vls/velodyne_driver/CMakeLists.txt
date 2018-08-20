cmake_minimum_required(VERSION 2.8.3)
project(velodyne_driver)

set(${PROJECT_NAME}_CATKIN_DEPS 
    diagnostic_updater
    dynamic_reconfigure
    nodelet
    roscpp
    tf
    velodyne_msgs)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

# This driver uses Boost threads
find_package(Boost REQUIRED COMPONENTS thread)

# libpcap provides no pkg-config or find_package module:
set(libpcap_LIBRARIES -lpcap)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Generate dynamic_reconfigure server
generate_dynamic_reconfigure_options(cfg/VelodyneNode.cfg)

# objects needed by other ROS packages that depend on this one
catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
               INCLUDE_DIRS include
               LIBRARIES velodyne_input)

# compile the driver and input library
add_subdirectory(src/lib)
add_subdirectory(src/driver)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_velodyne.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(PROGRAMS src/vdump
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)

  # these dependencies are only needed for unit testing
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  # Download packet capture (PCAP) files containing test data.
  # Store them in devel-space, so rostest can easily find them.
  catkin_download_test_data(
    ${PROJECT_NAME}_tests_class.pcap
    http://download.ros.org/data/velodyne/class.pcap
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 65808d25772101358a3719b451b3d015)
  catkin_download_test_data(
    ${PROJECT_NAME}_tests_32e.pcap
    http://download.ros.org/data/velodyne/32e.pcap
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 e41d02aac34f0967c03a5597e1d554a9)
  catkin_download_test_data(
    ${PROJECT_NAME}_tests_vlp16.pcap
    http://download.ros.org/data/velodyne/vlp16.pcap
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 f45c2bb1d7ee358274e423ea3b66fd73)
  
  # unit tests
  add_rostest(tests/pcap_node_hertz.test)
  add_rostest(tests/pcap_nodelet_hertz.test)
  add_rostest(tests/pcap_32e_node_hertz.test)
  add_rostest(tests/pcap_32e_nodelet_hertz.test)
  add_rostest(tests/pcap_vlp16_node_hertz.test)
  add_rostest(tests/pcap_vlp16_nodelet_hertz.test)
  
  # parse check all the launch/*.launch files
  roslaunch_add_file_check(launch)

endif (CATKIN_ENABLE_TESTING)
