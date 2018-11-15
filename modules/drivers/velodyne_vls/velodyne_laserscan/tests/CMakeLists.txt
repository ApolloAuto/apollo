### Unit tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

# These dependencies are only needed for unit testing
find_package(roslaunch REQUIRED)
find_package(rostest REQUIRED)

# C++ gtests
#catkin_add_gtest(test_calibration test_calibration.cpp)
#add_dependencies(test_calibration ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_calibration velodyne_rawdata ${catkin_LIBRARIES})

# ROS rostests
add_rostest_gtest(test_lazy_subscriber_node lazy_subscriber_node.test lazy_subscriber.cpp)
add_dependencies(test_lazy_subscriber_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_lazy_subscriber_node ${catkin_LIBRARIES})

add_rostest_gtest(test_lazy_subscriber_nodelet lazy_subscriber_nodelet.test lazy_subscriber.cpp)
add_dependencies(test_lazy_subscriber_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_lazy_subscriber_nodelet ${catkin_LIBRARIES})

add_rostest_gtest(test_system_node system_node.test system.cpp)
add_dependencies(test_system_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_system_node ${catkin_LIBRARIES})

add_rostest_gtest(test_system_nodelet system_nodelet.test system.cpp)
add_dependencies(test_system_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_system_nodelet ${catkin_LIBRARIES})

