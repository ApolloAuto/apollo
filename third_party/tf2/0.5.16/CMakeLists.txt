cmake_minimum_required(VERSION 2.8.3)
project(tf2)

find_package(console_bridge REQUIRED)
find_package(catkin REQUIRED COMPONENTS geometry_msgs rostime tf2_msgs)
find_package(Boost REQUIRED COMPONENTS signals system thread)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES tf2 
   DEPENDS console_bridge
   CATKIN_DEPENDS geometry_msgs tf2_msgs rostime)

include_directories (src/bt)
include_directories(include ${catkin_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})

# export user definitions

#CPP Libraries
add_library(tf2 src/cache.cpp src/buffer_core.cpp src/static_cache.cpp)
target_link_libraries(tf2 ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
add_dependencies(tf2 ${catkin_EXPORTED_TARGETS})

install(TARGETS tf2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Tests
if(CATKIN_ENABLE_TESTING)

catkin_add_gtest(test_cache_unittest test/cache_unittest.cpp)
target_link_libraries(test_cache_unittest tf2  ${console_bridge_LIBRARIES})
add_dependencies(test_cache_unittest ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(test_static_cache_unittest test/static_cache_test.cpp)
target_link_libraries(test_static_cache_unittest tf2  ${console_bridge_LIBRARIES})
add_dependencies(test_static_cache_unittest ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(test_simple test/simple_tf2_core.cpp)
target_link_libraries(test_simple tf2  ${console_bridge_LIBRARIES})
add_dependencies(test_simple ${catkin_EXPORTED_TARGETS})

add_executable(speed_test EXCLUDE_FROM_ALL test/speed_test.cpp)
target_link_libraries(speed_test tf2  ${console_bridge_LIBRARIES})
add_dependencies(tests speed_test)
add_dependencies(tests ${catkin_EXPORTED_TARGETS})

endif()
