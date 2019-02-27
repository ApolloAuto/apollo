cmake_minimum_required(VERSION 2.8.3)
project(tf2)
# set (CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/build/install CACHE STRING "" FORCE)
# set (UNIT_TEST_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/tests CACHE STRING "" FORCE)
# set(CMAKE_CXX_FLAGS "-std=c++11 -pthread -fPIE -fPIC -Wno-deprecated -pipe -W -Werror -Wall -g -O2" CACHE STRING "" FORCE)

find_package(Boost REQUIRED COMPONENTS signals system thread)
include_directories(include)

# export user definitions
#CPP Libraries
add_library(tf2 SHARED src/cache.cpp src/buffer_core.cpp src/static_cache.cpp src/time.cpp)
target_link_libraries(tf2 ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  DESTINATION third_party)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib)

install(DIRECTORY include
  DESTINATION ${CMAKE_INSTALL_PREFIX})

install(DIRECTORY include
  DESTINATION ${CMAKE_INSTALL_PREFIX}/third_party)

# Tests
add_subdirectory(test)
