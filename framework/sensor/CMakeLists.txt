cmake_minimum_required(VERSION 2.8.3)
project(apollo_sensor)

add_subdirectory(proto)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

file(GLOB_RECURSE SENSOR_SRCS
  "*.cpp"
)

add_library(apollo_sensor SHARED
  ${SENSOR_SRCS}
)
target_link_libraries(apollo_sensor sensor_proto protobuf cybertron glog yaml-cpp)

install(TARGETS apollo_sensor LIBRARY DESTINATION lib)
install(DIRECTORY conf DESTINATION .)
install(DIRECTORY dag DESTINATION .)
