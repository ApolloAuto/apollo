cmake_minimum_required(VERSION 2.8.3)
project(cybertron)

set(INCLUDE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${INCLUDE_ROOT} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR}/cybertron/tools/cvt/sensor)
enable_language(ASM)

# Cuda Environment
find_package(CUDA)

include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
# End Cuda Environment

include(${CMAKE_SOURCE_DIR}/cmake/Packages.cmake)

add_subdirectory(cybertron/proto)
add_subdirectory(cybertron/common)
add_subdirectory(cybertron/record)

add_subdirectory(cybertron/tools)
add_subdirectory(examples)
#add_subdirectory(benchmark)
add_subdirectory(cybertron/mainboard)

add_subdirectory(sensor)
add_subdirectory(third_party)

add_subdirectory(python)
add_subdirectory(cybertron/blocker)

file(GLOB CYBERTRON_SRCS
  "cybertron/*.cpp"
  "cybertron/logger/*.cpp"
  "cybertron/node/node.cpp"
  "cybertron/file_util.cpp"
  "cybertron/scheduler/*.cpp"
  "cybertron/scheduler/policy/*.cpp"
  "cybertron/data/*.cpp"
  "cybertron/component/timer_component.cpp"
  "cybertron/class_loader/*.cpp"
  "cybertron/task/*.cpp"
  "cybertron/time/*.cpp"
  "cybertron/timer/*.cpp"
  "cybertron/tf2_cybertron/*.cpp"
  "cybertron/transport/*.cpp"
  "cybertron/transport/*/*.cpp"
  "cybertron/service_discovery/*.cpp"
  "cybertron/service_discovery/*/*.cpp"
  "cybertron/croutine/*.cpp"
  "cybertron/croutine/swap.S"
  "cybertron/parameter/*.cpp"
  "cybertron/croutine/context/*.cpp"
  "cybertron/event/*.cpp"
  "cybertron/mainboard/module_argument.cpp"
  "cybertron/mainboard/module_controller.cpp"
  "cybertron/message/*.cpp"
)

add_library(cybertron SHARED
  ${CYBERTRON_SRCS}
)

target_link_libraries(cybertron
  -ldl
  protobuf
  cybertron_proto
  cybertron_common
  cybertron_blocker
  fastrtps
  fastcdr
  -lrt
  glog
  gflags
  PocoFoundation
  -Wl,--no-undefined
  atomic
  tf2
  -luuid
 )

install(TARGETS cybertron LIBRARY DESTINATION lib)
install(DIRECTORY cybertron/conf DESTINATION .)

install(DIRECTORY "cybertron"
  DESTINATION "include"
  FILES_MATCHING
  PATTERN "bin" EXCLUDE
  PATTERN "conf" EXCLUDE
  PATTERN "launch" EXCLUDE
  PATTERN "mainboard" EXCLUDE
  PATTERN "*.h"
  )

enable_testing()
#add_subdirectory(cybertron/test)
