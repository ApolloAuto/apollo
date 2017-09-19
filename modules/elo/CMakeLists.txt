project(localization_test)
cmake_minimum_required(VERSION 2.8)
IF(CMAKE_VERSION VERSION_GREATER "2.8.11")
  CMAKE_POLICY(SET CMP0022 OLD)
ENDIF()

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(/usr/local/cuda-8.0/targets/aarch64-linux/include/)
find_package(OpenCV)

add_executable(localization_test localization_test.cc)
target_link_libraries(localization_test ${PROJECT_SOURCE_DIR}/lib/libcamera_localization.so ${OpenCV_LIBS} ${PROJECT_SOURCE_DIR}/lib/libcaffe.so)
