cmake_minimum_required(VERSION 2.8)

project(perception_util)
include(cmake/Options.cmake)
include(cmake/Packages.cmake)

file(GLOB _SRCS "*.cpp")
list(APPEND CUDA_NVCC_FLAGS -std=c++11)

set(cuda_file undistortion.cu util.cu region_output.cu network.cu)
cuda_compile(cuda_obj ${cuda_file} SHARED)

add_library(${PROJECT_NAME} SHARED ${cuda_obj} ${_SRCS})
target_link_libraries(${PROJECT_NAME})
