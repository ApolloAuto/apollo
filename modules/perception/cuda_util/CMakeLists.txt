cmake_minimum_required(VERSION 2.8)

project(cuda_util)
include(cmake/Options.cmake)
include(cmake/Packages.cmake)

file(GLOB _SRCS "*.cc")
list(APPEND CUDA_NVCC_FLAGS -Wno-deprecated-gpu-targets -std=c++11)

set(cuda_file undistortion.cu util.cu region_output.cu network.cu connected_component_gpu.cu block_uf.cu)
cuda_compile(cuda_obj ${cuda_file} SHARED)

include_directories(/usr/local/cuda/targets/x86_64-linux/include/)

add_library(${PROJECT_NAME} SHARED ${cuda_obj} ${_SRCS})
target_link_libraries(${PROJECT_NAME})
