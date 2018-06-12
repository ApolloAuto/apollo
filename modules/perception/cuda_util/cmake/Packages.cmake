# include this file, you can get the ${include_path}s & %{lib_path}s

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

include(${CMAKE_SOURCE_DIR}/cmake/Cuda.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/Eigen.cmake)

include_directories(SYSTEM ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/../../../)
