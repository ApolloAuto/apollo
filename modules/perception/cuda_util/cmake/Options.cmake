# Perception Version Info, will be compiled in binary.
set(build_version "1.0.0.0")

######################## enviroment      #####################################################
set (CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/build/install CACHE STRING "" FORCE)
                                                                # set default install dir
set (CMAKE_CXX_FLAGS "-std=c++11 -g -O2 -DNDEBUG -fopenmp -Wall" CACHE STRING "" FORCE)
                                                                # -O2 need to be added back
                                                                # DO NOT remove -g flags,
                                                                # we should reserve symbol
                                                                # info to debug coredump  file.

#set (CMAKE_CXX_COMPILER /usr/bin/g++ CACHE STRING "" FORCE)

######################## option switches #####################################################
SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};-cudart=static")

set(BUILD_SHARED_LIBS         OFF)       # add_library(...) -> .os/.a
