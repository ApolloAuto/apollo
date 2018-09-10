set (build_version "1.0.0.0")
#set(ENABLE_WARNNING "OPEN")
option(BUILD_SHARED_LIBS "" ON)

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread -fPIE -fPIC -Wno-deprecated -pipe -W -Werror -Wall -g -O2" CACHE STRING "" FORCE)

#message("Build Type:" "${CMAKE_BUILD_TYPE} ${CMAKE_CXX_FLAGS}")

set (CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install CACHE STRING "" FORCE)
set (UNIT_TEST_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/test/cybertron/unit_test CACHE STRING "" FORCE)
set (COV_HOME ${CMAKE_SOURCE_DIR}/tools/ccover)
set (CMAKE_CXX_COMPILER_CCOVER ${COV_HOME}/bin/g++ CACHE STRING "" FORCE)

if(DEFINED ENABLE_WARNNING)
    message(INFO "ENABLE_WARNNING")
    #-Wconversion-null -Wsign-compare -Wreturn-type -Wunused-variable -Wcpp")
    add_definitions("-W -Wall")
else(DEFINED ENABLE_WARNNING)
    add_definitions(-w)
endif()

