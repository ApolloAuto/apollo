cmake_minimum_required(VERSION 3.5)
project(tf2 CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-fPIC -DNDEBUG -O3 -pipe -Wall")

# set (CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/build/install CACHE STRING "" FORCE)
# set (UNIT_TEST_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/tests CACHE STRING "" FORCE)
# set(CMAKE_CXX_FLAGS "-std=c++11 -pthread -fPIE -fPIC -Wno-deprecated -pipe -W -Werror -Wall -g -O2" CACHE STRING "" FORCE)

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories( 
	include
	${Boost_INCLUDE_DIR} # for signals2 is header only
)

option(BUILD_TESTS "Build the tests" ON)

# export user definitions
#CPP Libraries
add_library(tf2 SHARED
	src/cache.cpp
	src/buffer_core.cpp
	src/static_cache.cpp
	src/time.cpp
)

target_link_libraries(tf2
	${Boost_LIBRARIES}
)

# ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION
    ${CMAKE_INSTALL_PREFIX}/lib
)

install(DIRECTORY include DESTINATION
  ${CMAKE_INSTALL_PREFIX}
)

install(FILES README.md DESTINATION
  ${CMAKE_INSTALL_PREFIX}
)

if (BUILD_TESTS)
    include(cmake/GTest.cmake)
    fetch_googletest(
        ${PROJECT_SOURCE_DIR}/cmake
        ${PROJECT_BINARY_DIR}/googletest
    )
    enable_testing()
    add_subdirectory(test)
endif()
