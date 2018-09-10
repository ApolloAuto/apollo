
set(BAIDU_ROOT "${CMAKE_SOURCE_DIR}/../../..")

# list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

include(${CMAKE_SOURCE_DIR}/cmake/Options.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/Protobuf.cmake)
#include(${CMAKE_SOURCE_DIR}/cmake/Functions.cmake)

include(${CMAKE_SOURCE_DIR}/cmake/GTest.cmake)
#include(${CMAKE_SOURCE_DIR}/cmake/Python.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/GLog.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/Fastrtps.cmake)
#include(${CMAKE_SOURCE_DIR}/cmake/Console_bridge.cmake)
#include(${CMAKE_SOURCE_DIR}/cmake/Boost.cmake)

# include(${CMAKE_SOURCE_DIR}/cmake/Poco.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/TInfo.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/Ncurses.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/GFlags.cmake)
include_directories(${CMAKE_SOURCE_DIR})
include(${CMAKE_SOURCE_DIR}/cmake/TF2.cmake)
