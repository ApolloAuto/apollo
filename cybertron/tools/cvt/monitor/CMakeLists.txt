cmake_minimum_required(VERSION 2.8.12)
project(cyber_monitor)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(OUTTER_LIBRARIES
    cybertron
    cybertron_proto
    cybertron_common
    fastrtps
    console_bridge
    protobuf
    glog
    pthread
    ncurses
)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB MSG_SRC ${CMAKE_CURRENT_SOURCE_DIR}/messages/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${MSG_SRC})
target_link_libraries(${PROJECT_NAME} ${OUTTER_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
