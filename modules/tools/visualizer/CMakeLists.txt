cmake_minimum_required(VERSION 2.8.12)

project(cyber_visualizer)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

set(CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/third_party/Qt5.5.1/5.5/gcc_64)
find_package(Qt5 REQUIRED COMPONENTS Gui Widgets Core)

set(OUTTER_LIBRARIES
    cybertron
    cybertron_proto
    cybertron_common
    sensor_msgs
    fastrtps
    console_bridge
    protobuf
    glog
    pthread
    GL
)

include_directories(.)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB UIS ${CMAKE_CURRENT_SOURCE_DIR}/*.ui)
file(GLOB RESOURCE ${CMAKE_CURRENT_SOURCE_DIR}/*.qrc)

qt5_wrap_ui(WRAP_UIS ${UIS})
#qt5_wrap_cpp(MOC widget.h)
qt5_add_resources(RCC ${RESOURCE})

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${UIS} ${RCC})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
target_link_libraries(${PROJECT_NAME} Qt5::Gui)
target_link_libraries(${PROJECT_NAME} Qt5::Core)
target_link_libraries(${PROJECT_NAME} ${OUTTER_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROTO_SRC_DIR})

install(TARGETS ${PROJECT_NAME}
DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

