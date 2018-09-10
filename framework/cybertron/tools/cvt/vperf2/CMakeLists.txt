cmake_minimum_required(VERSION 2.8.12)

project(cyber_vperf2)

#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
#set(CMAKE_CXX_FLAGS "-std=c++11 -pthread -fPIE -fPIC -Wno-deprecated -pipe " CACHE STRING "" FORCE)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

set(CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/third_party/Qt5.5.1/5.5/gcc_64)
find_package(Qt5 REQUIRED COMPONENTS Gui Widgets Core)

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

install(TARGETS ${PROJECT_NAME}
DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

