cmake_minimum_required(VERSION 2.8.3)
project(usb_cam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS image_transport roscpp std_msgs std_srvs sensor_msgs camera_info_manager cv_bridge)

#find_package(CUDA REQUIRED)
find_package(Boost 1.54  REQUIRED COMPONENTS system thread)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -O2 -fopenmp -mavx2 -fPIC")

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

link_directories(
  /usr/local/apollo/jsoncpp/lib
  /usr/local/apollo/adv_plat/lib
  ${catkin_LIB_DIRS}
)

include_directories(include
  /usr/local/apollo/adv_plat
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
)

# Build the USB camera library
add_library(${PROJECT_NAME} src/usb_cam.cpp)
target_link_libraries(${PROJECT_NAME}
    yaml-cpp
    libadv_trigger_ctl.a
    libjsoncpp.so
    libadv_plat_common.a
    ${avcodec_LIBRARIES}
    ${swscale_LIBRARIES}
    ${catkin_LIBRARIES}
    )

## Build the USB camera nodelet
add_library(${PROJECT_NAME}_nodelet nodes/usb_cam_nodelet.cpp nodes/usb_cam_wrapper.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet
    ${PROJECT_NAME}
    libadv_trigger_ctl.a
    libjsoncpp.so
    libadv_plat_common.a
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
)

## Declare USB camera cpp executable
add_executable(${PROJECT_NAME}_node nodes/usb_cam_node.cpp nodes/usb_cam_wrapper.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  libadv_trigger_ctl.a
  libjsoncpp.so
  libadv_plat_common.a
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_nodelet
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Copy libjsoncpp.so
install(
  DIRECTORY /usr/local/apollo/jsoncpp/lib
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/..
  FILES_MATCHING PATTERN "*.so*"
)

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
  FILES_MATCHING PATTERN "*.yaml"
)

install(FILES nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

