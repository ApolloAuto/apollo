cmake_minimum_required(VERSION 2.8.3)
project(gnss_driver)

find_package(catkin REQUIRED COMPONENTS
    nodelet
    roscpp
    std_msgs
    tf2_ros
)

catkin_package(
   LIBRARIES ${catkin_LIBRARIES} roscpp
   CATKIN_DEPENDS roscpp std_msgs tf2_ros
   DEPENDS  protobuf
)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-g -pipe -W -Wall -Wno-sign-compare -Wno-unused-parameter -fPIC -O2 -Wno-parentheses -rdynamic -std=c++11")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

find_package(Proj)
if(DEFINED PROTOBUF_LIBRARIES)
    MESSAGE(STATUS "proto: " ${PROTOBUF_LIBRARIES})
else()
    set(PROTOBUF_LIBRARIES "protobuf")
    MESSAGE(STATUS "proto: " ${PROTOBUF_LIBRARIES})
endif()

set(MODULE_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../")
set(COMMON_PROTO_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/common/proto/")
include_directories(${MODULE_ROOT_DIR})
include_directories(${COMMON_PROTO_INCLUDE_DIR})

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
link_directories(lib)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(ERROR_CODE_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/common/proto/error_code.pb.cc")
set(HEADER_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/common/proto/header.pb.cc")
set(GEOMETRY_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/common/proto/geometry.pb.cc")

set(LOCALIZATION_POSE_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/localization/proto/pose.pb.cc")
set(LOCALIZATION_GPS_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/localization/proto/gps.pb.cc")
set(LOCALIZATION_IMU_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/localization/proto/imu.pb.cc")

FILE(GLOB GNSS_PROTO_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.pb.cc")

add_library(rtcm third_party/rtcm3.c third_party/rtcm.c third_party/rtkcmn.c third_party/novatel.c third_party/rcvraw.c)
target_link_libraries(rtcm m)

add_library(utils src/util/utils.cpp ${GNSS_PROTO_SRCS} ${LOCALIZATION_POSE_PB_SRCS} ${LOCALIZATION_IMU_PB_SRCS} ${LOCALIZATION_GPS_PB_SRCS} ${HEADER_PB_SRCS} ${ERROR_CODE_SRCS} ${GEOMETRY_PB_SRCS})
target_link_libraries(utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_library(stream_nodelet src/stream/stream_nodelet.cpp src/stream/raw_stream.cpp src/impl/stream/serial_stream.cpp src/impl/stream/tcp_stream.cpp src/impl/stream/udp_stream.cpp src/impl/stream/ntrip_stream.cpp)
target_link_libraries(stream_nodelet utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_library(parser_nodelet src/parser/parser_nodelet.cpp src/parser/data_parser.cpp src/impl/parser/novatel/novatel_parser.cpp)
add_dependencies(parser_nodelet novatel_msgs_generate_messages_cpp rtcm)
target_link_libraries(parser_nodelet rtcm utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} proj)

add_library(rtcm_parser_nodelet src/parser/rtcm_parser_nodelet.cpp src/parser/rtcm_parser.cpp src/impl/parser/rtcm/rtcm3_parser.cpp)
add_dependencies(rtcm_parser_nodelet novatel_msgs_generate_messages_cpp rtcm)
target_link_libraries(rtcm_parser_nodelet rtcm utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_library(tf_broadcaster_nodelet src/tf/tf_broadcaster_nodelet.cpp src/tf/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster_nodelet utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_executable(parser_cli tests/parser_cli.cpp src/impl/parser/novatel/novatel_parser.cpp)
target_link_libraries(parser_cli rtcm utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_executable(test_monitor tests/test_monitor.cpp)
target_link_libraries(test_monitor utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

install(
    TARGETS stream_nodelet parser_nodelet rtcm_parser_nodelet tf_broadcaster_nodelet utils rtcm
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY conf/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/conf)
