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

set(GNSS_STATUS_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/gnss_status.pb.cc")
set(CONFIG_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/config.pb.cc")
set(GPGGA_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/gpgga.pb.cc")
set(GNSS_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/gnss.pb.cc")
set(IMU_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/imu.pb.cc")
set(INS_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/ins.pb.cc")
SET(GNSS_OBS_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/gnss_raw_observation.pb.cc")

add_library(utils src/impl/utils.cpp ${CONFIG_PB_SRCS} ${GPGGA_PB_SRCS} ${GNSS_PB_SRCS} ${IMU_PB_SRCS} ${INS_PB_SRCS} ${GNSS_STATUS_PB_SRCS} ${LOCALIZATION_POSE_PB_SRCS} ${LOCALIZATION_IMU_PB_SRCS} ${LOCALIZATION_GPS_PB_SRCS} ${HEADER_PB_SRCS} ${ERROR_CODE_SRCS} ${GEOMETRY_PB_SRCS} ${GNSS_OBS_PB_SRCS})
target_link_libraries(utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_library(stream_nodelet src/stream_nodelet.cpp src/raw_stream.cpp src/impl/serial_stream.cpp src/impl/tcp_stream.cpp src/impl/udp_stream.cpp src/impl/ntrip_stream.cpp)
target_link_libraries(stream_nodelet utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_library(parser_nodelet src/parser_nodelet.cpp src/data_parser.cpp src/impl/novatel_parser.cpp)
add_dependencies(parser_nodelet novatel_msgs_generate_messages_cpp)
target_link_libraries(parser_nodelet utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} proj)

add_library(tf_broadcaster_nodelet src/tf_broadcaster_nodelet.cpp src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster_nodelet utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_executable(parser_cli tests/parser_cli.cpp src/impl/novatel_parser.cpp)
target_link_libraries(parser_cli utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_executable(test_monitor tests/test_monitor.cpp)
target_link_libraries(test_monitor utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

install(
    TARGETS stream_nodelet parser_nodelet tf_broadcaster_nodelet utils test_monitor
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
