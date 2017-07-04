cmake_minimum_required(VERSION 2.8.3)
project(gnss_driver)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
)

catkin_package(
   LIBRARIES ${catkin_LIBRARIES} roscpp
   CATKIN_DEPENDS roscpp std_msgs
   DEPENDS  protobuf
)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-g -pipe -W -Wall -Wno-sign-compare -Wno-unused-parameter -fPIC -O2 -Wno-parentheses -rdynamic -std=c++11")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

find_package(Proj)
IF (DEFINED PROTOBUF_LIBRARIES)
    MESSAGE(STATUS "proto: " ${PROTOBUF_LIBRARIES})
ELSE()
    set(PROTOBUF_LIBRARIES "protobuf")
    MESSAGE(STATUS "proto: " ${PROTOBUF_LIBRARIES})
ENDIF (defined PROTOBUF_LIBRARIES)

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

SET(ERROR_CODE_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/common/proto/error_code.pb.cc")
SET(HEADER_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/common/proto/header.pb.cc")
SET(GEOMETRY_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/common/proto/geometry.pb.cc")

SET(LOCALIZATION_POSE_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/localization/proto/pose.pb.cc")
SET(LOCALIZATION_GPS_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/localization/proto/gps.pb.cc")
SET(LOCALIZATION_IMU_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/localization/proto/imu.pb.cc")

SET(GNSS_STATUS_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/gnss_status.pb.cc")
SET(CONFIG_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/config.pb.cc")
SET(GPGGA_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/gpgga.pb.cc")
SET(GNSS_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/gnss.pb.cc")
SET(IMU_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/imu.pb.cc")
SET(INS_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/ins.pb.cc")

add_library(utils src/impl/utils.cpp ${CONFIG_PB_SRCS} ${GPGGA_PB_SRCS} ${GNSS_PB_SRCS} ${IMU_PB_SRCS} ${INS_PB_SRCS} ${GNSS_STATUS_PB_SRCS} ${LOCALIZATION_POSE_PB_SRCS} ${LOCALIZATION_IMU_PB_SRCS} ${LOCALIZATION_GPS_PB_SRCS} ${HEADER_PB_SRCS} ${ERROR_CODE_SRCS} ${GEOMETRY_PB_SRCS})
target_link_libraries(utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_library(stream_nodelet src/stream_nodelet.cpp src/raw_stream.cpp src/impl/serial_stream.cpp src/impl/tcp_stream.cpp src/impl/udp_stream.cpp src/impl/ntrip_stream.cpp)
target_link_libraries(stream_nodelet utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_library(parser_nodelet src/parser_nodelet.cpp src/data_parser.cpp src/impl/novatel_parser.cpp)
add_dependencies(parser_nodelet novatel_msgs_generate_messages_cpp)
target_link_libraries(parser_nodelet utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} proj)

add_executable(parser_cli tests/parser_cli.cpp src/impl/novatel_parser.cpp)
target_link_libraries(parser_cli utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_executable(test_monitor tests/test_monitor.cpp)
target_link_libraries(test_monitor utils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

install(
     TARGETS stream_nodelet parser_nodelet utils test_monitor
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
