cmake_minimum_required(VERSION 2.8.3)
project(lslidar_c16_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  LslidarC16Layer.msg
  LslidarC16Packet.msg
  LslidarC16Point.msg
  LslidarC16Scan.msg
  LslidarC16Sweep.msg
  LslidarC16ScanUnified.msg
)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)
