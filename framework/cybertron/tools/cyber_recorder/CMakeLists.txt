cmake_minimum_required(VERSION 2.8)
project(cyber_recorder)

add_executable(${PROJECT_NAME}
  main.cpp
  info.cpp
  recorder.cpp
  player.cpp
  spliter.cpp
  recoverer.cpp
)

target_link_libraries(${PROJECT_NAME}
  cybertron_common
  cybertron
  cybertron_proto
  cybertron_record
  ${GLOG_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
