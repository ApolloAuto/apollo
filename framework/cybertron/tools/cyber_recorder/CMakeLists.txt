cmake_minimum_required(VERSION 2.8)
project(cyber_recorder)

add_library(cybertron_recorder_tool
  info.cpp
  recorder.cpp
  player.cpp
  spliter.cpp
  recoverer.cpp
)
target_link_libraries(cybertron_recorder_tool
  cybertron
  cybertron_proto
  cybertron_record
  ${GLOG_LIBRARIES}
)

install(TARGETS cybertron_recorder_tool LIBRARY DESTINATION lib)

add_executable(${PROJECT_NAME}
  main.cpp
)

target_link_libraries(${PROJECT_NAME}
  cybertron_recorder_tool
)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)






