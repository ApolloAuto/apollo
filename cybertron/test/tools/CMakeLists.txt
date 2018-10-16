project(tools)

add_executable(recorder_recorder_test recorder_test.cpp)
target_link_libraries(recorder_recorder_test cybertron gtest)

install(TARGETS
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)
