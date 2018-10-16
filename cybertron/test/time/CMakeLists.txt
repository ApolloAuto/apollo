project(time_test)

add_executable(time_test time_test.cpp)
target_link_libraries(time_test
  cybertron
  gtest
)

add_executable(duration_test duration_test.cpp)
target_link_libraries(duration_test
  cybertron
  gtest
)

install(TARGETS time_test duration_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)
