project(common_test)

add_executable(environment_test environment_test.cpp)
target_link_libraries(environment_test cybertron gtest)

add_executable(common_file_test file_test.cpp)
target_link_libraries(common_file_test cybertron gtest)

add_executable(logfileobject_test logfileobject_test.cpp)
target_link_libraries(logfileobject_test cybertron gtest)

add_executable(log_test log_test.cpp)
target_link_libraries(log_test cybertron gtest)

install(TARGETS
  environment_test
  common_file_test
  logfileobject_test
  log_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)
