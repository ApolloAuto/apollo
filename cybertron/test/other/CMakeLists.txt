project(other_test)

add_executable(init_cybertron_test init_cybertron_test.cpp)
target_link_libraries(init_cybertron_test cybertron gtest)

install(TARGETS
  init_cybertron_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)