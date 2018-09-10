project(mainboard_test)

add_executable(mainboard_test mainboard_test.cpp)
target_link_libraries(mainboard_test cybertron gtest)

install(TARGETS
  mainboard_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)
