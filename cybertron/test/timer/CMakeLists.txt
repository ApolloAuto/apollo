project(timer_test)

add_executable(timer_manager_test
timer_manager_test.cpp
)

target_link_libraries(timer_manager_test
  cybertron
  gtest
)

add_executable(timing_wheel_test
  timing_wheel_test.cpp
)

target_link_libraries(timing_wheel_test
  cybertron
  gtest
)

install(TARGETS
timer_manager_test
  timing_wheel_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)