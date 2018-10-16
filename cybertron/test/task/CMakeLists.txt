project(scheduler_test)

add_executable(task_test task_test.cpp)
target_link_libraries(task_test
  cybertron
  gtest
)
install(TARGETS task_test DESTINATION ${UNIT_TEST_INSTALL_PREFIX})
