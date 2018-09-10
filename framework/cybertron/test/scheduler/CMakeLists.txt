project(scheduler_test)

add_executable(proc_balancer_test proc_balancer_test.cpp)
target_link_libraries(proc_balancer_test
  cybertron
  gtest
)
install(TARGETS proc_balancer_test DESTINATION ${UNIT_TEST_INSTALL_PREFIX})

add_executable(scheduler_test scheduler_test.cpp)
target_link_libraries(scheduler_test
  cybertron
  gtest
)
install(TARGETS scheduler_test DESTINATION ${UNIT_TEST_INSTALL_PREFIX})

add_executable(routine_balancer_test routine_balancer_test.cpp)
target_link_libraries(routine_balancer_test
  cybertron
  gtest
)
install(TARGETS routine_balancer_test DESTINATION ${UNIT_TEST_INSTALL_PREFIX})

add_executable(processor_test processor_test.cpp)
target_link_libraries(processor_test
  cybertron
  gtest
)
install(TARGETS processor_test DESTINATION ${UNIT_TEST_INSTALL_PREFIX})

add_executable(scheduler_policy_test scheduler_policy_test.cpp)
target_link_libraries(scheduler_policy_test
  cybertron
  gtest
)
install(TARGETS scheduler_policy_test DESTINATION ${UNIT_TEST_INSTALL_PREFIX})

add_executable(task_test task_test.cpp)
target_link_libraries(task_test
  cybertron
  gtest
)
install(TARGETS task_test DESTINATION ${UNIT_TEST_INSTALL_PREFIX})