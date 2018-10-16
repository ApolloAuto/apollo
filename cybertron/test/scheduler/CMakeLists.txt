project(scheduler_test)

add_executable(scheduler_test scheduler_test.cpp)
target_link_libraries(scheduler_test
  cybertron
  gtest
)
install(TARGETS scheduler_test DESTINATION ${UNIT_TEST_INSTALL_PREFIX})

#add_executable(scheduler_policy_test scheduler_policy_test.cpp)
#target_link_libraries(scheduler_policy_test
#  cybertron
#  gtest
#)
#install(TARGETS scheduler_policy_test DESTINATION ${UNIT_TEST_INSTALL_PREFIX})
