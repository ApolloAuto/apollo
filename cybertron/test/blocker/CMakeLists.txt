project(blocker_test)

add_executable(blocker_test
  blocker_test.cpp
)

target_link_libraries(blocker_test
  cybertron
  cybertron_blocker
  gtest
)

add_executable(blocker_manager_test
  blocker_manager_test.cpp
)

target_link_libraries(blocker_manager_test
  cybertron
  cybertron_blocker
  gtest
)

install(TARGETS
  blocker_test
  blocker_manager_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)