project(dispatcher_test)

add_executable(dispatcher_dispatcher_test
  dispatcher_test.cpp
)

target_link_libraries(dispatcher_dispatcher_test
  cybertron
  cybertron_dispatcher
  gtest
)

add_executable(dispatcher_message_test
  message_test.cpp
)

target_link_libraries(dispatcher_message_test
  cybertron
  cybertron_dispatcher
  gtest
)

install(TARGETS
  dispatcher_dispatcher_test
  dispatcher_message_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)