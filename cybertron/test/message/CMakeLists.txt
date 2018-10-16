project(message)

add_executable(message_traits_test message_traits_test.cpp)
target_link_libraries(message_traits_test cybertron gtest)

add_executable(raw_message_test raw_message_test.cpp)
target_link_libraries(raw_message_test cybertron gtest)

install(TARGETS
  message_traits_test
  raw_message_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)
