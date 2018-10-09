project(data_test)

add_executable(cache_buffer_test cache_buffer_test.cpp)
target_link_libraries(cache_buffer_test cybertron gtest)

add_executable(data_visitor_test data_visitor_test.cpp)
target_link_libraries(data_visitor_test cybertron gtest)

add_executable(channel_buffer_test channel_buffer_test.cpp)
target_link_libraries(channel_buffer_test cybertron gtest)

add_executable(data_dispatcher_test data_dispatcher_test.cpp)
target_link_libraries(data_dispatcher_test cybertron gtest)

install(TARGETS
  cache_buffer_test
  data_visitor_test
  channel_buffer_test
  data_dispatcher_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)
