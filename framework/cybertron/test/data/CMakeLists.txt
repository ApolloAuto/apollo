project(data_test)

add_executable(data_test any_test.cpp)
target_link_libraries(data_test cybertron gtest)

add_executable(cache_buffer_test cache_buffer_test.cpp)
target_link_libraries(cache_buffer_test cybertron gtest)

add_executable(data_cache_test data_cache_test.cpp)
target_link_libraries(data_cache_test cybertron gtest)

add_executable(data_visitor_test data_visitor_test.cpp)
target_link_libraries(data_visitor_test cybertron gtest)

add_executable(data_fusion_test data_fusion_test.cpp)
target_link_libraries(data_fusion_test cybertron gtest)

add_executable(data_fusion_notifier_test data_fusion_notifier_test.cpp)
target_link_libraries(data_fusion_notifier_test cybertron gtest)

install(TARGETS 
  data_test
  cache_buffer_test
  data_cache_test
  data_visitor_test
  data_fusion_test
  data_fusion_notifier_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)
