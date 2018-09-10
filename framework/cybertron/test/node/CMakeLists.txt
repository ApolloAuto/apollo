project(node_test)

add_executable(node_test node_test.cpp)
target_link_libraries(node_test cybertron gtest)

add_executable(writer_reader_test writer_reader_test.cpp)
target_link_libraries(writer_reader_test cybertron gtest)

install(TARGETS
  node_test
  writer_reader_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)