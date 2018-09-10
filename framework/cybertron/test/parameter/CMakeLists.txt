project(parameter_test)

add_executable(parameter_test parameter_test.cpp)
target_link_libraries(parameter_test
  cybertron
  gtest
)

add_executable(parameter_server_test parameter_server_test.cpp)
target_link_libraries(parameter_server_test
  cybertron
  gtest
)

add_executable(parameter_client_test parameter_client_test.cpp)
target_link_libraries(parameter_client_test
  cybertron
  gtest
)

install(TARGETS parameter_test parameter_server_test parameter_client_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)
