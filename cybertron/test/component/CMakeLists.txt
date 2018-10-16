project(component_test)

add_executable(component_test component_test.cpp)
target_link_libraries(component_test cybertron gtest)

add_executable(timer_component_test timer_component_test.cpp)
target_link_libraries(timer_component_test cybertron gtest)

install(TARGETS
  component_test
  timer_component_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)
