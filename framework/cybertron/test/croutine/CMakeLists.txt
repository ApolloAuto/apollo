project(croutine_test)

add_executable(croutine_test 
  croutine_test.cpp
  routine_factory_test.cpp
)
target_link_libraries(croutine_test
  cybertron
  gtest
)

install(TARGETS croutine_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)
