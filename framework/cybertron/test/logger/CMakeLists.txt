project(logger_test)

add_executable(logger_test logger_test.cpp)
target_link_libraries(logger_test cybertron gtest)
install(TARGETS logger_test DESTINATION ${UNIT_TEST_INSTALL_PREFIX})
