project(record_test)

add_executable(record_file_test record_file_test.cpp)
target_link_libraries(record_file_test cybertron_common cybertron_record gtest)
install(TARGETS record_file_test DESTINATION ${UNIT_TEST_INSTALL_PREFIX})

add_executable(record_test record_test.cpp)
target_link_libraries(record_test cybertron_common cybertron_record gtest)
install(TARGETS record_test DESTINATION ${UNIT_TEST_INSTALL_PREFIX})
