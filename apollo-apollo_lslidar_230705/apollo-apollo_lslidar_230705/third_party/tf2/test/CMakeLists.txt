ADD_DEFINITIONS(-fno-access-control -DUNIT_TEST=1)

add_executable(cache_unittest cache_unittest.cpp)
target_link_libraries(cache_unittest
  gtest_main
  tf2
)

# install(TARGETS cache_unittest
#   DESTINATION ${UNIT_TEST_INSTALL_PREFIX})
#
add_executable(simple_tf2_core simple_tf2_core.cpp)
target_link_libraries(simple_tf2_core
  gtest_main
  tf2
)

# install(TARGETS simple_tf2_core
#   DESTINATION ${UNIT_TEST_INSTALL_PREFIX})

add_executable(tf2_speed_test speed_test.cpp)
target_link_libraries(tf2_speed_test
  gtest_main
  tf2
)
#
# install(TARGETS tf2_speed_test
#   DESTINATION ${UNIT_TEST_INSTALL_PREFIX})
#

add_executable(tf2_static_cache_test static_cache_test.cpp)
target_link_libraries(tf2_static_cache_test
  gtest_main
  tf2
)

# install(TARGETS tf2_static_cache_test
#   DESTINATION ${UNIT_TEST_INSTALL_PREFIX})
#
add_executable(tf2_speed_2_test speed_2_test.cpp)
target_link_libraries(tf2_speed_2_test
  gtest_main
  tf2
)

# install(TARGETS tf2_speed_2_test
#   DESTINATION ${UNIT_TEST_INSTALL_PREFIX})
