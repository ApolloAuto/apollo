project(base_test)

add_executable(atomic_rw_lock_test atomic_rw_lock_test.cpp)
target_link_libraries(atomic_rw_lock_test gtest)

add_executable(bounded_queue_test bounded_queue_test.cpp)
target_link_libraries(bounded_queue_test gtest)

add_executable(signal_test signal_test.cpp)
target_link_libraries(signal_test cybertron gtest)

add_executable(object_pool_test object_pool_test.cpp)
target_link_libraries(object_pool_test  gtest)

add_executable(atomic_hash_map_test atomic_hash_map_test.cpp)
target_link_libraries(atomic_hash_map_test  gtest)

install(TARGETS 
  atomic_rw_lock_test
  bounded_queue_test
  signal_test
  object_pool_test
  atomic_hash_map_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)
