project(transport_test)

add_executable(transport_common_test common_test.cpp)
target_link_libraries(transport_common_test cybertron gtest)

add_executable(transport_message_test message_test.cpp)
target_link_libraries(transport_message_test cybertron gtest)

add_executable(transport_shm_test shm_test.cpp)
target_link_libraries(transport_shm_test cybertron gtest)

add_executable(transport_rtps_test rtps_test.cpp)
target_link_libraries(transport_rtps_test cybertron gtest)

add_executable(dispatcher_test dispatcher_test.cpp)
target_link_libraries(dispatcher_test cybertron gtest)

add_executable(intra_dispatcher_test intra_dispatcher_test.cpp)
target_link_libraries(intra_dispatcher_test cybertron gtest)

add_executable(shm_dispatcher_test shm_dispatcher_test.cpp)
target_link_libraries(shm_dispatcher_test cybertron gtest)

add_executable(rtps_dispatcher_test rtps_dispatcher_test.cpp)
target_link_libraries(rtps_dispatcher_test cybertron gtest)

add_executable(intra_transceiver_test intra_transceiver_test.cpp)
target_link_libraries(intra_transceiver_test cybertron gtest)

add_executable(shm_transceiver_test shm_transceiver_test.cpp)
target_link_libraries(shm_transceiver_test cybertron gtest)

add_executable(rtps_transceiver_test rtps_transceiver_test.cpp)
target_link_libraries(rtps_transceiver_test cybertron gtest)

add_executable(hybrid_transceiver_test hybrid_transceiver_test.cpp)
target_link_libraries(hybrid_transceiver_test cybertron gtest)

add_executable(transport_test transport_test.cpp)
target_link_libraries(transport_test cybertron gtest)

add_executable(reactor_test reactor_test.cpp)
target_link_libraries(reactor_test cybertron gtest)

install(TARGETS
  transport_common_test
  transport_message_test
  transport_shm_test
  transport_rtps_test
  dispatcher_test
  intra_dispatcher_test
  shm_dispatcher_test
  rtps_dispatcher_test
  intra_transceiver_test
  shm_transceiver_test
  rtps_transceiver_test
  hybrid_transceiver_test
  transport_test
  reactor_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)
