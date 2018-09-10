project(topology_test)

add_executable(node_manager_test
  node_manager_test.cpp
)

target_link_libraries(node_manager_test
  cybertron
  gtest
)

add_executable(service_manager_test
  service_manager_test.cpp
)

target_link_libraries(service_manager_test
  cybertron
  gtest
)

add_executable(channel_manager_test
  channel_manager_test.cpp
)

target_link_libraries(channel_manager_test
  cybertron
  gtest
)

add_executable(topology_test
  topology_test.cpp
)

target_link_libraries(topology_test
  cybertron
  gtest
)

add_executable(graph_test
  graph_test.cpp
)

target_link_libraries(graph_test
  cybertron
  gtest
)

add_executable(role_test
  role_test.cpp
)

target_link_libraries(role_test
  cybertron
  gtest
)

add_executable(warehouse_test
  warehouse_test.cpp
)

target_link_libraries(warehouse_test
  cybertron
  gtest
)

install(TARGETS
  node_manager_test
  service_manager_test
  channel_manager_test
  topology_test
  graph_test
  role_test
  warehouse_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)