cmake_minimum_required(VERSION 2.8.3)
project(wrapper_test)

# python install path
set(CYBER_PYTHON_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib/python/wrapper/test)

add_compile_options(-g)

# test case
add_executable(test_init
  test_init.cpp
)
target_link_libraries(test_init cybertron gtest)
install(TARGETS test_init DESTINATION bin)

add_executable(test_node
  test_node.cpp
)
target_link_libraries(test_node cybertron gtest)
install(TARGETS test_node DESTINATION bin)

add_executable(test_record
  test_record.cpp
)
target_link_libraries(test_record cybertron cybertron_record gtest)
install(TARGETS test_record DESTINATION bin)

install(TARGETS
  test_init
  test_node
  test_record
  DESTINATION ${CYBER_PYTHON_INSTALL_DIR}
)