cmake_minimum_required(VERSION 2.8.3)
project(python_test)

# python install path
set(CYBER_PYTHON_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/python/cybertron/test)

add_compile_options(-g)

# test case
add_executable(test_pynode_listener
  test_pynode_listener.cpp
)
target_link_libraries(test_pynode_listener cybertron)
install(TARGETS test_pynode_listener DESTINATION bin)

add_executable(test_pynode_talker
  test_pynode_talker.cpp
)
target_link_libraries(test_pynode_talker cybertron)
install(TARGETS test_pynode_talker DESTINATION bin)

add_executable(test_pyrecord
  test_pyrecord.cpp
)
target_link_libraries(test_pyrecord cybertron cybertron_record)
install(TARGETS test_pyrecord DESTINATION bin)



