cmake_minimum_required(VERSION 2.8.3)
project(python_examples)

# python install path
set(CYBER_PYTHON_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib/python/examples)

add_compile_options(-g)

# test case
add_executable(py_listener
  py_listener.cpp
)
target_link_libraries(py_listener cybertron)
install(TARGETS py_listener DESTINATION bin)

add_executable(py_talker
  py_talker.cpp
)
target_link_libraries(py_talker cybertron)
install(TARGETS py_talker DESTINATION bin)

add_executable(py_record
  py_record.cpp
)
target_link_libraries(py_record cybertron cybertron_record)
install(TARGETS py_record DESTINATION bin)

install(TARGETS
  py_listener
  py_talker
  py_record
  DESTINATION ${CYBER_PYTHON_INSTALL_DIR}
)