cmake_minimum_required(VERSION 2.8.3)
project(python_wrapper)

# python install path
set(CYBER_PYTHON_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/python/cybertron)

add_compile_options(-g)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
link_directories(${PYTHON_LIBRARY_DIRS})

# python wrapper libs
set(CYBER_NODE "_cyber_node")
set(CYBER_RECORD "_cyber_record")
set(CYBER_INIT "_cyber_init")

add_library(${CYBER_NODE} SHARED cyber_node_wrap.cpp)
target_link_libraries(${CYBER_NODE} cybertron
${PYTHON_LIBRARIES}) 

add_library(${CYBER_RECORD} SHARED cyber_record_wrap.cpp)
target_link_libraries(${CYBER_RECORD} cybertron cybertron_record
${PYTHON_LIBRARIES}) 

add_library(${CYBER_INIT} SHARED cyber_init_wrap.cpp)
target_link_libraries(${CYBER_INIT} cybertron
${PYTHON_LIBRARIES}) 

# remove lib prefix
set_target_properties(${CYBER_NODE} PROPERTIES PREFIX "")
set_target_properties(${CYBER_RECORD} PROPERTIES PREFIX "")
set_target_properties(${CYBER_INIT} PROPERTIES PREFIX "")

install(TARGETS ${CYBER_NODE}
    DESTINATION ${CYBER_PYTHON_INSTALL_DIR}
)
install(TARGETS ${CYBER_RECORD}
    DESTINATION ${CYBER_PYTHON_INSTALL_DIR}
)
install(TARGETS ${CYBER_INIT}
    DESTINATION ${CYBER_PYTHON_INSTALL_DIR}
)
