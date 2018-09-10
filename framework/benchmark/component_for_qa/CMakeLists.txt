cmake_minimum_required(VERSION 2.8.3)

add_library(component_for_qa SHARED
    component.cpp
    component_test.cpp
    data_pool.cpp
    whisper_common.cpp
)
target_link_libraries(component_for_qa cybertron)
install(TARGETS component_for_qa LIBRARY DESTINATION lib)
install(DIRECTORY conf/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/conf/
)
install(DIRECTORY dag/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/dag/
)
install(DIRECTORY launch/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/launch/
)
