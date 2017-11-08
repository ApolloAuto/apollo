add_definitions(-Dprotected=public -Dprivate=public)
project(traffic_light_interface_test)

file(GLOB TEST_SRCS "*.cpp")
add_executable(${PROJECT_NAME} ${TEST_SRCS})
target_link_libraries(${PROJECT_NAME} gtest gtest_main traffic_light_interface)

enable_testing()
add_test(NAME ${PROJECT_NAME}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME})
install(DIRECTORY data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
