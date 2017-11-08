project(traffic_light_onboard_test)
add_definitions(-Dprotected=public -Dprivate=public)

file(GLOB TEST_SRCS "*.cpp")
add_executable(${PROJECT_NAME} ${TEST_SRCS})
target_link_libraries(${PROJECT_NAME} ${GTEST_LIBS} -Wl,--whole-archive traffic_light_onboard -Wl,--no-whole-archive ${Caffe_LINK} ${PERCEPTION_LIBS} ${MAP_LIBS})

enable_testing()
add_test(NAME ${PROJECT_NAME}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME})
install(DIRECTORY data conf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
