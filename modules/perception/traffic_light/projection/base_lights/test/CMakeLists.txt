project(tl_lights_based_projection_test)
add_definitions(-Dprotected=public -Dprivate=public)

file(GLOB TEST_CPP_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_executable(${PROJECT_NAME} ${TEST_CPP_FILES})
target_link_libraries(${PROJECT_NAME} tl_lights_based_projection traffic_light_interface tl_projection tl_base ${MAP_LIBS} ${PERCEPTION_LIBS} ${GTEST_LIBS})

enable_testing()
add_test(NAME ${PROJECT_NAME}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME})
install(DIRECTORY data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
install(DIRECTORY conf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

