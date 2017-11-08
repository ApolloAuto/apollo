cmake_minimum_required(VERSION 2.8)
project(traffic_light_color_reviser_test)

AUX_SOURCE_DIRECTORY(. TEST)
add_executable(${PROJECT_NAME} ${TEST})

target_link_libraries(${PROJECT_NAME} traffic_light_reviser_strategy tl_base config_manager adu_common onboard
        ${OpenCV_LIBS} gtest pthread gtest_main ${Caffe_LINK} ${CAFFE_DEPENDS})

enable_testing()
add_test(NAME ${PROJECT_NAME}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ./${PROJECT_NAME})
install(
        DIRECTORY conf
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
