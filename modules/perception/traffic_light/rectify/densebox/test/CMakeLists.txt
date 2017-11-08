cmake_minimum_required(VERSION 2.8)
project(traffic_light_rectify_densebox_test)

AUX_SOURCE_DIRECTORY(. TEST)
add_executable(${PROJECT_NAME} ${TEST})
#set(CMAKE_CXX_FLAGS "-Wall")
target_link_libraries(${PROJECT_NAME} ${XLOG_LIBRARIES} traffic_light_rectify_densebox tl_base base
        ${OpenCV_LIBS} gtest pthread gtest_main ${Caffe_LINK} ${CAFFE_DEPENDS})


enable_testing()
add_test(NAME ${PROJECT_NAME}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ./${PROJECT_NAME})
smart_install(
        conf data img
        ${CMAKE_CURRENT_BINARY_DIR})
