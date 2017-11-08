project(traffic_light_rectify_unity)

AUX_SOURCE_DIRECTORY(./ src)
AUX_SOURCE_DIRECTORY(./crop crop)
add_library(${PROJECT_NAME} ${src} ${crop})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${MAP_LIBS} tl_base config_manager adu_common onboard ${ROS_LIBS})

add_subdirectory(test)