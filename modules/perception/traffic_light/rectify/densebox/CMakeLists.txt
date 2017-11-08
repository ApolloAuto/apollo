project(traffic_light_rectify_densebox)

AUX_SOURCE_DIRECTORY(../unity/crop Crop)
AUX_SOURCE_DIRECTORY(./detection Detection)
AUX_SOURCE_DIRECTORY(./selector Selector)
AUX_SOURCE_DIRECTORY(./ DB)
add_library(${PROJECT_NAME} ${DB} ${Crop} ${Detection} ${Selector})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${MAP_LIBS} tl_base config_manager adu_common onboard ${ROS_LIBS})

add_subdirectory(test)
