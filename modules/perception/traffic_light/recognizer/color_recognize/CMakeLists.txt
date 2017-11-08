project(traffic_light_recognizer_color_recognize)

AUX_SOURCE_DIRECTORY(./ THIS)
add_library(${PROJECT_NAME} ${THIS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

add_subdirectory(test)