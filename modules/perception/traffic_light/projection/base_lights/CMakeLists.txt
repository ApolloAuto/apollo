project(tl_lights_based_projection)

file(GLOB CPP_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_library(${PROJECT_NAME} ${CPP_FILES})

add_subdirectory(test)
