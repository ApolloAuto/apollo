project(static_transform_broadcaster)

AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR} DIR_SRCS)
add_executable(${PROJECT_NAME} ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME} cybertron) 

install(TARGETS ${PROJECT_NAME} 
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)
