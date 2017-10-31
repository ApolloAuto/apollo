#add_definitions(-DTIME_CONSISTENCY_CHECK)
add_library(velodyne_parser 
    velodyne_parser.cpp  
    velodyne64_parser.cpp 
    # velodyne32_parser.cpp  
    velodyne16_parser.cpp
    calibration.cpp
    online_calibration.cpp
    util.cpp
)
target_link_libraries(velodyne_parser
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES})

install(TARGETS velodyne_parser
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
