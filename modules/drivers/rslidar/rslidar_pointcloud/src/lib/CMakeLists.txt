#add_definitions(-DTIME_CONSISTENCY_CHECK)
add_library(rslidarParser 
    rslidarParser.cpp  
    rslidar16Parser.cpp
    rslidar32Parser.cpp 
    util.cpp
    calibration.cpp
)


target_link_libraries(rslidarParser
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES})

install(TARGETS rslidarParser
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
