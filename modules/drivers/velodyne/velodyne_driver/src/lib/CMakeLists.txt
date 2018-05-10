add_library(
    velodyne_input 
    input.cpp
    socket_input.cpp
    # pcap_input.cpp
)
target_link_libraries(velodyne_input
    ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
    add_dependencies(velodyne_input ${catkin_EXPORTED_TARGETS})
endif()

if(ENABLE_GPS_DISCARD)
    add_definitions(-DDISCARD_GPS_CHECK_FOR_UNITTEST)
endif(ENABLE_GPS_DISCARD)

install(TARGETS velodyne_input
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
