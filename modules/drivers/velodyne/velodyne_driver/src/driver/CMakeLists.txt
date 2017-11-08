# build the driver node
add_executable(driver_node 
  driver_node.cpp 
  driver.cpp 
  driver16.cpp
  # driver32.cpp
  driver64.cpp
  )
target_link_libraries(driver_node
  velodyne_input
  ${catkin_LIBRARIES}
)

# build the nodelet version
add_library(driver_nodelet 
  driver_nodelet.cpp
  driver.cpp 
  driver16.cpp
  # driver32.cpp
  driver64.cpp
)
target_link_libraries(driver_nodelet
  velodyne_input
  ${catkin_LIBRARIES}
)

# install runtime files
install(TARGETS driver_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
