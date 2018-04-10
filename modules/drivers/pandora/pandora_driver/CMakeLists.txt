cmake_minimum_required(VERSION 2.8.3)
project(pandora_driver)
find_package(catkin REQUIRED)
catkin_package(
    CATKIN_DEPENDS)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_ros
  roscpp
  roslib
  sensor_msgs
)

find_package( Boost REQUIRED )
find_package( PCL REQUIRED COMPONENTS common )
find_package( OpenCV REQUIRED )

catkin_package()
set (CMAKE_CXX_FLAGS "-fPIC --std=c++11")

add_subdirectory(src/Pandar40P)

include_directories(
	.
	include
	src/Pandar40P/include
	${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME} SHARED
	src/pandora.cc
	src/pandora_client.c
	src/pandora_camera.cc
	src/util.c
	src/tcp_command_client.c
)

target_link_libraries(${PROJECT_NAME} 
		Pandar40P
        ${catkin_LIBRARIES}
		${Boost_LIBRARIES}
		${PCL_IO_LIBRARIES}
		${OpenCV_LIBS}
		yaml-cpp
		jpeg
)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})

	add_executable(pandora_node
		src/main.cc
	)

	target_link_libraries(pandora_node
	  Pandar40P
		${PROJECT_NAME}
		${Boost_LIBRARIES}
		${PCL_IO_LIBRARIES}
		${OpenCV_LIBS}
		yaml-cpp
		jpeg
	)

	add_executable(pandora_test
		test/test.cc
	)

	target_link_libraries(pandora_test
	  Pandar40P
		${PROJECT_NAME}
		${Boost_LIBRARIES}
		${PCL_IO_LIBRARIES}
		${OpenCV_LIBS}
		yaml-cpp
		jpeg
	)

endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS pandora_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS pandora_test
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

