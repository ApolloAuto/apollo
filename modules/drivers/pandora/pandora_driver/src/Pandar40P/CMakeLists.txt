cmake_minimum_required(VERSION 2.8)
project(Pandar40P)


find_package( Boost REQUIRED )
find_package(PCL REQUIRED COMPONENTS common)

set (CMAKE_CXX_FLAGS "-fPIC --std=c++11")


include_directories(
	.
	include
	${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)


add_library( ${PROJECT_NAME}
	src/input.cc
	src/pandar40p_internal.cc
	src/pandar40p.cc
)

target_link_libraries(${PROJECT_NAME}
		${Boost_LIBRARIES}
		${PCL_IO_LIBRARIES}
)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
	

	add_executable(pandar40_test
		test/test.cc
	)

	target_link_libraries(pandar40_test
		${PROJECT_NAME}
		${Boost_LIBRARIES}
		${PCL_IO_LIBRARIES}
	)

endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

