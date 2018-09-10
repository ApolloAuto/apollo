#Build examples
add_definitions(-DMODULE_NAME="driver")
add_library(driver_component driver_component.cpp)
target_link_libraries(driver_component cybertron)
install(TARGETS driver_component LIBRARY DESTINATION lib)