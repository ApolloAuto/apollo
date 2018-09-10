#Build examples

add_definitions(-DMODULE_NAME="logtest")
add_library(log_component log_component.cpp)
target_link_libraries(log_component cybertron)
install(TARGETS log_component LIBRARY DESTINATION lib)
