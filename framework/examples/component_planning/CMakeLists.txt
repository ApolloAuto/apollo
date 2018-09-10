#Build examples

add_definitions(-DMODULE_NAME="planning")
add_library(planning_component planning_component.cpp)
target_link_libraries(planning_component cybertron)
install(TARGETS planning_component LIBRARY DESTINATION lib)