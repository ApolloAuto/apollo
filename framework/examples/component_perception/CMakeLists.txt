#Build examples
# cuda_add_library(perception_component
#   component/perception_component.cpp
#   component/perception.cu
# )
add_definitions(-DMODULE_NAME="perception")
add_library(perception_component perception_component.cpp)
target_link_libraries(perception_component cybertron)
install(TARGETS perception_component LIBRARY DESTINATION lib)