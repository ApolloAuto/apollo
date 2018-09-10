cmake_minimum_required(VERSION 2.8.3)

add_library(sender_component sender_component.cpp)
target_link_libraries(sender_component cybertron)
install(TARGETS sender_component LIBRARY DESTINATION lib)

add_library(receiver_component receiver_component.cpp)
target_link_libraries(receiver_component cybertron)
install(TARGETS receiver_component LIBRARY DESTINATION lib)
