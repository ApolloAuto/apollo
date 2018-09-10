add_definitions(-DMODULE_NAME="watchdog")
add_library(watchdog_component watchdog_component.cpp watchdog.cpp)
target_link_libraries(watchdog_component cybertron)
install(TARGETS watchdog_component LIBRARY DESTINATION lib)