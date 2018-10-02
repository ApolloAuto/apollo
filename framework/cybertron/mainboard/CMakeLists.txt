project(mainboard)

add_executable(mainboard
  mainboard.cpp
  module_argument.cpp
  module_controller.cpp
)

target_link_libraries(mainboard -pthread cybertron)

install(TARGETS mainboard DESTINATION bin)
