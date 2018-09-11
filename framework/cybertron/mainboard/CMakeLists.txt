project(mainboard)

add_executable(mainboard
  mainboard.cpp
)

target_link_libraries(mainboard -pthread cybertron)

install(TARGETS mainboard DESTINATION bin)
