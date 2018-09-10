#Build examples
add_executable(start
  start.cpp
)
target_link_libraries(start -pthread dl cybertron PocoFoundation)

add_executable(stop_croutine
  stop_croutine.cpp
)
target_link_libraries(stop_croutine dl cybertron)

add_subdirectory(component_perception)
add_subdirectory(component_planning)
add_subdirectory(component_driver)
add_subdirectory(component_watchdog)
add_subdirectory(component_log)

add_executable(sensor_test
  sensor_test.cpp
)
target_link_libraries(sensor_test -pthread dl cybertron apollo_sensor)
install(TARGETS sensor_test DESTINATION bin)

add_executable(server_example
  service.cpp
)
target_link_libraries(server_example -pthread dl cybertron)

add_executable(parameter_example
  parameter.cpp
)
target_link_libraries(parameter_example -pthread dl cybertron)

add_executable(system_hook_sample
  system_hook_sample.cpp
)
target_link_libraries(system_hook_sample -pthread dl cybertron)

add_executable(talker_listener
  talker_listener.cpp
)
target_link_libraries(talker_listener -pthread dl cybertron)
install(TARGETS talker_listener DESTINATION bin)

add_executable(talker
  talker.cpp
)
target_link_libraries(talker cybertron)
install(TARGETS talker DESTINATION bin)

add_executable(timer
timer.cpp
)
target_link_libraries(timer cybertron)
install(TARGETS timer DESTINATION bin)

add_executable(listener
  listener.cpp
)
target_link_libraries(listener cybertron)
install(TARGETS listener DESTINATION bin)

add_executable(history
  history.cpp
)
target_link_libraries(history -pthread dl cybertron)
install(TARGETS history DESTINATION bin)

add_executable(tf_test
  tf_test.cpp
)
target_link_libraries(tf_test -pthread dl cybertron)
install(TARGETS tf_test DESTINATION bin)

add_executable(rwlock
  rw_lock.cpp
)
target_link_libraries(rwlock -Wl,--no-as-needed -pthread dl cybertron_common)

add_executable(rbtree_example
  rbtree_example.cpp
)
target_link_libraries(rbtree_example -Wl,--no-as-needed -pthread dl cybertron)

add_executable(user_defined_task
  user_defined_task.cpp
)
target_link_libraries(user_defined_task -Wl,--no-as-needed -pthread cybertron)


install(DIRECTORY dag DESTINATION ${CMAKE_INSTALL_PREFIX})

install(DIRECTORY launch DESTINATION ${CMAKE_INSTALL_PREFIX})

install(FILES daily_run.bash DESTINATION bin)
install(FILES daily_run_launcher.bash DESTINATION ${CMAKE_INSTALL_PREFIX})
