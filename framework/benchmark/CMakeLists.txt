#Build examples

add_executable(benchmark_latency_gaussian
  benchmark_gaussian_elimination.cpp
)
target_link_libraries(benchmark_latency_gaussian -pthread dl cybertron)

add_executable(benchmark_configurable_dag
  benchmark_configurable_dag.cpp
)
target_link_libraries(benchmark_configurable_dag -pthread dl cybertron)

add_executable(benchmark_throughout
  benchmark_throughout.cpp
)
target_link_libraries(benchmark_throughout -pthread dl cybertron)

add_executable(benchmark_switch
  benchmark_switch.cpp
)
target_link_libraries(benchmark_switch -pthread dl cybertron)

add_executable(perf_objectpool
  perf_object_pool.cpp
)
target_link_libraries(perf_objectpool -pthread)

add_executable(perf_concurrent_objectpool
  perf_concurrent_object_pool.cpp
)
target_link_libraries(perf_concurrent_objectpool -latomic -pthread)

add_executable(perf_atomic_fifo
  perf_atomic_fifo.cpp
)
target_link_libraries(perf_atomic_fifo -Wl,--no-as-needed -latomic -pthread)

add_executable(init_and_shutdown init_and_shutdown.cpp)
target_link_libraries(init_and_shutdown cybertron)

add_executable(atomic_rw_lock atomic_rw_lock.cpp)
target_link_libraries(atomic_rw_lock cybertron)

add_executable(bounded_queue bounded_queue.cpp)
target_link_libraries(bounded_queue cybertron)

add_executable(atomic_hash_map atomic_hash_map.cpp)
target_link_libraries(atomic_hash_map cybertron)

add_executable(tf2_cybertron tf2_cybertron.cpp)
target_link_libraries(tf2_cybertron cybertron)

add_subdirectory(service_discovery)
add_subdirectory(transport)
add_subdirectory(component)
add_subdirectory(component_for_qa)

install(DIRECTORY dag DESTINATION ${CMAKE_INSTALL_PREFIX})
install(DIRECTORY launch DESTINATION ${CMAKE_INSTALL_PREFIX})
