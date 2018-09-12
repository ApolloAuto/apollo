cmake_minimum_required(VERSION 2.8.3)
project(benchmark_service_discovery)

add_executable(throughput throughput.cpp)
target_link_libraries(throughput cybertron)

add_executable(service_discovery_master master.cpp)
target_link_libraries(service_discovery_master cybertron)
install(TARGETS service_discovery_master DESTINATION bin)

add_executable(service_discovery_slave slave.cpp)
target_link_libraries(service_discovery_slave cybertron)
install(TARGETS service_discovery_slave DESTINATION bin)

add_executable(perf_graph perf_graph.cpp)
target_link_libraries(perf_graph cybertron)

install(FILES service_discovery_launcher.bash DESTINATION bin)
