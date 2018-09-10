cmake_minimum_required(VERSION 2.8.3)
project(benchmark_topology)

add_executable(throughput throughput.cpp)
target_link_libraries(throughput cybertron)

add_executable(topo_master master.cpp)
target_link_libraries(topo_master cybertron)
install(TARGETS topo_master DESTINATION bin)

add_executable(topo_slave slave.cpp)
target_link_libraries(topo_slave cybertron)
install(TARGETS topo_slave DESTINATION bin)

add_executable(perf_graph perf_graph.cpp)
target_link_libraries(perf_graph cybertron)

install(FILES topo_launcher.bash DESTINATION bin)
