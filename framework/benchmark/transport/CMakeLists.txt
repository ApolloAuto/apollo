cmake_minimum_required(VERSION 2.8.3)
project(benchmark_transport)

add_executable(intra intra.cpp)
target_link_libraries(intra cybertron)
install(TARGETS intra DESTINATION bin)

add_executable(inter_receiver inter_receiver.cpp)
target_link_libraries(inter_receiver cybertron)
install(TARGETS inter_receiver DESTINATION bin)

add_executable(inter_transmitter inter_transmitter.cpp)
target_link_libraries(inter_transmitter cybertron)
install(TARGETS inter_transmitter DESTINATION bin)

add_executable(likelihood_of_equal_id likelihood_of_equal_id.cpp)
target_link_libraries(likelihood_of_equal_id cybertron)

add_executable(perf_history perf_history.cpp)
target_link_libraries(perf_history cybertron)