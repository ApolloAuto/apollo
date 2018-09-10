cmake_minimum_required(VERSION 2.8)
project(tools)

add_subdirectory(cyber_launch)
add_subdirectory(cyber_recorder)
add_subdirectory(cvt)
add_subdirectory(geometry2)
add_subdirectory(extrinsics_broadcaster)

install(FILES setup.bash DESTINATION .)
install(FILES cyber_tools_auto_complete.bash DESTINATION .)
install(FILES module_env.bash DESTINATION .)
