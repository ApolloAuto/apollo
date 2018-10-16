cmake_minimum_required(VERSION 2.8.3)

include_directories(../../../thirdparty/comm/include)

add_library(TestPlugins1 SHARED  plugins1.cpp)
target_link_libraries(TestPlugins1 cybertron)

add_library(TestPlugins2 SHARED  plugins2.cpp)
target_link_libraries(TestPlugins2 cybertron)

add_executable(class_loader_test class_loader_test.cpp)
target_link_libraries(class_loader_test cybertron gtest PocoFoundation)

install(TARGETS TestPlugins1
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)

install(TARGETS TestPlugins2
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)

install(TARGETS class_loader_test
  DESTINATION ${UNIT_TEST_INSTALL_PREFIX}
)

