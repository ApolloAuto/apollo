cmake_minimum_required(VERSION 2.8)

project(third_party)

file(GLOB THIRD_PARTY_FILES "*/lib/*.so*")
file(GLOB QT_FILES "Qt5.5.1/5.5/gcc_64/lib/*.so*")

install(FILES ${THIRD_PARTY_FILES} ${QT_FILES} DESTINATION third_party)
install(DIRECTORY "Qt5.5.1/5.5/gcc_64/plugins" DESTINATION third_party)
add_subdirectory(tf2)
