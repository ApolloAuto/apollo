cmake_minimum_required(VERSION 2.8)
project(cybertron_python)

add_subdirectory(wrapper)
add_subdirectory(examples)

install(DIRECTORY "cybertron"
        DESTINATION "lib/python/"
        FILES_MATCHING
        PATTERN "*.py")

install(DIRECTORY "test"
        DESTINATION "test/cybertron/python/")