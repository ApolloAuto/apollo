package(default_visibility = ["//visibility:public"])

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_library

cc_binary(
    name = "libplanning_component.so",
    deps = [":planning_component_lib"],
    linkopts = ["-shared"],
)

cc_library(
    name = "planning_component_lib",
    srcs = ["planning_component.cc"],
    hdrs = ["planning_component.h"],
    deps = ["//framework:cybertron"],
    copts = ['-DMODULE_NAME=\\"planning\\"']
)