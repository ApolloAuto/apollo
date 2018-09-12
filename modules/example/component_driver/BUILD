package(default_visibility = ["//visibility:public"])

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_library

cc_binary(
    name = "libdriver_component.so",
    deps = [":driver_component_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "driver_component_lib",
    srcs = ["driver_component.cc"],
    hdrs = ["driver_component.h"],
    deps = [
        "//modules/example/component_driver/proto:driver_proto",
        "//framework:cybertron",
    ],
    copts = ['-DMODULE_NAME=\\"driver\\"']
)