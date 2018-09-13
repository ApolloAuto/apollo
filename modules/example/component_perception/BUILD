load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_library

cc_binary(
    name = "libperception_component.so",
    deps = [":perception_component_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "perception_component_lib",
    srcs = ["perception_component.cc"],
    hdrs = ["perception_component.h"],
    deps = [
        "//framework:cybertron",
        "//modules/example/component_driver/proto:driver_proto"
        ],
    copts = ['-DMODULE_NAME=\\"perception\\"']
)

cpplint()
