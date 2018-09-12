load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "routing_lib",
    srcs = [
        "routing.cc",
    ],
    hdrs = [
        "routing.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/map/hdmap:hdmap_util",
        "//modules/routing/common:routing_gflags",
        "//modules/routing/core",
        "//modules/routing/graph",
        "//modules/routing/proto:routing_proto",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "routing_component_lib",
    srcs = [
        "routing_component.cc",
    ],
    hdrs = [
        "routing_component.h",
    ],
    deps = [
        ":routing_lib",
    ],
    copts = ['-DMODULE_NAME=\\"routing\\"']
)

cc_binary(
    name = "librouting_component.so",
    deps = [":routing_component_lib"],
    linkopts = ["-shared"],
)

filegroup(
    name = "routing_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

cpplint()
