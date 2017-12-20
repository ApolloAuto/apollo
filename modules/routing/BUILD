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
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
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

cc_binary(
    name = "routing",
    srcs = ["main.cc"],
    deps = [
        ":routing_lib",
        "//external:gflags",
    ],
)

filegroup(
    name = "routing_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

cpplint()
