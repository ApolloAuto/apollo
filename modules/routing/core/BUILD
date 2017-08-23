load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "core",
    deps = [
        ":routing_navigator",
        ":routing_node_range_manager",
    ],
)

cc_library(
    name = "routing_navigator",
    srcs = [
        "navigator.cc",
    ],
    hdrs = [
        "navigator.h",
    ],
    deps = [
        ":routing_node_range_manager",
        "//modules/common",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/proto:common_proto",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
        "//modules/routing/common:routing_gflags",
        "//modules/routing/graph",
        "//modules/routing/proto:routing_proto",
        "//modules/routing/strategy",
    ],
)

cc_library(
    name = "routing_node_range_manager",
    srcs = [
        "node_range_manager.cc",
    ],
    hdrs = [
        "node_range_manager.h",
    ],
    deps = [
        "//modules/common/proto:common_proto",
        "//modules/routing/common:routing_gflags",
        "//modules/routing/graph",
    ],
)

cpplint()
