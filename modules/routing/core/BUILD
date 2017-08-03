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
        "//modules/common/proto:common_proto",
        "//modules/map/proto:map_proto",
        "//modules/routing/proto:routing_proto",
        "//modules/routing/common",
        "//modules/routing/graph",
        "//modules/routing/strategy",
        "//modules/common",
        "//modules/common/time",
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
        "//modules/routing/common",
        "//modules/routing/graph"
    ],
)

cpplint()
