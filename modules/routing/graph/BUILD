load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "graph",
    deps = [
        ":routing_topo_graph",
        ":routing_topo_node",
    ],
)

cc_library(
    name = "routing_topo_node",
    srcs = [
        "topo_node.cc",
    ],
    hdrs = [
        "topo_node.h",
    ],
    deps = [
        "//modules/common/proto:common_proto",
        "//modules/map/proto:map_proto",
        "//modules/routing/proto:routing_proto",
    ],
)

cc_library(
    name = "routing_topo_graph",
    srcs = [
        "topo_graph.cc",
    ],
    hdrs = [
        "topo_graph.h",
    ],
    deps = [
        ":routing_topo_node",
        "//modules/common",
        "//modules/common/proto:common_proto",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
        "//modules/routing/common",
        "//modules/routing/proto:routing_proto",
    ],
)

cc_library(
    name = "routing_graph_test_utils",
    srcs = [
        "test_utils.cc",
    ],
    hdrs = [
        "test_utils.h",
    ],
    deps = [
        "@gtest//:main",
        ":routing_topo_node",
        ":routing_topo_graph",
        "//modules/map/proto:map_proto",
    ],
)

cc_test(
    name = "topo_node_test",
    size = "small",
    srcs = [
        "topo_node_test.cc",
    ],
    deps = [
        ":routing_graph_test_utils",
        "@gtest//:main",
    ],
)

cpplint()
