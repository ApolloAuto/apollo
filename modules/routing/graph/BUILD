load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "graph",
    deps = [
        ":routing_node_with_range",
        ":routing_sub_topo_graph",
        ":routing_topo_graph",
        ":routing_topo_node",
        ":routing_topo_range_manager",
    ],
)

cc_library(
    name = "routing_topo_range",
    srcs = [
        "topo_range.cc",
    ],
    hdrs = [
        "topo_range.h",
    ],
    deps = [
        "//modules/routing/common:routing_gflags",
    ],
)

cc_library(
    name = "routing_topo_range_manager",
    srcs = [
        "topo_range_manager.cc",
    ],
    hdrs = [
        "topo_range_manager.h",
    ],
    deps = [
        ":routing_topo_node",
        ":routing_topo_range",
        "//modules/common/util:map_util",
    ],
)

cc_library(
    name = "routing_range_utils",
    hdrs = [
        "range_utils.h",
    ],
    deps = [
    ],
)

cc_library(
    name = "routing_node_with_range",
    srcs = [
        "node_with_range.cc",
    ],
    hdrs = [
        "node_with_range.h",
    ],
    deps = [
        ":routing_topo_node",
        ":routing_topo_range",
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
        ":routing_range_utils",
        ":routing_topo_range",
        "//modules/common:log",
        "//modules/common/proto:common_proto",
        "//modules/common/util:map_util",
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
        "//modules/routing/common:routing_gflags",
        "//modules/routing/proto:routing_proto",
    ],
)

cc_library(
    name = "routing_sub_topo_graph",
    srcs = [
        "sub_topo_graph.cc",
    ],
    hdrs = [
        "sub_topo_graph.h",
    ],
    deps = [
        ":routing_node_with_range",
        ":routing_topo_node",
        "//modules/common",
        "//modules/common/proto:common_proto",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
        "//modules/routing/common:routing_gflags",
        "//modules/routing/proto:routing_proto",
    ],
)

cc_library(
    name = "routing_topo_test_utils",
    srcs = [
        "topo_test_utils.cc",
    ],
    hdrs = [
        "topo_test_utils.h",
    ],
    deps = [
        ":routing_topo_graph",
        ":routing_topo_node",
    ],
)

cc_test(
    name = "topo_node_test",
    size = "small",
    srcs = [
        "topo_node_test.cc",
    ],
    deps = [
        ":routing_topo_test_utils",
        "@gtest//:main",
    ],
)

cc_test(
    name = "topo_range_test",
    size = "small",
    srcs = [
        "topo_range_test.cc",
    ],
    deps = [
        ":routing_node_with_range",
        ":routing_range_utils",
        ":routing_topo_test_utils",
        "@gtest//:main",
    ],
)

cc_test(
    name = "topo_graph_test",
    size = "small",
    srcs = [
        "topo_graph_test.cc",
    ],
    deps = [
        ":routing_node_with_range",
        ":routing_range_utils",
        ":routing_topo_graph",
        ":routing_topo_test_utils",
        "@gtest//:main",
    ],
)

cc_test(
    name = "sub_topo_graph_test",
    size = "small",
    srcs = [
        "sub_topo_graph_test.cc",
    ],
    deps = [
        ":routing_node_with_range",
        ":routing_range_utils",
        ":routing_sub_topo_graph",
        ":routing_topo_graph",
        ":routing_topo_test_utils",
        "@gtest//:main",
    ],
)

cpplint()
