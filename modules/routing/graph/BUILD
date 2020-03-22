load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "graph",
    deps = [
        ":routing_sub_topo_graph",
        ":routing_topo_graph",
        ":routing_topo_range_manager",
    ],
)

cc_library(
    name = "routing_topo_range",
    srcs = ["topo_range.cc"],
    hdrs = ["topo_range.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        "//modules/routing/common:routing_gflags",
    ],
)

cc_library(
    name = "routing_topo_range_manager",
    srcs = ["topo_range_manager.cc"],
    hdrs = ["topo_range_manager.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        ":routing_topo_node",
    ],
)

cc_library(
    name = "routing_range_utils",
    hdrs = ["range_utils.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
)

cc_library(
    name = "routing_node_with_range",
    srcs = ["node_with_range.cc"],
    hdrs = ["node_with_range.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        ":routing_topo_node",
    ],
)

cc_library(
    name = "routing_topo_node",
    srcs = ["topo_node.cc"],
    hdrs = ["topo_node.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        ":routing_range_utils",
        ":routing_topo_range",
        "//cyber",
        "//modules/common/util:map_util",
        "//modules/routing/proto:routing_proto",
    ],
)

cc_library(
    name = "routing_topo_graph",
    srcs = ["topo_graph.cc"],
    hdrs = ["topo_graph.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        ":routing_topo_node",
    ],
)

cc_library(
    name = "routing_sub_topo_graph",
    srcs = ["sub_topo_graph.cc"],
    hdrs = ["sub_topo_graph.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        ":routing_node_with_range",
    ],
)

cc_library(
    name = "routing_topo_test_utils",
    srcs = ["topo_test_utils.cc"],
    hdrs = ["topo_test_utils.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        ":routing_topo_graph",
    ],
)

cc_test(
    name = "topo_node_test",
    size = "small",
    srcs = ["topo_node_test.cc"],
    deps = [
        ":routing_topo_test_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "topo_range_test",
    size = "small",
    srcs = ["topo_range_test.cc"],
    deps = [
        ":routing_node_with_range",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "topo_graph_test",
    size = "small",
    srcs = ["topo_graph_test.cc"],
    deps = [
        ":routing_topo_test_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "sub_topo_graph_test",
    size = "small",
    srcs = ["sub_topo_graph_test.cc"],
    deps = [
        ":routing_sub_topo_graph",
        ":routing_topo_test_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
