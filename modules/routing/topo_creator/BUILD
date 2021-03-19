load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "edge_creator",
    srcs = ["edge_creator.cc"],
    hdrs = ["edge_creator.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        "//modules/routing/common:routing_gflags",
        "//modules/routing/proto:routing_cc_proto",
        "//modules/routing/proto:routing_config_cc_proto",
        "//modules/routing/proto:topo_graph_cc_proto",
    ],
)

cc_library(
    name = "graph_creator",
    srcs = ["graph_creator.cc"],
    hdrs = ["graph_creator.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        ":edge_creator",
        ":node_creator",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/map/hdmap/adapter:opendrive_adapter",
    ],
)

cc_test(
    name = "graph_creator_test",
    size = "small",
    srcs = ["graph_creator_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":graph_creator",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "node_creator",
    srcs = ["node_creator.cc"],
    hdrs = ["node_creator.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        "//modules/map/proto:map_lane_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
        "//modules/routing/proto:routing_config_cc_proto",
        "//modules/routing/proto:topo_graph_cc_proto",
    ],
)

cc_binary(
    name = "topo_creator",
    srcs = ["topo_creator.cc"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        ":graph_creator",
        "//modules/map/hdmap:hdmap_util",
    ],
)

cpplint()
