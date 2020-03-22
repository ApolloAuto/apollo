load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "edge_creator",
    srcs = ["edge_creator.cc"],
    hdrs = ["edge_creator.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        "//modules/routing/common:routing_gflags",
        "//modules/routing/proto:routing_proto",
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
        "//modules/routing/proto:routing_proto",
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
