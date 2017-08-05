load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "edge_creator",
    srcs = [
        "edge_creator.cc",
    ],
    hdrs = [
        "edge_creator.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/routing/proto:routing_proto",
        "//modules/map/proto:map_proto",
        "//modules/common:common",
        "//modules/routing/common"
    ],
)

cc_library(
    name = "graph_creator",
    srcs = [
        "graph_creator.cc",
    ],
    hdrs = [
        "graph_creator.h",
    ],
    deps = [
        "//external:gflags",
        ":edge_creator",
        ":node_creator",
        "//modules/routing/proto:routing_proto",
        "//modules/map/proto:map_proto",
        "//modules/common:common",
        "//modules/common/util",
        "//modules/routing/common"
    ],
)

cc_library(
    name = "node_creator",
    srcs = [
        "node_creator.cc",
    ],
    hdrs = [
        "node_creator.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/routing/proto:routing_proto",
        "//modules/map/proto:map_proto",
        "//modules/common:common",
        "//modules/routing/common"
    ],
)

cc_binary(
    name = "topo_creator",
    srcs = ["topo_creator.cc"],
    deps = [
        "//external:gflags",
        ":graph_creator",
        "//modules/common:log",
        "//modules/routing/proto:routing_proto",
        "//modules/map/proto:map_proto",
        "//modules/common:common",
        "//modules/routing/common"
    ],
)

cpplint()
