load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "graph",
    deps = [
        ":routing_topo_node",
        ":routing_topo_graph",
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
        "//modules/common/proto:common_proto",
        "//modules/map/proto:map_proto",
        "//modules/routing/proto:routing_proto",
        "//modules/common:common",
        "//modules/routing/common"
    ],
)

cpplint()
