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
    ],
)

cc_binary(
    name = "topo_creator",
    srcs = ["topo_creator.cc"],
    deps = [
        "//external:gflags",
        ":graph_creator",
        "//modules/common:log",
        "//modules/routing/common",
        "//modules/control/proto:control_proto",
    ],
)

cpplint()
