load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    py_dest = "tools/routing",
    targets = [
        ":debug_passage_region",
        ":debug_route",
        ":road_show",
    ],
)

py_binary(
    name = "debug_passage_region",
    srcs = ["debug_passage_region.py"],
    deps = [
        ":debug_topo",
        "//modules/common_msgs/routing_msgs:routing_py_pb2",
        "//modules/routing/proto:topo_graph_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

py_binary(
    name = "debug_route",
    srcs = ["debug_route.py"],
    deps = [
        ":debug_topo",
        ":util",
        "//modules/routing/proto:topo_graph_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

py_library(
    name = "debug_topo",
    srcs = ["debug_topo.py"],
    deps = [
        ":util",
        "//modules/routing/proto:topo_graph_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

py_binary(
    name = "road_show",
    srcs = ["road_show.py"],
    deps = [
        ":util",
        "//modules/tools/common:proto_utils",
    ],
)

py_library(
    name = "util",
    srcs = ["util.py"],
    deps = [
        "//modules/common_msgs/map_msgs:map_py_pb2",
        "//modules/common_msgs/routing_msgs:routing_py_pb2",
        "//modules/routing/proto:topo_graph_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)
