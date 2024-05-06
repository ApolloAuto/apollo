load("//tools/proto:proto.bzl", "apollo_py_binary", "apollo_py_library")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

# install(
#     name = "install",
#     py_dest = "tools/routing",
#     targets = [
#         ":debug_passage_region",
#         ":debug_route",
#         ":road_show",
#     ],
# )

apollo_py_binary(
    name = "debug_passage_region",
    srcs = ["debug_passage_region.py"],
    deps = [
        ":debug_topo",
        "//modules/common_msgs/routing_msgs:routing_py_pb2",
        "//modules/routing/proto:topo_graph_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

apollo_py_binary(
    name = "debug_route",
    srcs = ["debug_route.py"],
    deps = [
        ":debug_topo",
        ":util",
        "//modules/routing/proto:topo_graph_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

apollo_py_library(
    name = "debug_topo",
    srcs = ["debug_topo.py"],
    deps = [
        ":util",
        "//modules/routing/proto:topo_graph_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

apollo_py_binary(
    name = "road_show",
    srcs = ["road_show.py"],
    deps = [
        ":util",
        "//modules/tools/common:proto_utils",
    ],
)

apollo_py_library(
    name = "util",
    srcs = ["util.py"],
    deps = [
        "//modules/common_msgs/map_msgs:map_py_pb2",
        "//modules/common_msgs/routing_msgs:routing_py_pb2",
        "//modules/routing/proto:topo_graph_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

apollo_package()