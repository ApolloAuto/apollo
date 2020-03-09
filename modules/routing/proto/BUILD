load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "routing_proto",
    deps = [
        ":routing_proto_lib",
    ],
)

proto_library(
    name = "routing_proto_lib",
    srcs = [
        "poi.proto",
        "routing.proto",
        "routing_config.proto",
        "topo_graph.proto",
    ],
    deps = [
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/map/proto:map_proto_lib",
    ],
)

py_proto(
    name = "poi_pb2",
    src = "poi.proto",
    deps = [
        "//modules/routing/proto:routing_pb2",
    ],
)

py_proto(
    name = "routing_config_pb2",
    src = "routing_config.proto",
)

py_proto(
    name = "routing_pb2",
    src = "routing.proto",
    deps = [
        "//modules/common/proto:error_code_pb2",
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:header_pb2",
        "//modules/map/proto:map_parking_space_pb2",
    ],
)

py_proto(
    name = "topo_graph_pb2",
    src = "topo_graph.proto",
    deps = [
        "//modules/map/proto:map_geometry_pb2",
    ],
)
