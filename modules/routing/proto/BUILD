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
