load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "third_party_perception_proto",
    deps = [
        ":third_party_perception_proto_lib",
    ],
)

proto_library(
    name = "third_party_perception_proto_lib",
    srcs = ["radar_obstacle.proto"],
    deps = [
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

py_proto(
    name = "radar_obstacle_pb2",
    src = "radar_obstacle.proto",
    deps = [
        "//modules/common/proto:error_code_pb2",
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:header_pb2",
    ],
)
