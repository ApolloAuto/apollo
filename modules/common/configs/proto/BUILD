load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "vehicle_config_proto",
    deps = [
        ":vehicle_config_proto_lib",
    ],
)

proto_library(
    name = "vehicle_config_proto_lib",
    srcs = ["vehicle_config.proto"],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

py_proto(
    name = "vehicle_config_pb2",
    src = "vehicle_config.proto",
    deps = [
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:header_pb2",
    ],
)
