load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hesai_proto",
    deps = [
        ":sensor_hesai_proto",
    ],
)

cc_proto_library(
    name = "sensor_hesai_proto",
    deps = [
        ":hesai_proto_lib",
    ],
)

proto_library(
    name = "hesai_proto_lib",
    srcs = [
        "config.proto",
        "hesai.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

py_proto(
    name = "hesai_pb2",
    src = "hesai.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "config_pb2",
    src = "config.proto",
    deps = [
        "//modules/drivers/hesai/proto:hesai_pb2",
    ],
)
