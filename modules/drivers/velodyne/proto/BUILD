load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_proto",
    deps = [
        ":sensor_velodyne_proto",
    ],
)

cc_proto_library(
    name = "sensor_velodyne_proto",
    deps = [
        ":velodyne_proto_lib",
    ],
)

proto_library(
    name = "velodyne_proto_lib",
    srcs = [
        "config.proto",
        "velodyne.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

py_proto(
    name = "velodyne_pb2",
    src = "velodyne.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "config_pb2",
    src = "config.proto",
    deps = [
        "//modules/drivers/velodyne/proto:velodyne_pb2",
    ],
)
