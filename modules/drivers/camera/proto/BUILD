load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_proto",
    deps = [
        ":sensor_camera_proto",
    ],
)

cc_proto_library(
    name = "sensor_camera_proto",
    deps = [
        ":camera_proto_lib",
    ],
)

proto_library(
    name = "camera_proto_lib",
    srcs = ["config.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

py_proto(
    name = "config_pb2",
    src = "config.proto",
)
