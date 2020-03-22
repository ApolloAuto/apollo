load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "multi_lidar_fusion_config_proto",
    deps = [
        ":multi_lidar_fusion_config_lib",
    ],
)

proto_library(
    name = "multi_lidar_fusion_config_lib",
    srcs = ["multi_lidar_fusion_config.proto"],
)

py_proto(
    name = "multi_lidar_fusion_config_pb2",
    src = "multi_lidar_fusion_config.proto",
)
