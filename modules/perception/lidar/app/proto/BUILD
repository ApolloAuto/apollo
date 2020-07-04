load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "app_proto",
    deps = [
        ":app_proto_lib",
    ],
)

proto_library(
    name = "app_proto_lib",
    srcs = [
        "lidar_obstacle_detection_config.proto",
        "lidar_obstacle_segmentation_config.proto",
        "lidar_obstacle_tracking_config.proto",
    ],
)

py_proto(
    name = "lidar_obstacle_tracking_config_pb2",
    src = "lidar_obstacle_tracking_config.proto",
)

py_proto(
    name = "lidar_obstacle_segmentation_config_pb2",
    src = "lidar_obstacle_segmentation_config.proto",
)

py_proto(
    name = "lidar_obstacle_detection_config_pb2",
    src = "lidar_obstacle_detection_config.proto",
)
