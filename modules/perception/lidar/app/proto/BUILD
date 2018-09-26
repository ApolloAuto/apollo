load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "lidar_obstacle_segmentation_config_proto",
    deps = [
        ":lidar_obstacle_segmentation_config_proto_lib",
    ],
)

proto_library(
    name = "lidar_obstacle_segmentation_config_proto_lib",
    srcs = [
        "lidar_obstacle_segmentation_config.proto",
    ],
)

cc_proto_library(
    name = "lidar_obstacle_tracking_config_proto",
    deps = [
        ":lidar_obstacle_tracking_config_proto_lib",
    ],
)

proto_library(
    name = "lidar_obstacle_tracking_config_proto_lib",
    srcs = [
        "lidar_obstacle_tracking_config.proto",
    ],
)

cpplint()
