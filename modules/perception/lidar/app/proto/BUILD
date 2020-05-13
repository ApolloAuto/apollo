load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library")
load("@com_github_grpc_grpc//bazel:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "app_cc_proto",
    deps = [
        ":lidar_obstacle_detection_config_cc_proto",
        ":lidar_obstacle_segmentation_config_cc_proto",
        ":lidar_obstacle_tracking_config_cc_proto",
    ],
)

cc_proto_library(
    name = "lidar_obstacle_detection_config_cc_proto",
    deps = [
        ":lidar_obstacle_detection_config_proto",
    ],
)

proto_library(
    name = "lidar_obstacle_detection_config_proto",
    srcs = ["lidar_obstacle_detection_config.proto"],
)

cc_proto_library(
    name = "lidar_obstacle_segmentation_config_cc_proto",
    deps = [
        ":lidar_obstacle_segmentation_config_proto",
    ],
)

proto_library(
    name = "lidar_obstacle_segmentation_config_proto",
    srcs = ["lidar_obstacle_segmentation_config.proto"],
)

cc_proto_library(
    name = "lidar_obstacle_tracking_config_cc_proto",
    deps = [
        ":lidar_obstacle_tracking_config_proto",
    ],
)

proto_library(
    name = "lidar_obstacle_tracking_config_proto",
    srcs = [
        "lidar_obstacle_tracking_config.proto",
    ],
)

py_proto_library(
    name = "lidar_obstacle_tracking_config_pb2",
    deps = [
        ":lidar_obstacle_tracking_config_proto",
    ],
)

py_proto_library(
    name = "lidar_obstacle_segmentation_config_pb2",
    deps = [":lidar_obstacle_segmentation_config_proto"],
)

py_proto_library(
    name = "lidar_obstacle_detection_config_pb2",
    deps = ["lidar_obstacle_detection_config_proto"],
)
