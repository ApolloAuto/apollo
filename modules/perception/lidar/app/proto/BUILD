load("//tools:cpplint.bzl", "cpplint")

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
        "lidar_obstacle_segmentation_config.proto",
        "lidar_obstacle_tracking_config.proto",
    ],
)

cpplint()
