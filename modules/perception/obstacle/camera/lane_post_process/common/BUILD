load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_camera_lane_post_process_common_type",
    hdrs = ["type.h"],
    deps = [
        "//modules/common:log",
        "@eigen//:eigen",
        "@opencv2//:core",
    ],
)

cc_library(
    name = "perception_obstacle_camera_lane_post_process_common_util",
    hdrs = ["util.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/obstacle/camera/lane_post_process/common:perception_obstacle_camera_lane_post_process_common_type",
    ],
)

cc_library(
    name = "perception_obstacle_camera_lane_post_process_common_connected_component_analysis",
    srcs = ["connected_component.cc"],
    hdrs = ["connected_component.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/obstacle/camera/lane_post_process/common:perception_obstacle_camera_lane_post_process_common_type",
        "@eigen//:eigen",
        "@opencv2//:core",
    ],
)

cpplint()
