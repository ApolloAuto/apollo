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
        "//modules/perception/obstacle/camera/lane_post_process/common:type",
    ],
)

cc_library(
    name = "perception_obstacle_camera_lane_post_process_common_connected_component_analysis",
    srcs = ["connected_component.cc"],
    hdrs = ["connected_component.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/obstacle/camera/lane_post_process/common:type",
        "@eigen//:eigen",
        "@opencv2//:core",
    ],
)

cc_library(
    name = "projector",
    hdrs = ["projector.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/obstacle/camera/common:perception_obstacle_camera_common",
        "@eigen//:eigen",
        "@opencv2//:core",
    ],
)

cpplint()
