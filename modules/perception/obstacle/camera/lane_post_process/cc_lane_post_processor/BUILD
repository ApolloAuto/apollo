load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_frame",
    srcs = ["lane_frame.cc"]
    hdrs = [
        "group.h",
        "lane_frame.h"
    ],
    deps = [
        "//modules/common:log",
        "@eigen//:eigen",
        "@opencv2//:core",
        "//modules/perception/obstacle/camera/lane_post_process/common:type",
        "//modules/perception/obstacle/camera/lane_post_process/common:util",
        "//modules/perception/obstacle/camera/lane_post_process/common:connected_component_analysis",
        "//modules/perception/obstacle/camera/lane_post_process/common:projector"
    ],
)

cc_library(
    name = "cc_lane_post_processor",
    srcs = ["cc_lane_post_processor.cc"],
    hdrs = ["cc_lane_post_processor.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/obstacle/camera/interface:perception_obstacle_camera_interface",
        "//modules/perception/obstacle/camera/lane_post_process/cc_lane_post_processor:cc_lane_post_processor",
        "@eigen//:eigen",
        "@opencv2//:core",
    ],
)

cpplint()
