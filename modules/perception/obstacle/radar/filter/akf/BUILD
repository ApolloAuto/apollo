load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_radar_filter_adaptive_kalman_filter",
    srcs = [
        "adaptive_kalman_filter.cc",
    ],
    hdrs = [
        "adaptive_kalman_filter.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/radar/interface:perception_obstacle_radar_interface",
        "@eigen//:eigen",
        "@pcl//:pcl",
    ],
)

cpplint()
