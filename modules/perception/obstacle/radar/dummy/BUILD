load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_radar_dummy",
    srcs = [
        "dummy_algorithms.cc",
    ],
    hdrs = [
        "dummy_algorithms.h",
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
