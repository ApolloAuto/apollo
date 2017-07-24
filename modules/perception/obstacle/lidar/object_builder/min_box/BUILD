load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_lidar_object_builder_min_box",
    srcs = ["min_box.cc"],
    hdrs = ["min_box.h"],
    deps = [
        "//modules/perception/lib/base:base",
        "//modules/perception/lib/pcl_util:pcl_util",
        "//modules/perception/common:perception_common",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/lidar/interface:perception_obstacle_lidar_interface",
        "//modules/common:log",
        "//modules/common:common",
        "@eigen//:eigen",
    ],
)

cpplint()
