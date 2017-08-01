load("//tools:cpplint.bzl", "cpplint")

load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_lidar_visualizer",
    srcs = [
        "pcl_obstacle_visualizer.cc",
        "pcl_vis_util.cc",
        ],
    hdrs = [
        "pcl_obstacle_visualizer.h",
        "pcl_vis_util.h",
        ],
    deps = [
        "//modules/perception/lib/pcl_util:pcl_util",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/lib/base:base",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "@eigen//:eigen",
        "@pcl//:pcl",
        "@vtk//:vtk",
    ],
)

cpplint()
