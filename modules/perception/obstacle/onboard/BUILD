load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_hdmapinput",
    srcs = ["hdmap_input.cc"],
    hdrs = ["hdmap_input.h"],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/map/hdmap",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "perception_obstacle_lidar_process",
    srcs = ["lidar_process.cc"],
    hdrs = ["lidar_process.h"],
    deps = [
        ":perception_obstacle_hdmapinput",
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/lidar/dummy:perception_obstacle_lidar_dummy",
        "//modules/perception/obstacle/lidar/interface:perception_obstacle_lidar_interface",
        "@eigen//:eigen",
        "@ros//:ros_common",
    ],
)

cpplint()
