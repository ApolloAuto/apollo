package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_hdmapinput",
    srcs = ["hdmap_input.cc"],
    hdrs = ["hdmap_input.h"],
    deps = [
        "//modules/perception/lib/base:base",
        "//modules/perception/lib/pcl_util:pcl_util",
        "//modules/perception/common:perception_common",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/common:log",
        "//modules/common:common",
        "//modules/map/hdmap:hdmap",
        "//external:gflags",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "perception_obstacle_lidar_process",
    srcs = ["lidar_process.cc"],
    hdrs = ["lidar_process.h"],
    deps = [
        ":perception_obstacle_hdmapinput",
        "//modules/perception/lib/base:base",
        "//modules/perception/lib/pcl_util:pcl_util",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/common:perception_common",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/lidar/dummy:perception_obstacle_lidar_dummy",
        "//modules/perception/obstacle/lidar/interface:perception_obstacle_lidar_interface",
        "//modules/common:log",
        "//modules/common:common",
        "//external:gflags",
        "@eigen//:eigen",
        "@ros//:ros_common",
    ],
)
