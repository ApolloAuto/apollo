load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "offline_lidar_perception_tool",
    srcs = ["offline_lidar_perception_tool.cc"],
    data = [
        "conf/config_manager.config",
        "conf/offline_lidar_perception_test.flag",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/lidar/visualizer:perception_obstacle_lidar_visualizer",
        "//modules/perception/obstacle/onboard:perception_obstacle_lidar_process",
        "@eigen//:eigen",
    ],
)