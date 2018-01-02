load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_lidar_type_fuser_sequence_type_fuser",
    srcs = [
        "sequence_type_fuser.cc",
        "util.cc"
    ],
    hdrs = [
        "sequence_type_fuser.h",
        "util.h"
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/lib/config_manager",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/lidar/interface:perception_obstacle_lidar_interface",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "sequence_type_fuser_test",
    size = "small",
    srcs = [
        "sequence_type_fuser_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
        "//modules/perception/conf:perception_config",
    ],
    linkopts = [
        "-lqhull",
    ],
    deps = [
        ":perception_obstacle_lidar_type_fuser_sequence_type_fuser",
        "//modules/common:log",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/lidar/object_builder/min_box:perception_obstacle_lidar_object_builder_min_box",
        "//modules/perception/obstacle/lidar/tracker/hm_tracker:perception_obstacle_lidar_tracker_hm_tracker",
        "@gtest//:main",
        "@pcl//:pcl",
    ],
)

cpplint()
