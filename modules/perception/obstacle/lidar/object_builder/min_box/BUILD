load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_lidar_object_builder_min_box",
    srcs = ["min_box.cc"],
    hdrs = ["min_box.h"],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/lidar/interface:perception_obstacle_lidar_interface",
        "@eigen",
    ],
)

cc_test(
    name = "min_box_test",
    size = "small",
    srcs = [
        "min_box_test.cc",
    ],
    data = ["//modules/perception:perception_data"],
    linkopts = [
        "-lqhull",
    ],
    deps = [
        ":perception_obstacle_lidar_object_builder_min_box",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "@gtest//:main",
    ],
)

cpplint()
