load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_common",
    srcs = [
        "file_system_util.cc",
        "geometry_util.cc",
        "graph_util.cc",
        "hungarian_bigraph_matcher.cc",
        "pose_util.cc",
        "object_sequence.cc"
    ],
    hdrs = [
        "convex_hullxy.h",
        "disjoint_set.h",
        "file_system_util.h",
        "geometry_util.h",
        "graph_util.h",
        "hungarian_bigraph_matcher.h",
        "pose_util.h",
        "object_sequence.h"
    ],
    linkopts = [
        "-lqhull",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/base",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "perception_obstacle_common_test",
    size = "small",
    srcs = [
        "convex_hullxy_test.cc",
        "disjoint_set_test.cc",
        "file_system_util_test.cc",
        "geometry_util_test.cc",
        "graph_util_test.cc",
        "hungarian_bigraph_matcher_test.cc",
        "object_sequence_test.cc",
        "pose_util_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
    ],
    deps = [
        ":perception_obstacle_common",
        "//modules/perception/obstacle/lidar/object_builder/min_box:perception_obstacle_lidar_object_builder_min_box",
        "@gtest//:main",
    ],
)

cpplint()
