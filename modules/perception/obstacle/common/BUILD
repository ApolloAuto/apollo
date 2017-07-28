load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_common",
    srcs = [
        "geometry_util.cc",
        "graph_util.cc",
        "file_system_util.cc",
        "hungarian_bigraph_matcher.cc",
        "pose_util.cc",
    ],
    hdrs = [
        "convex_hullxy.h",
        "geometry_util.h",
        "disjoint_set.h",
        "circular_array.h",
        "file_system_util.h",
        "graph_util.h",
        "hungarian_bigraph_matcher.h",
        "pose_util.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/pcl_util:pcl_util",
        "//modules/perception/lib/base:base",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "perception_obstacle_common_test",
    size = "small",
    srcs = [
        "geometry_util_test.cc",
    ],
    deps = [
        ":perception_obstacle_common",
        "@gtest//:main"
    ],
)

cpplint()

