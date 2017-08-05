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
    ],
    hdrs = [
        "convex_hullxy.h",
        "disjoint_set.h",
        "file_system_util.h",
        "geometry_util.h",
        "graph_util.h",
        "hungarian_bigraph_matcher.h",
        "pose_util.h",
    ],
    linkopts = [
        "-lqhull",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/base",
        "//modules/perception/lib/pcl_util",
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
        "@gtest//:main",
    ],
)

cpplint()
