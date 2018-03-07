load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    srcs = [
        "geometry_util.cc",
        "graph_util.cc",
        "hungarian_bigraph_matcher.cc",
        "object_sequence.cc",
        "pose_util.cc",
    ],
    hdrs = [
        "camera.h",
        "convex_hullxy.h",
        "disjoint_set.h",
        "geometry_util.h",
        "graph_util.h",
        "hungarian_bigraph_matcher.h",
        "object_sequence.h",
        "pose_util.h",
    ],
    linkopts = [
        "-lqhull",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/base",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/base",
        "@eigen",
        "@opencv2//:core",
        "@yaml_cpp//:yaml",
    ],
)

cc_test(
    name = "common_test",
    size = "small",
    srcs = [
        "convex_hullxy_test.cc",
        "disjoint_set_test.cc",
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
        ":common",
        "//modules/perception/obstacle/lidar/object_builder/min_box",
        "@gtest//:main",
    ],
)

cpplint()
