load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    srcs = [
        "file_system_util.cc",
        "geometry_util.cc",
        "graph_util.cc",
        "perception_gflags.cc",
    ],
    hdrs = [
        "file_system_util.h",
        "geometry_util.h",
        "graph_util.h",
        "perception_gflags.h",
    ],
    deps = [
        ":pcl_util",
        "//modules/common",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:math_utils",
        "@eigen",
    ],
)

cc_library(
    name = "pcl_util",
    srcs = [],
    hdrs = ["pcl_types.h"],
    deps = [
        "@pcl",
    ],
)

cc_library(
    name = "convex_hullxy",
    srcs = [],
    hdrs = ["convex_hullxy.h"],
    linkopts = [
        "-lqhull",
    ],
    deps = [
        "@pcl",
    ],
)

cc_library(
    name = "mem_util",
    srcs = ["mem_util.cc"],
    hdrs = ["mem_util.h"],
    deps = [
        "//modules/common:log",
        "@caffe//:lib",
        "@cuda",
        "@eigen",
        "@opencv2//:core",
        "@opencv2//:imgproc",
        "@undistort",
    ],
)

cc_test(
    name = "convex_hullxy_test",
    size = "small",
    srcs = [
        "convex_hullxy_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
    ],
    deps = [
        ":convex_hullxy",
        ":pcl_util",
        "//modules/common:log",
        "@eigen",
        "@gtest//:main",
    ],
)

cc_test(
    name = "common_test",
    size = "small",
    srcs = [
        "file_system_util_test.cc",
        "geometry_util_test.cc",
        "graph_util_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
    ],
    deps = [
        ":common",
        "//modules/perception/obstacle/lidar/object_builder/min_box",
        "@eigen",
        "@gtest//:main",
    ],
)

cpplint()
