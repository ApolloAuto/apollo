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
        "//modules/common/math:math_utils",
        "//modules/common/configs:config_gflags",
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
