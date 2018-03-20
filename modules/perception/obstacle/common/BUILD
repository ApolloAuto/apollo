load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    srcs = [
        "hungarian_bigraph_matcher.cc",
        "pose_util.cc",
    ],
    hdrs = [
        "hungarian_bigraph_matcher.h",
        "pose_util.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/util",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
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
        "hungarian_bigraph_matcher_test.cc",
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
