load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "min_box",
    srcs = ["min_box.cc"],
    hdrs = ["min_box.h"],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/common",
        "//modules/perception/common:convex_hullxy",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/lidar/interface",
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
        ":min_box",
        "//modules/perception/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/obstacle/common",
        "@gtest//:main",
    ],
)

cpplint()
