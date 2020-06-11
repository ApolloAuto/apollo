load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "basic",
    hdrs = ["basic.h"],
    deps = [
        ":common",
        "@eigen",
    ],
)

cc_library(
    name = "common",
    hdrs = ["common.h"],
    deps = [
        "//modules/perception/base:box",
        "//modules/perception/base:point_cloud",
    ],
)

cc_library(
    name = "convex_hull_2d",
    hdrs = ["convex_hull_2d.h"],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "roi_filter",
    srcs = ["roi_filter.cc"],
    hdrs = ["roi_filter.h"],
    deps = [
        ":common",
        "//modules/perception/base:base_type",
        "//modules/perception/base:object",
        "//modules/perception/base:point_cloud",
        "@eigen",
    ],
)

cc_library(
    name = "camera_homography",
    srcs = ["camera_homography.cc"],
    hdrs = ["camera_homography.h"],
    deps = [
        "//modules/perception/base:camera",
        "@eigen",
    ],
)

cc_test(
    name = "basic_test",
    size = "small",
    srcs = ["basic_test.cc"],
    deps = [
        ":basic",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "camera_homography_test",
    size = "small",
    srcs = ["camera_homography_test.cc"],
    deps = [
        ":camera_homography",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "roi_filter_test",
    size = "small",
    srcs = ["roi_filter_test.cc"],
    deps = [
        ":roi_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "convex_hull_2d_test",
    size = "small",
    srcs = ["convex_hull_2d_test.cc"],
    deps = [
        ":convex_hull_2d",
        "//modules/perception/base:point_cloud",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
