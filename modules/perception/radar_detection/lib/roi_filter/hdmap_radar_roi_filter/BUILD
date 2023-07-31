load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "hdmap_radar_roi_filter_test",
    size = "small",
    srcs = ["hdmap_radar_roi_filter_test.cc"],
    deps = [
        ":hdmap_radar_roi_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "hdmap_radar_roi_filter",
    srcs = ["hdmap_radar_roi_filter.cc"],
    hdrs = ["hdmap_radar_roi_filter.h"],
    deps = [
        "//modules/perception/common/base",
        "//modules/perception/common/algorithm/geometry:roi_filter",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/radar_detection/interface:base_roi_filter",
        "@eigen",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
