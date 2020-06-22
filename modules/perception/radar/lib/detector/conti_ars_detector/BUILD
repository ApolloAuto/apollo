load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "conti_ars_detector_test",
    size = "small",
    srcs = ["conti_ars_detector_test.cc"],
    deps = [
        ":conti_ars_detector",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "conti_ars_detector",
    srcs = ["conti_ars_detector.cc"],
    hdrs = ["conti_ars_detector.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/common/geometry:roi_filter",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/radar/common:radar_util",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/lib/interface:base_detector",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
