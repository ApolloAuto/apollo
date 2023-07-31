load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
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
        "//modules/perception/common/base",
        "//modules/perception/common/algorithm/geometry:roi_filter",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/radar_detection/common:radar_util",
        "//modules/perception/radar_detection/common:types",
        "//modules/perception/radar_detection/interface:base_detector",
        "@eigen",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
