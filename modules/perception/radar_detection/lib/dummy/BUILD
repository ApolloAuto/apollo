load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "dummy_algorithms_test",
    size = "small",
    srcs = ["dummy_algorithms_test.cc"],
    deps = [
        ":dummy_algorithms",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "dummy_algorithms",
    srcs = ["dummy_algorithms.cc"],
    hdrs = ["dummy_algorithms.h"],
    deps = [
        "//modules/perception/common/base",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/radar_detection/common:types",
        "//modules/perception/radar_detection/interface:base_detector",
        "//modules/perception/radar_detection/interface:base_preprocessor",
        "//modules/perception/radar_detection/interface:base_roi_filter",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
