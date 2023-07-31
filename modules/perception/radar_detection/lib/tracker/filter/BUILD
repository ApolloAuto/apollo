load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "adaptive_kalman_filter_test",
    size = "small",
    srcs = ["adaptive_kalman_filter_test.cc"],
    deps = [
        ":adaptive_kalman_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "adaptive_kalman_filter",
    srcs = ["adaptive_kalman_filter.cc"],
    hdrs = ["adaptive_kalman_filter.h"],
    deps = [
        "//modules/perception/common/base",
        "//modules/perception/radar_detection/interface:base_filter",
        "@eigen",
    ],
)

apollo_package()
cpplint()
