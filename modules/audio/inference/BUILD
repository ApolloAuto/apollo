load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "moving_detection",
    srcs = ["moving_detection.cc"],
    hdrs = ["moving_detection.h"],
    deps = [
        "@fftw3",
    ],
)

cc_library(
    name = "direction_detection",
    srcs = ["direction_detection.cc"],
    hdrs = ["direction_detection.h"],
    deps = [
        "//cyber",
        "//third_party:libtorch",
    ],
)

cc_test(
    name = "moving_detection_test",
    size = "small",
    srcs = ["moving_detection_test.cc"],
    deps = [
        ":moving_detection",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
