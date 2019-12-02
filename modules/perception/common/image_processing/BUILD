load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hough_transfer",
    srcs = ["hough_transfer.cc"],
    hdrs = ["hough_transfer.h"],
)

cc_test(
    name = "hough_transfer_test",
    size = "small",
    srcs = ["hough_transfer_test.cc"],
    deps = [
        ":hough_transfer",
        "@gtest//:main",
    ],
)

cpplint()
