load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "moving_detection",
    srcs = ["moving_detection.cc"],
    hdrs = ["moving_detection.h"],
)

cpplint()
