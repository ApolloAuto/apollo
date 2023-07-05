load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "test_helper",
    hdrs = ["test_helper.h"],
    deps = [
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
