load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "singleton",
    srcs = [],
    hdrs = ["singleton.h"],
    deps = [
        "//cyber",
    ],
)

cc_test(
    name = "singleton_test",
    size = "small",
    srcs = [
        "singleton_test.cc",
    ],
    deps = [
        "@gtest//:main",
        ":singleton",
    ],
)

cpplint()
