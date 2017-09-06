load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "digital_filter",
    srcs = [
        "digital_filter.cc",
    ],
    hdrs = [
        "digital_filter.h",
    ],
    deps = [
        "//modules/common:log",
    ],
)

cc_library(
    name = "mean_filter",
    srcs = [
        "mean_filter.cc",
    ],
    hdrs = [
        "mean_filter.h",
    ],
    deps = [
        "//modules/common:log",
    ],
)

cc_library(
    name = "filters",
    deps = [
        ":digital_filter",
        ":mean_filter",
    ],
)

cc_test(
    name = "digital_filter_test",
    size = "small",
    srcs = [
        "digital_filter_test.cc",
    ],
    deps = [
        ":digital_filter",
        "//modules/common",
        "@gtest//:main",
        "@ros//:ros_common",
    ],
)

cc_test(
    name = "mean_filter_test",
    size = "small",
    srcs = [
        "mean_filter_test.cc",
    ],
    deps = [
        ":mean_filter",
        "@gtest//:main",
    ],
)

cpplint()
