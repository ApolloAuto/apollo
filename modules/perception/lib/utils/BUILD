load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "utils",
    deps = [
        ":perception_perf",
        ":perception_time_util",
        ":perception_timer",
    ],
)

cc_library(
    name = "perception_perf",
    hdrs = ["perf.h"],
)

cc_library(
    name = "perception_timer",
    srcs = ["timer.cc"],
    hdrs = ["timer.h"],
    deps = [
        ":perception_perf",
        "//cyber",
    ],
)

cc_test(
    name = "perception_timer_test",
    size = "small",
    srcs = ["timer_test.cc"],
    deps = [
        ":perception_timer",
        "@gtest//:main",
    ],
)

cc_library(
    name = "perception_time_util",
    hdrs = ["time_util.h"],
)

cc_test(
    name = "perception_time_util_test",
    size = "small",
    srcs = ["time_util_test.cc"],
    deps = [
        ":perception_time_util",
        "@gtest//:main",
    ],
)

cpplint()
