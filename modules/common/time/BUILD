load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "time",
    srcs = [
        "timer.cc",
    ],
    hdrs = [
        "clock.h",
        "time_util.h",
        "timer.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/common/configs:config_gflags",
        "@ros//:ros_common",
    ],
)

cc_test(
    name = "clock_test",
    size = "small",
    srcs = [
        "clock_test.cc",
    ],
    deps = [
        ":time",
        "@gtest//:main",
    ],
)

cc_test(
    name = "timer_test",
    size = "small",
    srcs = [
        "timer_test.cc",
    ],
    deps = [
        ":time",
        "@gtest//:main",
    ],
)

cpplint()
