load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "timer",
    srcs = ["timer.cc"],
    hdrs = ["timer.h"],
    deps = [
        ":timing_wheel",
        "//cyber/common:global_data",
    ],
)

cc_library(
    name = "timer_task",
    hdrs = ["timer_task.h"],
)

cc_library(
    name = "timer_bucket",
    hdrs = ["timer_bucket.h"],
    deps = [
        ":timer_task",
    ],
)

cc_library(
    name = "timing_wheel",
    srcs = ["timing_wheel.cc"],
    hdrs = ["timing_wheel.h"],
    deps = [
        ":timer_bucket",
        "//cyber/task",
        "//cyber/time",
        "//cyber/time:duration",
        "//cyber/time:rate",
    ],
)

cc_test(
    name = "timer_test",
    size = "small",
    timeout = "moderate",
    srcs = ["timer_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber:init",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
