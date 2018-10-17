load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "timer_lib",
    srcs = glob([
        "timer.cc",
        "timer_manager.cc",
        "timer_task.cc",
        "timing_slot.cc",
        "timing_wheel.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
          "//cybertron/base:thread_pool",
          "//cybertron/common:common",
          "//cybertron/croutine:croutine",
          "//cybertron/task:task",
          "//cybertron/time:rate",
          "//cybertron/time:time",
    ],
)

cc_test(
    name = "timer_manager_test",
    size = "small",
    srcs = [
        "timer_manager_test.cc",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "timing_wheel_test",
    size = "small",
    srcs = [
        "timing_wheel_test.cc",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cpplint()
