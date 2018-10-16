load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "timer_lib",
    srcs = glob([
        "timer.cpp",
        "timer_manager.cpp",
        "timer_task.cpp",
        "timing_slot.cpp",
        "timing_wheel.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
          "//cybertron/base:thread_pool",
          "//cybertron/common:common",
          "//cybertron/croutine:croutine",
          "//cybertron/data:data_lib",
          "//cybertron/event:event_lib",
          "//cybertron/scheduler:scheduler_lib",
          "//cybertron/task:task",
          "//cybertron/time:rate",
          "//cybertron/time:time",
    ],
)

cc_test(
    name = "timer_manager_test",
    size = "small",
    srcs = [
        "timer_manager_test.cpp",
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
        "timing_wheel_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cpplint()
