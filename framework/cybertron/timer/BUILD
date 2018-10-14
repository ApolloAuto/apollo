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
          "//cybertron/common:common_lib",
          "//cybertron/croutine:croutine_lib",
          "//cybertron/data:data_lib",
          "//cybertron/event:event_lib",
          "//cybertron/scheduler:scheduler_lib",
          "//cybertron/task:task_lib",
          "//cybertron/time:time_lib",
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
