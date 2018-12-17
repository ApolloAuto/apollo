load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "timer",
    srcs = ["timer.cc"],
    hdrs = ["timer.h"],
    deps = [
        "timer_manager",
        "//cyber/common:global_data",
    ],
)

cc_library(
    name = "timer_manager",
    srcs = ["timer_manager.cc"],
    hdrs = ["timer_manager.h"],
    deps = [
        "timing_wheel",
        "//cyber/common:macros",
        "//cyber/scheduler",
        "//cyber/task",
        "//cyber/time",
        "//cyber/time:rate",
    ],
)

cc_test(
    name = "timer_manager_test",
    size = "small",
    srcs = ["timer_manager_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@gtest//:main",
    ],
)

cc_library(
    name = "timer_task",
    srcs = ["timer_task.cc"],
    hdrs = ["timer_task.h"],
    deps = [
        "//cyber/base:bounded_queue",
        "//cyber/task",
    ],
)

cc_library(
    name = "timing_slot",
    srcs = ["timing_slot.cc"],
    hdrs = ["timing_slot.h"],
    deps = [
        "timer_task",
        "//cyber/base:bounded_queue",
    ],
)

cc_library(
    name = "timing_wheel",
    srcs = ["timing_wheel.cc"],
    hdrs = ["timing_wheel.h"],
    deps = [
        "timer_task",
        "timing_slot",
        "//cyber/base:bounded_queue",
        "//cyber/task",
        "//cyber/time",
        "//cyber/time:duration",
    ],
)

cc_test(
    name = "timing_wheel_test",
    size = "small",
    srcs = ["timing_wheel_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber:init",
        "@gtest//:main",
    ],
)

cpplint()
