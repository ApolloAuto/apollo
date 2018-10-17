load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "task",
    hdrs = [
        "task.h",
    ],
    deps = [
        "//cybertron/task:task_manager",
    ],
)

cc_test(
    name = "task_test",
    size = "small",
    srcs = [
        "task_test.cc",
    ],
    deps = [
        "//cybertron:cybertron_core",
        "@gtest//:main",
    ],
)

cc_library(
    name = "task_manager",
    srcs = [
        "task_manager.cc",
    ],
    hdrs = [
        "task_manager.h",
    ],
    deps = [
        "//cybertron/scheduler:scheduler",
    ],
)

cpplint()
